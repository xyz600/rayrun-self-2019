#pragma once

#include "bounding_box.hpp"
#include "ray.hpp"
#include "task_queue.hpp"
#include "triangle.hpp"
#include "vec.hpp"
#include "fixed_vector.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <set>
#include <vector>

using vector::Vec3;

class SimpleBVH {
public:
	SimpleBVH();
	bool construct(MeshTriangleList &&mesh_list);
	bool intersect(RayExt &ray) const;
	bool intersectAny(RayExt &ray) const;

	using meshid_index_t = PackedTrianglex8::meshid_index_t;
	using node_index_t = std::int32_t;

private:

	struct LeafNode {
		node_index_t leaf_meshid_from;
		node_index_t leaf_meshid_to;
		node_index_t start_mesh_index;

		LeafNode()
			: leaf_meshid_from(0)
			, leaf_meshid_to(0)
			, start_mesh_index(0) {}

		LeafNode(node_index_t leaf_meshid_from, node_index_t leaf_meshid_to, node_index_t start_mesh_index)
			: leaf_meshid_from(leaf_meshid_from), leaf_meshid_to(leaf_meshid_to), start_mesh_index(start_mesh_index) {}

		LeafNode(LeafNode &&src)
			: leaf_meshid_from(src.leaf_meshid_from), leaf_meshid_to(src.leaf_meshid_to), start_mesh_index(src.start_mesh_index) {}

		void operator=(LeafNode &&src)
		{
			leaf_meshid_from = src.leaf_meshid_from;
			leaf_meshid_to = src.leaf_meshid_to;
			start_mesh_index = src.start_mesh_index;
		}

		void operator=(const LeafNode &src)
		{
			leaf_meshid_from = src.leaf_meshid_from;
			leaf_meshid_to = src.leaf_meshid_to;
			start_mesh_index = src.start_mesh_index;
		}
	};

	struct Node {
	public:
		node_index_t parent;
		node_index_t self;
		node_index_t left;
		node_index_t right;

		// AABB
		AABB aabb;

		Node() noexcept { clear(); }

		Node(std::int32_t self) noexcept : self(self) { clear(); }

		void clear() noexcept {
			left = right = parent = SimpleBVH::Invalid;
			aabb.clear();
		}

		void set_leaf(const std::size_t leaf_from,
			const std::size_t leaf_to, LeafNode &leaf_node) noexcept {
			leaf_node.leaf_meshid_from = leaf_from;
			leaf_node.leaf_meshid_to = leaf_to;
		}

		node_index_t leaf_index() const noexcept {
#ifdef DEBUG
			assert(is_leaf());
#endif
			return -left;
		}

		void set_leaf_index(const node_index_t leaf_index) noexcept {
			left = -leaf_index;
		}

		bool is_leaf() const noexcept {
			return left < 0;
		}

		void copy_from(const Node &src) noexcept {
			self = src.self;
			left = src.left;
			right = src.right;
			parent = src.parent;
			aabb = src.aabb;
		}
	};

	struct Nodex8 {

		using leaf_index_t = std::uint8_t;

		static constexpr leaf_index_t LeafID = std::numeric_limits<leaf_index_t>::max();

		Nodex8(const std::vector<Node *> &node_list) {
			std::vector<AABB *> aabb_list;
			for (std::size_t i = 0; i < 8; i++) {
				const std::size_t index = std::min(i, node_list.size() - 1);
				aabb_list.push_back(&node_list[index]->aabb);
			}
			aabb.construct(aabb_list);
			node_size_in_children = node_list.size();
		}

		Nodex8(Nodex8 &&src) {
			self = src.self;
			std::copy(src.children.begin(), src.children.end(), children.begin());
			node_size_in_children = src.node_size_in_children;
			aabb = std::forward<PackedAABBx8>(src.aabb);
		}

		void operator=(Nodex8 &&src) {
			self = src.self;
			std::copy(src.children.begin(), src.children.end(), children.begin());
			node_size_in_children = src.node_size_in_children;
			aabb = std::forward<PackedAABBx8>(src.aabb);
		}

		bool is_leaf(const std::size_t index) const noexcept {
			return children[index] < 0;
		}

		node_index_t leaf_index(const std::size_t index) const noexcept {
#ifdef DEBUG
			assert(is_leaf(index));
#endif
			return -children[index];
		};

		PackedAABBx8 aabb;
		std::array<node_index_t, 8> children;
		node_index_t self;
		node_index_t node_size_in_children;
	};

	struct Range {

		meshid_index_t from;
		meshid_index_t to;
		meshid_index_t node_index;

		Range(std::int32_t from, std::int32_t to, std::int32_t node_index)
			: from(from), to(to), node_index(node_index) {}
		std::size_t size() const noexcept { return to - from; }
	};

	bool intersectSub(std::int32_t nodeIndex, RayExt &ray,
		meshid_index_t *hitMeshIndex) const;
	bool intersectAnySub(node_index_t nodeIndex, RayExt &ray,
		meshid_index_t *hitMeshIndex) const;

	void adjust_index(std::array<std::vector<meshid_index_t>, 3> &index_sorted_by,
		std::vector<std::uint8_t> &is_left_flag, const Range &range,
		const std::int32_t axis, const std::int32_t position);

	void select_best_split(
		const Range &range,
		const std::array<std::vector<meshid_index_t>, 3> &index_sorted_by,
		std::vector<AABB> &left_accumulated_AABB,
		std::vector<AABB> &right_accumulated_AABB, std::int32_t *best_axis,
		std::int32_t *best_position);

	node_index_t remove_node(node_index_t node_id) noexcept;

	void refit_for_insertion(node_index_t node_id) noexcept;

	void reorder_mesh(const std::vector<meshid_index_t> &mesh_index_list) noexcept;

	void reorder_node(const std::size_t max_depth) noexcept;

	void reorder_leafnode() noexcept;

	void optimize_through_insertion() noexcept;

	void insert_node(node_index_t new_node_id, node_index_t node_id, node_index_t unnecessary_node) noexcept;

	node_index_t search_insert_point(node_index_t node_id) const noexcept;

	void visit_for_reorder(node_index_t old_node_index, std::int32_t depth,
		std::int32_t max_depth, node_index_t &new_node_index,
		std::queue<node_index_t> &que,
		std::vector<node_index_t> &order);

	void collect_node_to_pack(node_index_t old_node_index, std::vector<node_index_t> &children);

	void setup_packed_node() noexcept;

	void setup_packed_triangle() noexcept;

	static constexpr std::int32_t AxisX = 0;
	static constexpr std::int32_t AxisY = 1;
	static constexpr std::int32_t AxisZ = 2;

	static constexpr std::uint8_t Left = 0;
	static constexpr std::uint8_t Right = 1;

	static constexpr node_index_t Invalid =
		std::numeric_limits<node_index_t>::max();

	static constexpr float CostBoundingBoxIntersect = 1.0;
	static constexpr float CostPolygonIntersect = 1.0;

	static constexpr std::int32_t leaf_size_threashold = 16;

	void validate_range(const std::array<std::vector<meshid_index_t>, 3> &mesh_id_sorted_by,
		const Range &range) const noexcept;

	void validate_order(const std::vector<node_index_t> &order);

	void validate_leaf();

	void validate_node(int node_index);

	void validate_node_index();

	void validate_packed_node();

	void validate_parent();

	void collect_range_parallel(int node_index, std::vector<std::pair<int, int>> &result);
	void collect_range_single(int node_index, std::vector<std::pair<int, int>> &result);

	// ノード
	std::vector<Node> m_node_list;
	std::vector<Nodex8> m_nodex8_list;
	std::vector<LeafNode> m_leafnode_list;
	MeshTriangleList m_mesh_list;
	std::vector<PackedTrianglex8> m_packed_triangle_list;

	node_index_t root_node_index;
};

// =================================
// implementation

SimpleBVH::SimpleBVH() {}

void SimpleBVH::collect_node_to_pack(node_index_t old_node_index, std::vector<node_index_t> &children) {
	children.clear();
	std::queue<std::size_t> que;
	que.push(old_node_index);
	while (!que.empty() && que.size() + children.size() < 8) {
		auto node = que.front();
		que.pop();
		if (m_node_list[node].is_leaf()) {
			children.push_back(node);
		}
		else {
			que.push(m_node_list[node].left);
			que.push(m_node_list[node].right);
		}
	}

	while (!que.empty()) {
		auto node = que.front();
		que.pop();
		children.push_back(node);
	}
}


void SimpleBVH::setup_packed_node() noexcept {
	std::queue<std::size_t> que;
	que.push(root_node_index);

	// 旧 node id -> 新 node id 
	std::vector<std::size_t> packed_parent_node(m_node_list.size());

	// new_node は、parent の何番目の children として登録されているか
	std::vector<std::size_t> children_id(m_node_list.size());

	std::size_t packed_node_index = 0;
	while (!que.empty()) {
		const auto old_node_index = que.front();
		que.pop();

		// 深さ 3 の old node を集める
		std::vector<node_index_t> children;
		collect_node_to_pack(old_node_index, children);

		// node_index の新規ノードを作成
		std::vector<Node *> node_list;
		for (auto child : children) {
			node_list.push_back(&m_node_list[child]);
		}
		m_nodex8_list.emplace_back(node_list);
		m_nodex8_list.back().self = packed_node_index;

		// children id の登録
		for (std::size_t i = 0; i < children.size(); i++) {
			children_id[m_node_list[children[i]].self] = i;
		}

		// root node 以外なら、parent node に今作ったノードを登録
		if (old_node_index > 0) {
			const auto packed_parent = packed_parent_node[old_node_index];
			auto &parent = m_nodex8_list[packed_parent];
#ifdef DEBUG
			assert(children_id[old_node_index] < 8);
#endif
			parent.children[children_id[old_node_index]] = packed_node_index;
		}

		// leaf node でないものは、その子供も pack する必要がある
		for (std::size_t i = 0; i < children.size(); i++) {
			if (node_list[i]->is_leaf()) {
				// leaf 判定を引き継ぐために、負の値にする必要がある
				m_nodex8_list.back().children[i] = -node_list[i]->leaf_index();
			}
			else {
				que.push(node_list[i]->self);
			}
			// single old child node -> parent new packed node
			packed_parent_node[node_list[i]->self] = packed_node_index;
		}
		packed_node_index++;
	}
}

void SimpleBVH::setup_packed_triangle() noexcept {
	std::queue<std::size_t> que;
	que.push(root_node_index);

	std::size_t packed_leaf_index = 0;

	while (!que.empty()) {
		const auto node_index = que.front();
		que.pop();

		auto &node = m_node_list[node_index];

		if (!node.is_leaf()) {
			que.push(node.left);
			que.push(node.right);
		}
		else {
			LeafNode &leaf_node = m_leafnode_list[node.leaf_index()];

			const std::size_t mesh_index_from = packed_leaf_index;
			leaf_node.start_mesh_index = leaf_node.leaf_meshid_from;

			for (meshid_index_t mesh_index = leaf_node.leaf_meshid_from;
				mesh_index < leaf_node.leaf_meshid_to; mesh_index += 8) {
				const auto to_index =
					std::min<meshid_index_t>(mesh_index + 8, leaf_node.leaf_meshid_to);
				m_packed_triangle_list.emplace_back(m_mesh_list.begin() + mesh_index,
					m_mesh_list.begin() + to_index);
				packed_leaf_index++;
			}
			leaf_node.leaf_meshid_from = mesh_index_from;
			leaf_node.leaf_meshid_to = packed_leaf_index;
		}
	}
}

bool SimpleBVH::construct(MeshTriangleList &&mesh_list) {
	m_mesh_list = std::forward<MeshTriangleList>(mesh_list);

	root_node_index = 0;

	m_node_list.emplace_back(root_node_index);
	m_node_list.resize(m_mesh_list.size() * 2);
	m_node_list[root_node_index].parent = root_node_index;
	m_leafnode_list.resize(m_mesh_list.size() * 2);

	const std::size_t num_mesh = m_mesh_list.size();

	std::vector<meshid_index_t> mesh_index_list(num_mesh);

	std::vector<Vec3> center_position(num_mesh);
	for (std::size_t index = 0; index < num_mesh; index++) {
		m_mesh_list[index].center(&center_position[index]);
	}

	std::array<std::vector<meshid_index_t>, 3> mesh_id_sorted_by;

	for (auto axis : { AxisX, AxisY, AxisZ }) {
		auto &index_list = mesh_id_sorted_by[axis];
		index_list.reserve(num_mesh);
		for (std::int32_t mesh_id = 0; mesh_id < num_mesh; mesh_id++) {
			index_list.push_back(mesh_id);
		}
		std::sort(index_list.begin(), index_list.end(),
			[&](const std::size_t index1, const std::size_t index2) {
			return center_position[index1][axis] <
				center_position[index2][axis];
		});
	}

	// because std::vector<bool> is packed & not atomic
	std::vector<std::uint8_t> is_left_flag(num_mesh);

	std::atomic_int32_t node_index = 0;

	std::atomic_int32_t leaf_index = 1;

	// AABB of range [0, index]
	std::vector<AABB> left_accumulated_AABB(num_mesh);

	// AABB of range [index, num_mesh - 1]
	std::vector<AABB> right_accumulated_AABB(num_mesh);

	TaskQueue<Range> que(m_mesh_list.size());
	que.push(Range(root_node_index, num_mesh, node_index));

	constexpr std::size_t num_threads = 16;
	std::vector<std::thread> threads(num_threads);

	auto setup_leaf = [&](Node &current_node, const Range &range) {
		const auto local_leaf_index = leaf_index++;
		current_node.set_leaf_index(local_leaf_index);
		auto &leaf_node = m_leafnode_list[local_leaf_index];
		leaf_node.leaf_meshid_from = range.from;
		leaf_node.leaf_meshid_to = range.to;

		// node is already cleared when node is created
		for (meshid_index_t mesh_index = range.from; mesh_index < range.to;
			mesh_index++) {
			mesh_index_list[mesh_index] =
				mesh_id_sorted_by[AxisX][mesh_index];
			current_node.aabb.enlarge(
				m_mesh_list[mesh_index_list[mesh_index]]);
		}
		que.update(range.to - range.from);
	};

	for (auto i = 0u; i < num_threads; ++i) {
		threads[i] = std::thread([&]() {
			while (!que.terminate()) {
				auto may_range = que.front_pop();
				if (!may_range.has_value()) {
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
				else {
					auto range = may_range.value();

					auto &current_node = m_node_list[range.node_index];

					if (range.size() <= leaf_size_threashold) {
						setup_leaf(current_node, range);
					}
					else {
						std::int32_t best_axis, best_position;
						select_best_split(range, mesh_id_sorted_by, left_accumulated_AABB,
							right_accumulated_AABB, &best_axis,
							&best_position);

						if (best_axis == Invalid) {
							setup_leaf(current_node, range);
						}
						else {
							// node is already cleared when node is created
							current_node.aabb.enlarge(left_accumulated_AABB[range.to - 1]);

							adjust_index(mesh_id_sorted_by, is_left_flag, range, best_axis,
								best_position);

							current_node.left = ++node_index;
							m_node_list[current_node.left].self = current_node.left;
							m_node_list[current_node.left].parent = current_node.self;
							current_node.right = ++node_index;
							m_node_list[current_node.right].self = current_node.right;
							m_node_list[current_node.right].parent = current_node.self;

							que.push(Range(range.from, best_position, current_node.left));
							que.push(Range(best_position, range.to, current_node.right));
						}
					}
				}
			}
		});
	}

	for (auto i = 0u; i < num_threads; ++i) {
		threads[i].join();
	}

#ifdef DEBUG
	{
		const Range range(0, m_mesh_list.size(), root_node_index);
		validate_range(mesh_id_sorted_by, range);
	}
#endif

	m_node_list.resize(++node_index);
	m_node_list.shrink_to_fit();

	m_leafnode_list.resize(leaf_index);
	m_leafnode_list.shrink_to_fit();

#ifdef DEBUG
	validate_leaf();
	validate_node_index();
	validate_parent();
#endif

	optimize_through_insertion();

#ifdef DEBUG
	validate_leaf();
	validate_node_index();
	validate_parent();
#endif

	reorder_node(5);
	reorder_mesh(mesh_index_list);

#ifdef DEBUG
	validate_node(root_node_index);
#endif

	setup_packed_triangle();

	setup_packed_node();

#ifdef DEBUG
	validate_packed_node();
#endif

	reorder_leafnode();

	return true;
}

void SimpleBVH::optimize_through_insertion() noexcept
{
	auto calculate_cost = [&](node_index_t index) {
		const auto &cur = m_node_list[index];
		const auto &left = m_node_list[cur.left];
		const auto &right = m_node_list[cur.right];

		const double cost_area = cur.aabb.area();
		const double cost_min = cost_area / std::min(left.aabb.area(), right.aabb.area());
		const double cost_sum = cost_area / (left.aabb.area() + right.aabb.area());
		return cost_area * cost_min * cost_sum;
	};

	// inner node を集める
	std::vector<std::pair<double, node_index_t>> inner_node;
	// depth 2 以下のノードを delete すると、root node の位置を変える必要があって、面倒なのでやらない
	for (node_index_t i = 1; i < m_node_list.size(); i++)
	{
		if (!m_node_list[i].is_leaf())
		{
			inner_node.emplace_back(-calculate_cost(i), i);
		}
	}
	std::sort(inner_node.begin(), inner_node.end());

	// 1% の node に対して、以下を実行
	const std::size_t optimize_node_threashold = std::min<std::size_t>(inner_node.size(), std::max<std::size_t>(15, inner_node.size() / 10));

	std::size_t optimize_count = 0;

	std::vector<bool> removed(m_node_list.size(), false);

	for (auto[_, node_id] : inner_node)
	{
		if (node_id != root_node_index && !removed[node_id])
		{
			// node を 1つ引っこ抜く
			const auto unnecessary_node = remove_node(node_id);

			// どこに挿入するのが最適かを、branch and bound で決定
			const auto insert_point = search_insert_point(node_id);
			insert_node(node_id, insert_point, unnecessary_node);
			optimize_count++;

			removed[unnecessary_node] = true;
			removed[node_id] = true;

			if (optimize_count == optimize_node_threashold)
			{
				break;
			}
		}
	}
}

struct InsertionOptimizationCost
{
	using node_index_t = SimpleBVH::node_index_t;

	double DirectCost;
	double InducedCost;
	node_index_t node;

	InsertionOptimizationCost()
		: DirectCost(0), InducedCost(0), node(0) {}

	InsertionOptimizationCost(double DirectCost)
		: DirectCost(DirectCost), InducedCost(0), node(0) {}

	InsertionOptimizationCost(double DirectCost, double InducedCost, node_index_t node)
		: DirectCost(DirectCost), InducedCost(InducedCost), node(node) {}

	double cost() const {
		return DirectCost + InducedCost;
	}

	double lowerbound() const noexcept {
		return InducedCost;
	}
};

bool operator>(const InsertionOptimizationCost &src1, const InsertionOptimizationCost &src2) {
	return src1.cost() > src2.cost();
}

SimpleBVH::node_index_t SimpleBVH::search_insert_point(node_index_t node_id) const noexcept
{
	std::priority_queue<InsertionOptimizationCost, std::vector<InsertionOptimizationCost>, std::greater<InsertionOptimizationCost>> que;
	que.emplace(m_node_list[root_node_index].aabb.area(), 0, root_node_index);

	node_index_t best_insert_point = Invalid;
	float best_cost = std::numeric_limits<float>::max();

	auto union_area = [&](const AABB &bb1, const AABB &bb2) {
		AABB ret;
		ret.enlarge(bb1);
		ret.enlarge(bb2);
		return ret.area();
	};

	while (!que.empty())
	{
		const auto cost = que.top();
		que.pop();
		if (cost.lowerbound() <= best_cost)
		{
			if (best_cost > cost.cost()) {
				best_cost = cost.cost();
				best_insert_point = cost.node;
			}
			const auto &node = m_node_list[cost.node];
			const auto &insert_node = m_node_list[node_id];
			const double induced_diff = union_area(node.aabb, insert_node.aabb) - node.aabb.area();

			if (!node.is_leaf()) {
				que.emplace(cost.InducedCost + induced_diff, union_area(m_node_list[node.left].aabb, insert_node.aabb), node.left);
				que.emplace(cost.InducedCost + induced_diff, union_area(m_node_list[node.right].aabb, insert_node.aabb), node.right);
			}
		}
	}
	return best_insert_point;
}

void SimpleBVH::insert_node(node_index_t new_node_id, node_index_t insert_point_id, node_index_t empty_node_id) noexcept
{
	auto &new_node = m_node_list[new_node_id];
	auto &insert_point = m_node_list[insert_point_id];
	auto &empty_node = m_node_list[empty_node_id];
	auto &parent = m_node_list[insert_point.parent];

	if (insert_point.self == root_node_index)
	{
		root_node_index = empty_node.self;
	}
	else {
		// parent <-> empty_node
		(parent.left == insert_point.self ? parent.left : parent.right) = empty_node.self;
		empty_node.parent = parent.self;
	}

	// empty_node <-> insert_point
	empty_node.left = insert_point.self;
	insert_point.parent = empty_node.self;

	// empty_node <-> new_node
	empty_node.right = new_node.self;
	new_node.parent = empty_node.self;

	refit_for_insertion(empty_node_id);
}

SimpleBVH::node_index_t SimpleBVH::remove_node(node_index_t node_id) noexcept
{
	const auto get_sibling = [&](const node_index_t node_id) {
		const auto &parent = m_node_list[m_node_list[node_id].parent];
		return parent.right == node_id ? parent.left : parent.right;
	};

	auto &cur = m_node_list[node_id];
	auto &parent = m_node_list[cur.parent];
	auto &sibling = m_node_list[get_sibling(node_id)];
	auto &parent_parent = m_node_list[parent.parent];

	const auto unnecessary_node = parent.self;

	// replace parent -> sibling
	(parent_parent.left == parent.self ? parent_parent.left : parent_parent.right) = sibling.self;
	sibling.parent = parent_parent.self;

	// clear parent
	parent.left = parent.right = Invalid;
	parent.parent = parent.self;

	// set current as root
	cur.parent = node_id;

	if (unnecessary_node == root_node_index)
	{
		root_node_index = sibling.self;
	}
	refit_for_insertion(sibling.self);

	return unnecessary_node;
}


void SimpleBVH::validate_parent()
{
	for (node_index_t i = 0; i < m_node_list.size(); i++)
	{
		const auto &cur = m_node_list[i];
		if (!cur.is_leaf()) {
			assert(m_node_list[cur.left].parent == cur.self);
			assert(m_node_list[cur.right].parent == cur.self);
		}
	}
}


void SimpleBVH::validate_range(
	const std::array<std::vector<meshid_index_t>, 3> &mesh_id_sorted_by,
	const Range &range) const noexcept {
	std::array<std::set<meshid_index_t>, 3> mesh_id_set;
	for (auto axis : { AxisX, AxisY, AxisZ }) {
		for (meshid_index_t index = range.from; index < range.to; index++) {
			mesh_id_set[axis].insert(mesh_id_sorted_by[axis][index]);
		}
	}
	assert(mesh_id_set[0] == mesh_id_set[1]);
	assert(mesh_id_set[0] == mesh_id_set[2]);
}

void SimpleBVH::validate_node_index() {
	std::set<node_index_t> node_id_set;
	node_id_set.insert(root_node_index);
	for (auto &node : m_node_list) {
		if (!node.is_leaf()) {
			node_id_set.insert(node.left);
			node_id_set.insert(node.right);
			assert(0 <= node.left && node.left < m_node_list.size());
			assert(0 <= node.right && node.right < m_node_list.size());
		}
	}
	assert(m_node_list.size() == node_id_set.size());
}

void SimpleBVH::validate_node(int node_index) {
	auto &node = m_node_list[node_index];
	if (node.is_leaf()) {
		const LeafNode &leaf_node = m_leafnode_list[node.leaf_index()];
		for (auto mesh_index = leaf_node.leaf_meshid_from;
			mesh_index < leaf_node.leaf_meshid_to; mesh_index++) {
			assert(node.aabb.contain(m_mesh_list[mesh_index]));
		}
	}
	else {
		assert(node.aabb.contain(m_node_list[node.left].aabb));
		assert(node.aabb.contain(m_node_list[node.right].aabb));
		validate_node(node.left);
		validate_node(node.right);
	}
}

void SimpleBVH::validate_leaf() {
	std::set<meshid_index_t> mesh_ids;
	for (auto node : m_node_list) {
		if (node.is_leaf()) {
			const auto &leaf_node = m_leafnode_list[node.leaf_index()];
			for (meshid_index_t id = leaf_node.leaf_meshid_from; id < leaf_node.leaf_meshid_to; id++) {
				assert(0 <= id && id < m_mesh_list.size());
				mesh_ids.insert(id);
			}
		}
	}
	assert(mesh_ids.size() == m_mesh_list.size());
}

void SimpleBVH::validate_order(const std::vector<node_index_t> &order) {
	std::set<meshid_index_t> mesh_ids;
	for (auto new_id : order) {
		assert(0 <= new_id && new_id < order.size());
		mesh_ids.insert(new_id);
	}
	assert(mesh_ids.size() == order.size());
}

void SimpleBVH::collect_range_parallel(int node_index, std::vector<std::pair<int, int>> &result) {
	const auto &node = m_nodex8_list[node_index];
	for (std::size_t i = 0; i < node.node_size_in_children; i++) {
		if (node.is_leaf(i)) {
			const auto &leaf_node = m_leafnode_list[node.leaf_index(i)];
			result.emplace_back(leaf_node.leaf_meshid_from, leaf_node.leaf_meshid_to);
		}
		else {
			collect_range_parallel(node.children[i], result);
		}
	}
}

void SimpleBVH::collect_range_single(int node_index, std::vector<std::pair<int, int>> &result) {
	const auto &node = m_node_list[node_index];
	if (node.is_leaf()) {
		const auto &leaf_node = m_leafnode_list[node.leaf_index()];
		result.emplace_back(leaf_node.leaf_meshid_from, leaf_node.leaf_meshid_to);
	}
	else {
		collect_range_single(node.left, result);
		collect_range_single(node.right, result);
	}
}

void SimpleBVH::validate_packed_node() {
	// 子ノードを回収したとき、leaf_index_node がすべて同じか？
	std::vector<std::pair<int, int>> leaf_range_single;
	collect_range_single(root_node_index, leaf_range_single);
	std::sort(leaf_range_single.begin(), leaf_range_single.end());

	std::vector<std::pair<int, int>> leaf_range_parallel;
	collect_range_parallel(0, leaf_range_parallel);
	std::sort(leaf_range_parallel.begin(), leaf_range_parallel.end());
	assert(leaf_range_single.size() == leaf_range_parallel.size());

	for (std::size_t i = 0; i < leaf_range_single.size(); i++) {
		assert(leaf_range_single[i] == leaf_range_parallel[i]);
	}
}

bool SimpleBVH::intersect(RayExt &ray) const {
	meshid_index_t hitMeshIdx = 0;
	const bool isHit = intersectSub(0, ray, &hitMeshIdx);
	if (isHit) {
		const auto &mesh = m_mesh_list[hitMeshIdx];
		const float u = ray.u;
		const float v = ray.v;
		ray.isect = mesh.vertex(0) * (1.0f - u - v) + mesh.vertex(1) * u +
			mesh.vertex(2) * v;
		ray.ns = mesh.normal(0) * (1.0f - u - v) + mesh.normal(1) * u +
			mesh.normal(2) * v;
		ray.faceid = mesh.faceid();
	}
	return isHit;
}

bool SimpleBVH::intersectAnySub(node_index_t nodeIndex, RayExt &ray,
	meshid_index_t *hitMeshIndex) const {

	const auto &node = m_nodex8_list[nodeIndex];

	constexpr int NOT_EQUAL = 4;

	__m256 distance = node.aabb.intersect_distance(ray, ray.tfar);
	__m256 valid_mask = _mm256_cmp_ps(distance, _mm256_set1_ps(PackedAABBx8::InvalidDistance), NOT_EQUAL);
	const auto bitmask = _mm256_movemask_ps(valid_mask);
	std::uint64_t packed_mask = _pdep_u64(bitmask & ((1 << node.node_size_in_children) - 1), 0x0101010101010101) * 0xFF;

	std::uint64_t extracted_index = _pext_u64(0x0706050403020100, packed_mask);

	const int count = __popcnt64(bitmask);

	FixedVector<std::size_t, 8> valid_index;
	for (std::size_t i = 0; i < count; i++) {
		valid_index.push_back(extracted_index & 0xff);
		extracted_index >>= 8;
	}

	if (!valid_index.empty()) {

		std::array<float, 8> distance_array;
		_mm256_storeu_ps(distance_array.data(), distance);

		if (valid_index.size() > 1) {
			std::sort(valid_index.begin(), valid_index.end(), [&](std::size_t i1, std::size_t i2) {
				if (node.is_leaf(i1) != node.is_leaf(i2)) {
					return node.is_leaf(i1) > node.is_leaf(i2);
				}
				else {
					return distance_array[i1] < distance_array[i2];
				}
			});
		}

		for (auto idx : valid_index) {
			if (node.is_leaf(idx)) {
				const auto &leaf_node = m_leafnode_list[node.leaf_index(idx)];
				for (meshid_index_t mesh_index = leaf_node.leaf_meshid_from;
					mesh_index < leaf_node.leaf_meshid_to; mesh_index++) {
					if (m_packed_triangle_list[mesh_index].intersect_distance(ray) !=
						PackedTrianglex8::InvalidIndex) {
						return true;
					}
				}
			}
			else if (intersectAnySub(node.children[idx], ray, hitMeshIndex)) {
				return true;
			}
		}
	}
	return false;
}

bool SimpleBVH::intersectAny(RayExt &ray) const {
	meshid_index_t hitMeshIndex;
	return intersectAnySub(0, ray, &hitMeshIndex);
}

void SimpleBVH::select_best_split(
	const Range &range,
	const std::array<std::vector<meshid_index_t>, 3> &index_sorted_by,
	std::vector<AABB> &left_accumulated_AABB,
	std::vector<AABB> &right_accumulated_AABB, std::int32_t *best_axis,
	std::int32_t *best_split_position) {
	*best_axis = Invalid;
	*best_split_position = Invalid;
	float best_sah_cost = std::numeric_limits<float>::max();

	for (const auto axis : { AxisX, AxisY, AxisZ }) {
		// register left accumulated AABB
		for (meshid_index_t index = range.from; index < range.to; index++) {
			left_accumulated_AABB[index].clear();
			const auto mesh_id = index_sorted_by[axis][index];
			left_accumulated_AABB[index].enlarge(m_mesh_list[mesh_id]);
			if (range.from < index) {
				left_accumulated_AABB[index].enlarge(left_accumulated_AABB[index - 1]);
			}
		}

		// register right accumulated AABB
		for (auto index = range.to - 1; range.from <= index; index--) {
			right_accumulated_AABB[index].clear();
			const auto mesh_id = index_sorted_by[axis][index];
			right_accumulated_AABB[index].enlarge(m_mesh_list[mesh_id]);
			if (index < range.to - 1) {
				right_accumulated_AABB[index].enlarge(
					right_accumulated_AABB[index + 1]);
			}
		}

		const float root_area = left_accumulated_AABB[range.to - 1].area();
		// cost of un-splited cost
		const float unsplited_cost = CostPolygonIntersect * range.size();
		if (best_sah_cost > unsplited_cost) {
			best_sah_cost = unsplited_cost;
		}

		// decide point such that [from, split_position), [split_position, to)
		for (meshid_index_t split_position = range.from + 1;
			split_position <= range.to - 1; split_position++) {
			const float sah_cost =
				2.0f * CostBoundingBoxIntersect +
				CostPolygonIntersect / root_area *
				(left_accumulated_AABB[split_position - 1].area() *
				(split_position - range.from) +
					right_accumulated_AABB[split_position].area() *
					(range.to - split_position));

			if (sah_cost < best_sah_cost) {
				best_sah_cost = sah_cost;
				*best_axis = axis;
				*best_split_position = split_position;
			}
		}
	}
}

void SimpleBVH::adjust_index(
	std::array<std::vector<meshid_index_t>, 3> &index_sorted_by,
	std::vector<std::uint8_t> &is_left_flag, const Range &range,
	const std::int32_t selected_axis, const std::int32_t split_position) {
	// keep correctness of sort for all axis.

	for (std::size_t index = range.from; index < split_position; index++) {
		const auto mesh_id = index_sorted_by[selected_axis][index];
		is_left_flag[mesh_id] = Left;
	}
	for (std::size_t index = split_position; index < range.to; index++) {
		const auto mesh_id = index_sorted_by[selected_axis][index];
		is_left_flag[mesh_id] = Right;
	}

	const std::size_t left_size = split_position - range.from;

	for (auto axis : { AxisX, AxisY, AxisZ }) {
		if (selected_axis != axis) {
			std::stable_partition(index_sorted_by[axis].begin() + range.from,
				index_sorted_by[axis].begin() + range.to,
				[&](const std::size_t mesh_id) {
				return is_left_flag[mesh_id] == Left;
			});
		}
	}
}

void SimpleBVH::visit_for_reorder(node_index_t old_node_index,
	std::int32_t depth, std::int32_t max_depth,
	node_index_t &new_node_index,
	std::queue<node_index_t> &que,
	std::vector<node_index_t> &order) {
	auto &current_node = m_node_list[old_node_index];
	order[old_node_index] = new_node_index;
	new_node_index++;

	if (!current_node.is_leaf()) {
		if (depth < max_depth) {
			visit_for_reorder(current_node.left, depth + 1, max_depth, new_node_index,
				que, order);
			visit_for_reorder(current_node.right, depth + 1, max_depth,
				new_node_index, que, order);
		}
		else {
			que.push(current_node.left);
			que.push(current_node.right);
		}
	}
}

void SimpleBVH::reorder_leafnode() noexcept
{
	// m_nodex8_list のアクセス順に leaf node の order を取得
	std::vector<node_index_t> order;
	std::queue<node_index_t> que;
	que.push(0);
	order.push_back(0);

	node_index_t new_leaf_index = 1;

	while (!que.empty())
	{
		auto node_index = que.front();
		que.pop();
		auto &node = m_nodex8_list[node_index];

		for (auto idx = 0; idx < node.node_size_in_children; idx++)
		{
			if (!node.is_leaf(idx))
			{
				que.push(node.children[idx]);
			}
			else {
				order.push_back(node.leaf_index(idx));
				node.children[idx] = -new_leaf_index++;
			}
		}
	}
#ifdef DEBUG
	validate_order(order);
	assert(order.size() == m_leafnode_list.size());
#endif

	// 同じサイズの buffer を用意して, copy_from
	std::vector<LeafNode> after_node_list(m_leafnode_list.size());
	for (node_index_t i = 0; i < m_leafnode_list.size(); i++)
	{
		after_node_list[i] = m_leafnode_list[order[i]];
	}

	// 書き戻す
	std::copy(after_node_list.begin(), after_node_list.end(), m_leafnode_list.begin());
}

void SimpleBVH::refit_for_insertion(node_index_t node_id) noexcept
{
	auto &node = m_node_list[node_id];
	if (!node.is_leaf())
	{
		auto &left = m_node_list[node.left];
		auto &right = m_node_list[node.right];

		node.aabb.clear();
		node.aabb.enlarge(left.aabb);
		node.aabb.enlarge(right.aabb);
	}
	if (root_node_index != node.self)
	{
		refit_for_insertion(node.parent);
	}
}

void SimpleBVH::reorder_mesh(
	const std::vector<meshid_index_t> &mesh_index_list) noexcept {
	const std::size_t mesh_size = m_mesh_list.size();
	std::vector<std::int32_t> order(mesh_size);

	std::int32_t new_mesh_index = 0;
	for (auto &node : m_node_list) {
		if (node.is_leaf()) {

			auto &leaf_node = m_leafnode_list[node.leaf_index()];
			const meshid_index_t new_meshid_from = new_mesh_index;
			for (auto old_mesh_index = leaf_node.leaf_meshid_from;
				old_mesh_index < leaf_node.leaf_meshid_to; old_mesh_index++) {
				order[mesh_index_list[old_mesh_index]] = new_mesh_index;
				new_mesh_index++;
			}
			leaf_node.leaf_meshid_from = new_meshid_from;
			leaf_node.leaf_meshid_to = new_mesh_index;
		}
	}

#ifdef DEBUG
	assert(new_mesh_index == m_mesh_list.size());
	validate_order(order);
#endif

	std::vector<MeshTriangle> new_mesh_list(mesh_size);
	for (std::int32_t old_mesh_id = 0; old_mesh_id < mesh_size; old_mesh_id++) {
		const std::int32_t new_mesh_id = order[old_mesh_id];
		new_mesh_list[new_mesh_id].copy_from(m_mesh_list[old_mesh_id]);
		new_mesh_list[new_mesh_id].faceid() = new_mesh_id;
	}
	for (std::int32_t new_mesh_id = 0; new_mesh_id < mesh_size; new_mesh_id++) {
		m_mesh_list[new_mesh_id].copy_from(new_mesh_list[new_mesh_id]);
	}
}

void SimpleBVH::reorder_node(const std::size_t max_depth) noexcept {
	std::vector<node_index_t> order(m_node_list.size());

	std::queue<node_index_t> que;
	que.push(root_node_index);

	node_index_t new_index = 0;

	while (!que.empty()) {
		int32_t old_node_index = que.front();
		que.pop();
		visit_for_reorder(old_node_index, 0, max_depth, new_index, que, order);
	}

#ifdef DEBUG
	validate_order(order);
#endif

	std::vector<Node> next_node_list(m_node_list.size(), 0);
	for (std::int32_t old_node_id = 0; old_node_id < new_index; old_node_id++) {
		const auto new_node_id = order[old_node_id];
		auto &new_node = next_node_list[new_node_id];
		new_node.copy_from(m_node_list[old_node_id]);
		new_node.self = order[m_node_list[old_node_id].self];
		if (!new_node.is_leaf()) {
			new_node.left = order[m_node_list[old_node_id].left];
			new_node.right = order[m_node_list[old_node_id].right];
		}
	}

	// TODO: setup parent appropreately

	std::swap(next_node_list, m_node_list);
	root_node_index = 0;
}

bool SimpleBVH::intersectSub(std::int32_t nodeIndex, RayExt &ray,
	meshid_index_t *hitMeshIndex) const {

	const auto &node = m_nodex8_list[nodeIndex];

	constexpr int NOT_EQUAL = 4;

	__m256 distance = node.aabb.intersect_distance(ray, ray.tfar);
	__m256 valid_mask = _mm256_cmp_ps(distance, _mm256_set1_ps(PackedAABBx8::InvalidDistance), NOT_EQUAL);
	const auto bitmask = _mm256_movemask_ps(valid_mask);
	std::uint64_t packed_mask = _pdep_u64(bitmask & ((1 << node.node_size_in_children) - 1), 0x0101010101010101) * 0xFF;

	std::uint64_t extracted_index = _pext_u64(0x0706050403020100, packed_mask);
	const int count = __popcnt64(bitmask);

	FixedVector<std::size_t, 8> valid_index;
	for (std::size_t i = 0; i < count; i++) {
		valid_index.push_back(extracted_index & 0xff);
		extracted_index >>= 8;
	}

	if (valid_index.empty()) {
		return false;
	}
	else {
		std::array<float, 8> distance_array;
		_mm256_storeu_ps(distance_array.data(), distance);

		if (valid_index.size() > 1) {
			std::sort(valid_index.begin(), valid_index.end(), [&](std::size_t i1, std::size_t i2) {
				return distance_array[i1] < distance_array[i2];
			});
		}

		bool success = false;
		for (auto idx : valid_index) {
			if (distance_array[idx] < ray.tfar) {
				if (node.is_leaf(idx)) {
					const auto &leaf_node = m_leafnode_list[node.leaf_index(idx)];
					for (std::int32_t mesh_index = leaf_node.leaf_meshid_from;
						mesh_index < leaf_node.leaf_meshid_to; mesh_index++) {
						const std::size_t subindex =
							m_packed_triangle_list[mesh_index].intersect_distance(ray);
						if (subindex != PackedTrianglex8::InvalidIndex) {
							*hitMeshIndex = leaf_node.start_mesh_index +
								8 * (mesh_index - leaf_node.leaf_meshid_from) + subindex;
							success = true;
						}
					}
				}
				else {
					success |= intersectSub(node.children[idx], ray, hitMeshIndex);
				}
			}
		}
		return success;
	}
}
