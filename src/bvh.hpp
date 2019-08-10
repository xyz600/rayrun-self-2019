﻿#pragma once

#include "bounding_box.hpp"
#include "ray.hpp"
#include "task_queue.hpp"
#include "triangle.hpp"
#include "vec.hpp"

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

private:
	struct Node {
	public:
		std::int32_t self;
		std::int32_t left;
		std::int32_t right;

		// AABB
		AABB aabb;

		std::size_t leaf_meshid_from;
		std::size_t leaf_meshid_to;
		std::size_t start_mesh_index;

		Node() noexcept { clear(); }

		Node(std::int32_t self) noexcept : self(self) { clear(); }

		void clear() noexcept {
			left = right = SimpleBVH::Invalid;
			leaf_meshid_from = SimpleBVH::Invalid;
			leaf_meshid_to = SimpleBVH::Invalid;
			aabb.clear();
		}

		void set_leaf(const std::size_t leaf_from,
			const std::size_t leaf_to) noexcept {
			this->leaf_meshid_from = leaf_from;
			this->leaf_meshid_to = leaf_to;
			left = right = SimpleBVH::Invalid;
		}

		bool is_leaf() const noexcept {
			return left == SimpleBVH::Invalid && right == SimpleBVH::Invalid;
		}

		void copy_from(const Node &src) noexcept {
			self = src.self;
			left = src.left;
			right = src.right;
			aabb = src.aabb;
			leaf_meshid_from = src.leaf_meshid_from;
			leaf_meshid_to = src.leaf_meshid_to;
			start_mesh_index = src.start_mesh_index;
		}
	};

	struct Nodex8 {

		static constexpr std::size_t LeafID = std::numeric_limits<std::size_t>::max();

		Nodex8(const std::vector<Node *> &node_list) {
			std::vector<AABB *> aabb_list;
			for (std::size_t i = 0; i < 8; i++) {
				const std::size_t index = std::min(i, node_list.size() - 1);
				aabb_list.push_back(&node_list[index]->aabb);
				leaf_meshid_from[i] = node_list[index]->leaf_meshid_from;
				leaf_meshid_to[i] = node_list[index]->leaf_meshid_to;
			}
			aabb.construct(aabb_list);
			node_size_in_children = 0;
		}

		bool is_leaf(const std::size_t index) {
			return children[index] == LeafID;
		}

		std::size_t self;
		std::array<std::size_t, 8> children;
		PackedAABBx8 aabb;
		std::array <std::size_t, 8> leaf_meshid_from;
		std::array<std::size_t, 8> leaf_meshid_to;
		std::size_t node_size_in_children;
	};

	struct Range {
		std::int32_t from;
		std::int32_t to;
		std::int32_t node_index;

		Range(std::int32_t from, std::int32_t to, std::int32_t node_index)
			: from(from), to(to), node_index(node_index) {}
		std::size_t size() const noexcept { return to - from; }
	};

	bool intersectSub(std::int32_t nodeIndex, RayExt &ray,
		std::int32_t *hitMeshIndex) const;
	bool intersectAnySub(std::int32_t nodeIndex, RayExt &ray,
		std::int32_t *hitMeshIndex) const;

	void adjust_index(std::array<std::vector<std::int32_t>, 3> &index_sorted_by,
		std::vector<std::uint8_t> &is_left_flag, const Range &range,
		const std::int32_t axis, const std::int32_t position);

	void select_best_split(
		const Range &range,
		const std::array<std::vector<std::int32_t>, 3> &index_sorted_by,
		std::vector<AABB> &left_accumulated_AABB,
		std::vector<AABB> &right_accumulated_AABB, std::int32_t *best_axis,
		std::int32_t *best_position);

	void reorder_mesh(const std::vector<std::int32_t> &mesh_index_list) noexcept;

	void reorder_node(const std::size_t max_depth) noexcept;

	void visit_for_reorder(std::int32_t old_node_index, std::int32_t depth,
		std::int32_t max_depth, std::int32_t &new_node_index,
		std::queue<std::int32_t> &que,
		std::vector<std::int32_t> &order);

	void collect_node_to_pack(std::int32_t old_node_index, std::vector<std::size_t> &children);

	void setup_packed_node() noexcept;

	void setup_packed_triangle() noexcept;

	static constexpr std::int32_t AxisX = 0;
	static constexpr std::int32_t AxisY = 1;
	static constexpr std::int32_t AxisZ = 2;

	static constexpr std::uint8_t Left = 0;
	static constexpr std::uint8_t Right = 1;

	static constexpr std::int32_t Invalid =
		std::numeric_limits<std::int32_t>::max();

	static constexpr float CostBoundingBoxIntersect = 1.0;
	static constexpr float CostPolygonIntersect = 1.0;

	static constexpr std::int32_t leaf_size_threashold = 16;

	void
		validate_range(const std::array<std::vector<int32_t>, 3> &mesh_id_sorted_by,
			const Range &range) const noexcept;

	void validate_order(const std::vector<std::int32_t> &order);

	void validate_leaf();

	void validate_node(int node_index);

	void validate_node_index();

	// ノード
	std::vector<Node> m_node_list;
	std::vector<Nodex8> m_nodex8_list;
	MeshTriangleList m_mesh_list;
	std::vector<PackedTrianglex8> m_packed_triangle_list;
};

// =================================
// implementation

SimpleBVH::SimpleBVH() {}

void SimpleBVH::collect_node_to_pack(std::int32_t old_node_index, std::vector<std::size_t> &children) {
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
	que.push(0);

	// 旧 node id -> 新 node id 
	std::vector<std::size_t> packed_parent_node(m_node_list.size());

	// new_node は、parent の何番目の children として登録されているか
	std::vector<std::size_t> children_id(m_node_list.size());

	std::size_t packed_node_index = 0;
	while (!que.empty()) {
		const auto old_node_index = que.front();
		que.pop();

		// 深さ 3 の old node を集める
		std::vector<std::size_t> children;
		collect_node_to_pack(old_node_index, children);

		// node_index の新規ノードを作成
		std::vector<Node *> node_list;
		for (auto child : children) {
			node_list.push_back(&m_node_list[child]);
		}
		while (node_list.size() < 8) {
			node_list.push_back(&m_node_list[children.back()]);
		}
		m_nodex8_list.emplace_back(node_list);
		m_nodex8_list.back().self = packed_node_index;
		m_nodex8_list.back().node_size_in_children = children.size();

		// children id の登録
		for (std::size_t i = 0; i < children.size(); i++) {
			children_id[m_node_list[children[i]].self] = i;
		}

		// parent node に今作ったノードを登録
		const auto packed_parent = packed_parent_node[old_node_index];
		auto &parent = m_nodex8_list[packed_parent];
		assert(children_id[old_node_index] < 8);
		parent.children[children_id[old_node_index]] = packed_node_index;

		// leaf node でないものは、その子供も pack する必要がある
		for (std::size_t i = 0; i < children.size(); i++) {
			if (node_list[i]->is_leaf()) {
				m_nodex8_list.back().children[i] = Nodex8::LeafID;
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
	que.push(0);

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
			const std::size_t mesh_index_from = packed_leaf_index;
			node.start_mesh_index = node.leaf_meshid_from;

			for (std::size_t mesh_index = node.leaf_meshid_from;
				mesh_index < node.leaf_meshid_to; mesh_index += 8) {
				const std::size_t to_index =
					std::min(mesh_index + 8, node.leaf_meshid_to);
				m_packed_triangle_list.emplace_back(m_mesh_list.begin() + mesh_index,
					m_mesh_list.begin() + to_index);
				packed_leaf_index++;
			}
			node.leaf_meshid_from = mesh_index_from;
			node.leaf_meshid_to = packed_leaf_index;
		}
	}
}

bool SimpleBVH::construct(MeshTriangleList &&mesh_list) {
	m_mesh_list = std::forward<MeshTriangleList>(mesh_list);
	m_node_list.emplace_back(0);
	m_node_list.resize(m_mesh_list.size() * 2);

	const std::size_t num_mesh = m_mesh_list.size();

	std::vector<std::int32_t> mesh_index_list(num_mesh);

	std::vector<Vec3> center_position(num_mesh);
	for (std::size_t index = 0; index < num_mesh; index++) {
		m_mesh_list[index].center(&center_position[index]);
	}

	std::array<std::vector<std::int32_t>, 3> mesh_id_sorted_by;

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

	// AABB of range [0, index]
	std::vector<AABB> left_accumulated_AABB(num_mesh);

	// AABB of range [index, num_mesh - 1]
	std::vector<AABB> right_accumulated_AABB(num_mesh);

	TaskQueue<Range> que(m_mesh_list.size());
	que.push(Range(0, num_mesh, node_index));

	constexpr std::size_t num_threads = 1;
	std::vector<std::thread> threads(num_threads);

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
						current_node.set_leaf(range.from, range.to);
						// node is already cleared when node is created
						for (std::int32_t mesh_index = range.from; mesh_index < range.to;
							mesh_index++) {
							mesh_index_list[mesh_index] =
								mesh_id_sorted_by[AxisX][mesh_index];
							current_node.aabb.enlarge(
								m_mesh_list[mesh_index_list[mesh_index]]);
						}
						que.update(range.to - range.from);
					}
					else {
						std::int32_t best_axis, best_position;
						select_best_split(range, mesh_id_sorted_by, left_accumulated_AABB,
							right_accumulated_AABB, &best_axis,
							&best_position);

						if (best_axis == Invalid) {
							current_node.set_leaf(range.from, range.to);
							// node is already cleared when node is created
							for (std::int32_t mesh_index = range.from; mesh_index < range.to;
								mesh_index++) {
								mesh_index_list[mesh_index] =
									mesh_id_sorted_by[AxisX][mesh_index];
								current_node.aabb.enlarge(
									m_mesh_list[mesh_index_list[mesh_index]]);
							}
							que.update(range.to - range.from);
						}
						else {
							// node is already cleared when node is created
							current_node.aabb.enlarge(left_accumulated_AABB[range.to - 1]);

							adjust_index(mesh_id_sorted_by, is_left_flag, range, best_axis,
								best_position);

							current_node.left = ++node_index;
							m_node_list[current_node.left].self = current_node.left;
							current_node.right = ++node_index;
							m_node_list[current_node.right].self = current_node.right;

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

	m_node_list.resize(++node_index);
	m_node_list.shrink_to_fit();

	reorder_node(5);
	reorder_mesh(mesh_index_list);

	setup_packed_triangle();

	setup_packed_node();

	return true;
}

void SimpleBVH::validate_range(
	const std::array<std::vector<int32_t>, 3> &mesh_id_sorted_by,
	const Range &range) const noexcept {
	std::array<std::set<int32_t>, 3> mesh_id_set;
	for (auto axis : { AxisX, AxisY, AxisZ }) {
		for (std::int32_t index = range.from; index < range.to; index++) {
			mesh_id_set[axis].insert(mesh_id_sorted_by[axis][index]);
		}
	}
	assert(mesh_id_set[0] == mesh_id_set[1]);
	assert(mesh_id_set[0] == mesh_id_set[2]);
}

void SimpleBVH::validate_node_index() {
	std::set<std::int32_t> node_id_set;
	node_id_set.insert(0);
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
		for (std::int32_t mesh_index = node.leaf_meshid_from;
			mesh_index < node.leaf_meshid_to; mesh_index++) {
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
	std::set<std::int32_t> mesh_ids;
	for (auto node : m_node_list) {
		if (node.is_leaf()) {
			for (std::int32_t id = node.leaf_meshid_from; id < node.leaf_meshid_to;
				id++) {
				assert(0 <= id && id < m_mesh_list.size());
				mesh_ids.insert(id);
			}
		}
	}
	assert(mesh_ids.size() == m_mesh_list.size());
}

void SimpleBVH::validate_order(const std::vector<std::int32_t> &order) {
	std::set<std::int32_t> mesh_ids;
	for (auto new_id : order) {
		assert(0 <= new_id && new_id < order.size());
		mesh_ids.insert(new_id);
	}
	assert(mesh_ids.size() == order.size());
}

bool SimpleBVH::intersect(RayExt &ray) const {
	std::int32_t hitMeshIdx = 0;
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

bool SimpleBVH::intersectAnySub(std::int32_t nodeIndex, RayExt &ray,
	std::int32_t *hitMeshIndex) const {
	const auto &node = m_node_list[nodeIndex];

	if (node.is_leaf()) {
		for (std::int32_t mesh_index = node.leaf_meshid_from;
			mesh_index < node.leaf_meshid_to; mesh_index++) {
			if (m_packed_triangle_list[mesh_index].intersect_distance(ray) !=
				PackedTrianglex8::InvalidIndex) {
				return true;
			}
		}
		return false;
	} // 枝の場合は、子を見に行く
	else {
		const float left_aabb_distance =
			m_node_list[node.left].aabb.intersect_distance(ray, ray.tfar);
		const float right_aabb_distance =
			m_node_list[node.right].aabb.intersect_distance(ray, ray.tfar);

		std::int32_t near_node_id = node.left;
		std::int32_t far_node_id = node.right;
		if (std::isinf(left_aabb_distance) ||
			left_aabb_distance > right_aabb_distance) {
			std::swap(near_node_id, far_node_id);
		}
		const float near_distance =
			std::min(left_aabb_distance, right_aabb_distance);
		const float far_distance =
			std::max(left_aabb_distance, right_aabb_distance);

		if (near_distance < ray.tfar &&
			intersectAnySub(near_node_id, ray, hitMeshIndex)) {
			return true;
		}
		if (far_distance < ray.tfar &&
			intersectAnySub(far_node_id, ray, hitMeshIndex)) {
			return true;
		}
		return false;
	}
}

bool SimpleBVH::intersectAny(RayExt &ray) const {
	std::int32_t hitMeshIndex;
	return intersectAnySub(0, ray, &hitMeshIndex);
}

void SimpleBVH::select_best_split(
	const Range &range,
	const std::array<std::vector<std::int32_t>, 3> &index_sorted_by,
	std::vector<AABB> &left_accumulated_AABB,
	std::vector<AABB> &right_accumulated_AABB, std::int32_t *best_axis,
	std::int32_t *best_split_position) {
	*best_axis = Invalid;
	*best_split_position = Invalid;
	float best_sah_cost = std::numeric_limits<float>::max();

	for (const auto axis : { AxisX, AxisY, AxisZ }) {
		// register left accumulated AABB
		for (std::int32_t index = range.from; index < range.to; index++) {
			left_accumulated_AABB[index].clear();
			const std::int32_t mesh_id = index_sorted_by[axis][index];
			left_accumulated_AABB[index].enlarge(m_mesh_list[mesh_id]);
			if (range.from < index) {
				left_accumulated_AABB[index].enlarge(left_accumulated_AABB[index - 1]);
			}
		}

		// register right accumulated AABB
		for (std::int32_t index = range.to - 1; range.from <= index; index--) {
			right_accumulated_AABB[index].clear();
			const std::int32_t mesh_id = index_sorted_by[axis][index];
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
		for (std::int32_t split_position = range.from + 1;
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
	std::array<std::vector<std::int32_t>, 3> &index_sorted_by,
	std::vector<std::uint8_t> &is_left_flag, const Range &range,
	const std::int32_t selected_axis, const std::int32_t split_position) {
	// keep correctness of sort for all axis.

	for (std::size_t index = range.from; index < split_position; index++) {
		auto mesh_id = index_sorted_by[selected_axis][index];
		is_left_flag[mesh_id] = Left;
	}
	for (std::size_t index = split_position; index < range.to; index++) {
		auto mesh_id = index_sorted_by[selected_axis][index];
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

void SimpleBVH::visit_for_reorder(std::int32_t old_node_index,
	std::int32_t depth, std::int32_t max_depth,
	std::int32_t &new_node_index,
	std::queue<std::int32_t> &que,
	std::vector<std::int32_t> &order) {
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

void SimpleBVH::reorder_mesh(
	const std::vector<std::int32_t> &mesh_index_list) noexcept {
	const std::size_t mesh_size = m_mesh_list.size();
	std::vector<std::int32_t> order(mesh_size);

	std::int32_t new_mesh_index = 0;
	for (auto &node : m_node_list) {
		if (node.is_leaf()) {
			const std::int32_t new_meshid_from = new_mesh_index;
			for (std::int32_t old_mesh_index = node.leaf_meshid_from;
				old_mesh_index < node.leaf_meshid_to; old_mesh_index++) {
				order[mesh_index_list[old_mesh_index]] = new_mesh_index;
				new_mesh_index++;
			}
			node.leaf_meshid_from = new_meshid_from;
			node.leaf_meshid_to = new_mesh_index;
		}
	}
	assert(new_mesh_index == m_mesh_list.size());

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
	std::vector<std::int32_t> order(m_node_list.size());

	std::queue<std::int32_t> que;
	que.push(0);

	std::int32_t new_index = 0;

	while (!que.empty()) {
		int32_t old_node_index = que.front();
		que.pop();
		visit_for_reorder(old_node_index, 0, max_depth, new_index, que, order);
	}

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
	std::swap(next_node_list, m_node_list);
}

bool SimpleBVH::intersectSub(std::int32_t nodeIndex, RayExt &ray,
	std::int32_t *hitMeshIndex) const {
	const auto &node = m_node_list[nodeIndex];

	// 葉の場合は、ノードの三角形と交差判定
	if (node.is_leaf()) {
		bool success = false;
		for (std::int32_t mesh_index = node.leaf_meshid_from;
			mesh_index < node.leaf_meshid_to; mesh_index++) {
			const std::size_t subindex =
				m_packed_triangle_list[mesh_index].intersect_distance(ray);
			if (subindex != PackedTrianglex8::InvalidIndex) {
				*hitMeshIndex = node.start_mesh_index +
					8 * (mesh_index - node.leaf_meshid_from) + subindex;
				success = true;
			}
		}
		return success;
	} // 枝の場合は、子を見に行く
	else {
		const float left_aabb_distance =
			m_node_list[node.left].aabb.intersect_distance(ray, ray.tfar);
		const float right_aabb_distance =
			m_node_list[node.right].aabb.intersect_distance(ray, ray.tfar);

		std::int32_t near_node_id = node.left;
		std::int32_t far_node_id = node.right;
		if (std::isinf(left_aabb_distance) ||
			left_aabb_distance > right_aabb_distance) {
			std::swap(near_node_id, far_node_id);
		}
		const float near_distance =
			std::min(left_aabb_distance, right_aabb_distance);
		const float far_distance =
			std::max(left_aabb_distance, right_aabb_distance);

		bool update = false;
		if (near_distance < ray.tfar) {
			update |= intersectSub(near_node_id, ray, hitMeshIndex);
		}
		if (far_distance < ray.tfar) {
			update |= intersectSub(far_node_id, ray, hitMeshIndex);
		}
		return update;
	}
}
