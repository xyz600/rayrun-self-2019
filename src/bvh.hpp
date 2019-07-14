#pragma once

#include "bounding_box.hpp"
#include "ray.hpp"
#include "triangle.hpp"
#include "vec.hpp"

#include <array>
#include <cassert>
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
  bool intersectCheck(RayExt &ray) const;

private:
  struct Node {
  public:
    std::int32_t left;
    std::int32_t right;

    // AABB
    AABB aabb;

    std::size_t leaf_index;

    Node() noexcept {
      left = right = SimpleBVH::Invalid;
      leaf_index = SimpleBVH::Invalid;
      aabb.clear();
    }

    void set_leaf(const std::size_t leaf_index) noexcept {
      this->leaf_index = leaf_index;
      left = right = SimpleBVH::Invalid;
    }

    bool is_leaf() const noexcept {
      return left == SimpleBVH::Invalid && right == SimpleBVH::Invalid;
    }
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
                    std::int32_t *hitNodeIndex) const;

  void constructNode(const std::int32_t depth);

  void adjust_index(std::array<std::vector<std::int32_t>, 3> &index_sorted_by,
                    std::vector<std::uint8_t> &is_left_flag, const Range &range,
                    const std::int32_t axis, const std::int32_t position);

  void select_best_split(
      const Range &range,
      const std::array<std::vector<std::int32_t>, 3> &index_sorted_by,
      std::vector<AABB> &left_accumulated_AABB,
      std::vector<AABB> &right_accumulated_AABB, std::int32_t *best_axis,
      std::int32_t *best_position);

  void
  validate_range(const std::array<std::vector<int32_t>, 3> &mesh_id_sorted_by,
                 const Range &range) const noexcept;

  void
  validate(const std::array<std::vector<int32_t>, 3> &mesh_id_sorted_by) const
      noexcept;

  static constexpr std::int32_t AxisX = 0;
  static constexpr std::int32_t AxisY = 1;
  static constexpr std::int32_t AxisZ = 2;

  static constexpr std::uint8_t Left = 0;
  static constexpr std::uint8_t Right = 1;

  static constexpr std::int32_t Invalid =
      std::numeric_limits<std::int32_t>::max();

  static constexpr float CostBoundingBoxIntersect = 1.0;
  static constexpr float CostPolygonIntersect = 1.0;

private:
  // ノード
  std::vector<Node> m_node_list;
  MeshTriangleList m_mesh_list;
  std::vector<PackedTriangle> m_triangle_list;
};

// =================================
// implementation

SimpleBVH::SimpleBVH() {}

bool SimpleBVH::construct(MeshTriangleList &&mesh_list) {
  m_mesh_list = std::forward<MeshTriangleList>(mesh_list);

  m_node_list.reserve(m_mesh_list.size() * 2);
  m_node_list.resize(1);
  constructNode(0);
  m_node_list.shrink_to_fit();
  return true;
}

void SimpleBVH::validate_range(
    const std::array<std::vector<int32_t>, 3> &mesh_id_sorted_by,
    const Range &range) const noexcept {
  std::array<std::set<int32_t>, 3> mesh_id_set;
  for (auto axis : {AxisX, AxisY, AxisZ}) {
    for (std::int32_t index = range.from; index < range.to; index++) {
      mesh_id_set[axis].insert(mesh_id_sorted_by[axis][index]);
    }
  }
  assert(mesh_id_set[0] == mesh_id_set[1]);
  assert(mesh_id_set[0] == mesh_id_set[2]);
}

void SimpleBVH::validate(
    const std::array<std::vector<int32_t>, 3> &mesh_id_sorted_by) const
    noexcept {
  std::set<std::int32_t> ids;
  for (auto &node : m_node_list) {
    if (node.is_leaf()) {
      assert(0 <= node.leaf_index && node.leaf_index < m_mesh_list.size());
      ids.insert(node.leaf_index);
    }
  }
  assert(ids.size() == m_mesh_list.size());
}

bool SimpleBVH::intersect(RayExt &ray) const {
  std::int32_t hitNodeIdx = 0;
  const bool isHit = intersectSub(0, ray, &hitNodeIdx);
  if (isHit) {
    const Node &node = m_node_list[hitNodeIdx];
    const auto &mesh = m_mesh_list[node.leaf_index];
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

bool SimpleBVH::intersectCheck(RayExt &ray) const {
  // intersect()をそのまま流用
  std::int32_t hitNodeIdx = 0;
  return intersectSub(0, ray, &hitNodeIdx);
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

  for (const auto axis : {AxisX, AxisY, AxisZ}) {
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

  for (auto axis : {AxisX, AxisY, AxisZ}) {
    if (selected_axis != axis) {
      // sorted as [left_left, left_right, right_left, right_right]
      std::stable_partition(index_sorted_by[axis].begin() + range.from,
                            index_sorted_by[axis].begin() + range.to,
                            [&](const std::size_t mesh_id) {
                              return is_left_flag[mesh_id] == Left;
                            });
    }
  }
}

void SimpleBVH::constructNode(const std::int32_t depth) {
  const std::size_t num_mesh = m_mesh_list.size();

  std::vector<Vec3> center_position(num_mesh);
  for (std::size_t index = 0; index < num_mesh; index++) {
    m_mesh_list[index].center(&center_position[index]);
  }

  std::array<std::vector<std::int32_t>, 3> mesh_id_sorted_by;

  for (auto axis : {AxisX, AxisY, AxisZ}) {
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

  std::int32_t leaf_index = 0;
  std::int32_t node_index = 0;

  // AABB of range [0, index]
  std::vector<AABB> left_accumulated_AABB(num_mesh);

  // AABB of range [index, num_mesh - 1]
  std::vector<AABB> right_accumulated_AABB(num_mesh);

  std::queue<Range> que;
  que.emplace(0, num_mesh, node_index);

  while (!que.empty()) {
    auto range = que.front();
    que.pop();

    validate_range(mesh_id_sorted_by, range);

    auto &current_node = m_node_list[range.node_index];

    if (range.size() == 1) {
      // if size == 1, any axis is correct(so AxisX is selected)
      const std::int32_t mesh_id = mesh_id_sorted_by[AxisX][range.from];
      current_node.set_leaf(mesh_id);
      // node is already cleared when node is created
      current_node.aabb.enlarge(m_mesh_list[mesh_id]);

    } else {

      std::int32_t best_axis, best_position;
      select_best_split(range, mesh_id_sorted_by, left_accumulated_AABB,
                        right_accumulated_AABB, &best_axis, &best_position);

      // node is already cleared when node is created
      current_node.aabb.enlarge(left_accumulated_AABB[range.to - 1]);

      adjust_index(mesh_id_sorted_by, is_left_flag, range, best_axis,
                   best_position);

      current_node.left = node_index + 1;
      m_node_list.emplace_back();
      current_node.right = node_index + 2;
      m_node_list.emplace_back();

      // Range left{range.from, best_position, node_index + 1};
      //  validate_range(mesh_id_sorted_by, left);
      //  Range right{best_position, range.to, node_index + 2};
      //  validate_range(mesh_id_sorted_by, right);

      node_index += 2;
      que.emplace(range.from, best_position, current_node.left);

      que.emplace(best_position, range.to, current_node.right);
    }
  }
  // validate(mesh_id_sorted_by);
}

bool SimpleBVH::intersectSub(std::int32_t nodeIndex, RayExt &ray,
                             std::int32_t *hitNodeIndex) const {
  const auto &node = m_node_list[nodeIndex];
  // このAABBに交差しなければ終了
  if (!node.aabb.intersectCheck(ray, ray.tfar)) {
    return false;
  }
  // 葉の場合は、ノードの三角形と交差判定
  else if (node.is_leaf()) {
    auto &mesh = m_mesh_list[node.leaf_index];
    if (intersectTriangle(ray, mesh.vertex(0), mesh.vertex(1),
                          mesh.vertex(2))) {
      *hitNodeIndex = nodeIndex;
      return true;
    }
    return false;
  }
  // 枝の場合は、子を見に行く
  else {
    const bool h0 = intersectSub(node.left, ray, hitNodeIndex);
    const bool h1 = intersectSub(node.right, ray, hitNodeIndex);
    return h0 || h1;
  }
}