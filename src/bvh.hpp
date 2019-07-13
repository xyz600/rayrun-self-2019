#pragma once

#include "bounding_box.hpp"
#include "ray.hpp"
#include "triangle.hpp"
#include "vec.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

using vector::Vec3;

class SimpleBVH {

public:
  SimpleBVH();
  virtual ~SimpleBVH();
  bool construct(const std::vector<Vec3> &vs, const std::vector<Vec3> &ns,
                 const std::vector<Face> &fs);
  bool intersect(RayExt &ray) const;
  bool intersectCheck(RayExt &ray) const;

private:
  struct Node {
  public:
    // 枝であった場合の子のノードインデックス。葉の場合は全て-1が格納されている。
    std::int32_t childlen[2];
    // AABB
    AABB aabb;
    // 葉であった場合の頂点座標。枝の場合は無効な値。
    std::array<Vec3, 3> v;
    // 葉であった場合の法線。枝の場合は無効な値。
    std::array<Vec3, 3> n;
    //
    std::int32_t faceid = 0;
  };

  bool intersectSub(std::int32_t nodeIndex, RayExt &ray,
                    std::int32_t *hitNodeIndex) const;

  void constructNode(std::int32_t nodeIndex, MeshTriangle *triangles,
                     std::int32_t numTriangle, std::int32_t depth);

private:
  std::vector<Vec3> vs_;
  std::vector<Face> fs_;
  // 各三角形のAABB
  std::vector<AABB> nodeAABBs_;
  // ノード
  std::vector<Node> nodes_;
};

// =================================
// implementation

SimpleBVH::SimpleBVH() {}
SimpleBVH::~SimpleBVH() {}

bool SimpleBVH::construct(const std::vector<Vec3> &vs,
                          const std::vector<Vec3> &ns,
                          const std::vector<Face> &fs) {
  vs_ = vs;
  fs_ = fs;
  // 全三角形のデータをまとめたものを作成する
  const std::int32_t faceNum = (std::int32_t)fs_.size();
  std::vector<MeshTriangle> triangles;
  triangles.reserve(faceNum);
  for (std::int32_t faceNo = 0; faceNo < faceNum; ++faceNo) {
    //
    const Face &face = fs_[faceNo];
    //
    MeshTriangle tri;
    tri.v[0] = vs[face.idxVtx[0]];
    tri.v[1] = vs[face.idxVtx[1]];
    tri.v[2] = vs[face.idxVtx[2]];
    tri.n[0] = ns[face.idxNorm[0]];
    tri.n[1] = ns[face.idxNorm[1]];
    tri.n[2] = ns[face.idxNorm[2]];
    tri.aabb.clear();
    tri.aabb.extend(tri.v[0]);
    tri.aabb.extend(tri.v[1]);
    tri.aabb.extend(tri.v[2]);
    tri.faceid = faceNo;
    triangles.push_back(tri);
  }

  nodes_.reserve(faceNum * 2);
  nodes_.resize(1);
  constructNode(0, triangles.data(), (std::int32_t)triangles.size(), 0);
  nodes_.shrink_to_fit();
  return true;
}

bool SimpleBVH::intersect(RayExt &ray) const {
  std::int32_t hitNodeIdx = 0;
  const bool isHit = intersectSub(0, ray, &hitNodeIdx);
  if (isHit) {
    const Node &node = nodes_[hitNodeIdx];
    const float u = ray.u;
    const float v = ray.v;
    ray.isect = node.v[0] * (1.0f - u - v) + node.v[1] * u + node.v[2] * v;
    ray.ns = node.n[0] * (1.0f - u - v) + node.n[1] * u + node.n[2] * v;
    ray.faceid = node.faceid;
  }
  return isHit;
}

bool SimpleBVH::intersectCheck(RayExt &ray) const {
  // intersect()をそのまま流用
  std::int32_t hitNodeIdx = 0;
  return intersectSub(0, ray, &hitNodeIdx);
}

void SimpleBVH::constructNode(std::int32_t nodeIndex, MeshTriangle *triangles,
                              std::int32_t numTriangle, std::int32_t depth) {
  // このノードのAABBを求める
  auto &curNode = nodes_[nodeIndex];
  curNode.childlen[0] = -1;
  curNode.childlen[1] = -1;
  curNode.aabb.clear();
  for (std::int32_t triNo = 0; triNo < numTriangle; ++triNo) {
    curNode.aabb.extend(triangles[triNo].aabb);
  }
  // 三角形が一つしかない場合は葉
  if (numTriangle == 1) {
    auto &tri = triangles[0];
    auto &v = tri.v;
    auto &n = tri.n;
    curNode.v[0] = v[0];
    curNode.v[1] = v[1];
    curNode.v[2] = v[2];
    curNode.n[0] = n[0];
    curNode.n[1] = n[1];
    curNode.n[2] = n[2];
    curNode.faceid = tri.faceid;
    return;
  }
  // 軸と分割位置を適当に決める
  static std::int32_t axisNext = 0;
  std::int32_t axis = (axisNext++) % 3;
  // 三角形ソート
  auto sortPred = [&axis](const MeshTriangle &lhs, const MeshTriangle &rhs) {
    auto lhsc = lhs.aabb.center();
    auto rhsc = rhs.aabb.center();
    return lhsc[axis] < rhsc[axis];
  };
  std::int32_t bestTriIndex = numTriangle / 2;
  std::sort(triangles, triangles + numTriangle, sortPred);

  nodes_.resize(nodes_.size() + 1);
  curNode.childlen[0] = (std::int32_t)nodes_.size() - 1;
  constructNode(curNode.childlen[0], triangles, bestTriIndex, depth + 1);
  nodes_.resize(nodes_.size() + 1);
  curNode.childlen[1] = (std::int32_t)nodes_.size() - 1;
  constructNode(curNode.childlen[1], triangles + bestTriIndex,
                numTriangle - bestTriIndex, depth + 1);
}

bool SimpleBVH::intersectSub(std::int32_t nodeIndex, RayExt &ray,
                             std::int32_t *hitNodeIndex) const {

  const auto &node = nodes_[nodeIndex];
  // このAABBに交差しなければ終了
  if (!node.aabb.intersectCheck(ray, ray.tfar)) {
    return false;
  }
  // 葉の場合は、ノードの三角形と交差判定
  else if (node.childlen[0] == -1) {
    auto &v = node.v;
    if (intersectTriangle(ray, v[0], v[1], v[2])) {
      *hitNodeIndex = nodeIndex;
      return true;
    }
    return false;
  }
  // 枝の場合は、子を見に行く
  else {
    const bool h0 = intersectSub(node.childlen[0], ray, hitNodeIndex);
    const bool h1 = intersectSub(node.childlen[1], ray, hitNodeIndex);
    return h0 || h1;
  }
}