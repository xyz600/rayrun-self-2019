#pragma once

#include "bounding_box.hpp"
#include "vec.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

using vector::Vec3;

struct Face {
public:
  std::array<uint32_t, 3> vertex_index;
  std::array<uint32_t, 3> normal_index;
};

class MeshTriangle {
public:
  MeshTriangle() = default;
  MeshTriangle(const Vec3 *vertex_root, const Vec3 *normal_root,
               const Face &face, const std::uint32_t faceid) noexcept;
  const Vec3 &vertex(const std::size_t index) const noexcept;
  const Vec3 &normal(const std::size_t index) const noexcept;

  void copy_from(const MeshTriangle &src) noexcept;

  const std::uint32_t &faceid() const noexcept;
  std::uint32_t &faceid() noexcept;

  void center(Vec3 *result) const noexcept;

private:
  std::array<std::uint32_t, 3> m_vertex_index;
  std::array<std::uint32_t, 3> m_normal_index;
  const Vec3 *m_vertex_root;
  const Vec3 *m_normal_root;
  std::uint32_t m_faceid;
};

void MeshTriangle::copy_from(const MeshTriangle &src) noexcept {
  std::copy(src.m_vertex_index.begin(), src.m_vertex_index.end(),
            m_vertex_index.begin());
  std::copy(src.m_normal_index.begin(), src.m_normal_index.end(),
            m_normal_index.begin());
  m_vertex_root = src.m_vertex_root;
  m_normal_root = src.m_normal_root;
  m_faceid = src.m_faceid;
}

const std::uint32_t &MeshTriangle::faceid() const noexcept { return m_faceid; }
std::uint32_t &MeshTriangle::faceid() noexcept { return m_faceid; }

MeshTriangle::MeshTriangle(const Vec3 *vertex_root, const Vec3 *normal_root,
                           const Face &face,
                           const std::uint32_t faceid) noexcept
    : m_vertex_root(vertex_root), m_normal_root(normal_root), m_faceid(faceid) {
  std::copy(face.vertex_index.begin(), face.vertex_index.end(),
            m_vertex_index.begin());
  std::copy(face.normal_index.begin(), face.normal_index.end(),
            m_normal_index.begin());
}

const Vec3 &MeshTriangle::vertex(const std::size_t index) const noexcept {
  return m_vertex_root[m_vertex_index[index]];
}

const Vec3 &MeshTriangle::normal(const std::size_t index) const noexcept {
  return m_normal_root[m_normal_index[index]];
}

void MeshTriangle::center(Vec3 *result) const noexcept {
  *result = (vertex(0) + vertex(1) + vertex(2)) / 3.0f;
}

class MeshTriangleList {
public:
  using iterator = std::vector<MeshTriangle>::iterator;
  using const_iterator = std::vector<MeshTriangle>::const_iterator;

  iterator begin() noexcept;
  const_iterator begin() const noexcept;
  iterator end() noexcept;
  const_iterator end() const noexcept;

  MeshTriangleList() noexcept;
  MeshTriangleList(MeshTriangleList &&src) noexcept;
  MeshTriangleList(std::vector<Face> &&face_list,
                   std::vector<Vec3> &&vertex_list,
                   std::vector<Vec3> &&normal_list) noexcept;

  void operator=(MeshTriangleList &&src) noexcept;

  const MeshTriangle &operator[](const std::size_t index) const noexcept;
  MeshTriangle &operator[](const std::size_t index) noexcept;

  std::size_t size() const noexcept;

private:
  std::vector<Vec3> m_vertex_list;
  std::vector<Vec3> m_normal_list;
  std::vector<MeshTriangle> m_mesh_list;
};

void MeshTriangleList::operator=(MeshTriangleList &&src) noexcept {
  m_vertex_list = std::move(src.m_vertex_list);
  m_normal_list = std::move(src.m_normal_list);
  m_mesh_list = std::move(src.m_mesh_list);
}

MeshTriangleList::MeshTriangleList(MeshTriangleList &&src) noexcept {
  m_vertex_list = std::move(src.m_vertex_list);
  m_normal_list = std::move(src.m_normal_list);
  m_mesh_list = std::move(src.m_mesh_list);
}

typename MeshTriangleList::iterator MeshTriangleList::begin() noexcept {
  return m_mesh_list.begin();
}
typename MeshTriangleList::const_iterator MeshTriangleList::begin() const
    noexcept {
  return m_mesh_list.begin();
}
typename MeshTriangleList::iterator MeshTriangleList::end() noexcept {
  return m_mesh_list.end();
}
typename MeshTriangleList::const_iterator MeshTriangleList::end() const
    noexcept {
  return m_mesh_list.end();
}

MeshTriangleList::MeshTriangleList() noexcept {}

std::size_t MeshTriangleList::size() const noexcept {
  return m_mesh_list.size();
}

MeshTriangleList::MeshTriangleList(std::vector<Face> &&face_list,
                                   std::vector<Vec3> &&vertex_list,
                                   std::vector<Vec3> &&normal_list) noexcept
    : m_vertex_list(vertex_list), m_normal_list(normal_list) {
  for (std::size_t face_id = 0; face_id < face_list.size(); face_id++) {
    m_mesh_list.emplace_back(m_vertex_list.data(), m_normal_list.data(),
                             face_list[face_id], face_id);
  }
}

const MeshTriangle &MeshTriangleList::operator[](const std::size_t index) const
    noexcept {
  return m_mesh_list[index];
}

MeshTriangle &MeshTriangleList::operator[](const std::size_t index) noexcept {
  return m_mesh_list[index];
}

class PackedTriangle {
public:
  static constexpr float InvalidDistance =
      std::numeric_limits<float>::infinity();

  PackedTriangle(const MeshTriangle &src) noexcept;
  bool intersect(RayExt &ray) const noexcept;
  float intersect_distance(RayExt &ray) const noexcept;

private:
  Vec3 m_e1;
  Vec3 m_e2;
  Vec3 m_origin;
};

PackedTriangle::PackedTriangle(const MeshTriangle &src) noexcept {
  m_e1 = src.vertex(1) - src.vertex(0);
  m_e2 = src.vertex(2) - src.vertex(0);
  m_origin = src.vertex(0);
}

bool PackedTriangle::intersect(RayExt &ray) const noexcept {
  float t, u, v;

  // detとuの準備
  Vec3 P = vector::cross(ray.dir, m_e2);

  // ほぼ平行な場合かをチェック
  float det = vector::dot(m_e1, P);
  if (det == 0.0f) {
    return false;
  }
  float inv_det = 1.0f / det;

  // レイ原点からv0への距離
  Vec3 T = ray.pos - m_origin;

  // uを計算し、範囲内に収まっているかをチェック
  u = vector::dot(T, P) * inv_det;
  if (u < 0.0f || u > 1.0f) {
    return false;
  }

  // vも同様の計算
  Vec3 Q = vector::cross(T, m_e1);
  v = vector::dot(ray.dir, Q) * inv_det;
  if (v < 0.0f || u + v > 1.0f) {
    return false;
  }

  // tの範囲チェック
  t = vector::dot(m_e2, Q) * inv_det;
  if (t < ray.tnear || ray.tfar < t) {
    return false;
  }

  if (t >= ray.tfar) {
    return false;
  }

  //
  ray.tfar = t;
  ray.u = u;
  ray.v = v;
  return true;
}

float PackedTriangle::intersect_distance(RayExt &ray) const noexcept {
  float t, u, v;

  // detとuの準備
  Vec3 P = vector::cross(ray.dir, m_e2);

  // ほぼ平行な場合かをチェック
  float det = vector::dot(m_e1, P);
  if (det == 0.0f) {
    return InvalidDistance;
  }
  float inv_det = 1.0f / det;

  // レイ原点からv0への距離
  Vec3 T = ray.pos - m_origin;

  // uを計算し、範囲内に収まっているかをチェック
  u = vector::dot(T, P) * inv_det;
  if (u < 0.0f || u > 1.0f) {
    return InvalidDistance;
  }

  // vも同様の計算
  Vec3 Q = vector::cross(T, m_e1);
  v = vector::dot(ray.dir, Q) * inv_det;
  if (v < 0.0f || u + v > 1.0f) {
    return InvalidDistance;
  }

  // tの範囲チェック
  t = vector::dot(m_e2, Q) * inv_det;
  if (t < ray.tnear || ray.tfar < t) {
    return InvalidDistance;
  }

  //
  ray.tfar = t;
  ray.u = u;
  ray.v = v;
  return t;
}