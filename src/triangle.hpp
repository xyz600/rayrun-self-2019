#pragma once

#include "vec.hpp"

#include <array>

using vector::Vec3;

struct Face {
public:
  std::array<uint32_t, 3> idxVtx;
  std::array<uint32_t, 3> idxNorm;
};

struct MeshTriangle {
public:
  // ���_�ʒu
  std::array<Vec3, 3> v;
  // �@��
  std::array<Vec3, 3> n;
  // AABB
  AABB aabb;
  //
  int32_t faceid;
};

bool intersectTriangle(RayExt &ray, const Vec3 &v0, const Vec3 &v1,
                       const Vec3 &v2) {
  //
  float t, u, v;
  //
  Vec3 e1, e2;
  Vec3 P, Q, T;
  float det, inv_det;
  // v0�����L����G�b�W
  e1 = v1 - v0;
  e2 = v2 - v0;
  // det��u�̏���
  P = vector::cross(ray.dir, e2);
  // �قڕ��s�ȏꍇ�����`�F�b�N
  det = vector::dot(e1, P);
  if (det == 0.0f) {
    return false;
  }
  inv_det = 1.0f / det;
  // ���C���_����v0�ւ̋���
  T = ray.pos - v0;
  // u���v�Z���A�͈͓��Ɏ��܂��Ă��邩���`�F�b�N
  u = vector::dot(T, P) * inv_det;
  if (u < 0.0f || u > 1.0f) {
    return false;
  }
  // v�����l�̌v�Z
  Q = vector::cross(T, e1);
  v = vector::dot(ray.dir, Q) * inv_det;
  if (v < 0.0f || u + v > 1.0f) {
    return false;
  }
  // t�͈̔̓`�F�b�N
  t = vector::dot(e2, Q) * inv_det;
  if (t < ray.tnear || ray.tfar < t) {
    return false;
  }
  // �ʂ̕���
  // const bool isFlip = (det < 0.0f);
  //
  if (t >= ray.tfar) {
    return false;
  }
  //
  ray.tfar = t;
  ray.u = u;
  ray.v = v;
  return true;
}