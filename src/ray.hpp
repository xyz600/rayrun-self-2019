#pragma once

#include "vec.hpp"

using vector::Vec3;

struct RayExt {
public:
  Vec3 pos;
  Vec3 dir;
  float tnear;
  float tfar;
  Vec3 dinv;
  std::array<bool, 3> sign;
  // Œğ·“_
  Vec3 isect;
  //
  float u;
  float v;
  //
  Vec3 ns;
  //
  int32_t faceid;
};
