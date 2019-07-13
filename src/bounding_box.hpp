#pragma once

#include "ray.hpp"
#include "vec.hpp"

using vector::Vec3;

class AABB {
public:
  AABB();
  void clear();
  void extend(const Vec3 &point);
  void extend(const AABB &aabb);
  const Vec3 &center() const;
  const Vec3 &min() const;
  const Vec3 &max() const;
  Vec3 size() const;
  const Vec3 &operator[](int32_t index) const;
  bool intersectCheck(const RayExt &ray, float currentIntersectT) const;

private:
  Vec3 mn;
  Vec3 mx;
};

AABB::AABB() { clear(); }
void AABB::clear() {
  const float maxv = std::numeric_limits<float>::max();
  const float minv = std::numeric_limits<float>::lowest();
  mn = Vec3{maxv, maxv, maxv};
  mx = Vec3{minv, minv, minv};
}

void AABB::extend(const Vec3 &point) {
  mn = vector::min(point, mn);
  mx = vector::max(point, mx);
}

const Vec3 &AABB::center() const { return (mn + mx) * 0.5f; }

const Vec3 &AABB::min() const { return mn; }

const Vec3 &AABB::max() const { return mx; }

Vec3 AABB::size() const { return max() - min(); }

void AABB::extend(const AABB &aabb) {
  mn = vector::min(aabb.mn, mn);
  mx = vector::max(aabb.mx, mx);
}

const Vec3 &AABB::operator[](int32_t index) const { return *(&mn + index); }

bool AABB::intersectCheck(const RayExt &ray, float currentIntersectT) const {
  const AABB &aabb = *this;
  float tmin, tmax, tymin, tymax, tzmin, tzmax;
  tmin = (aabb[ray.sign[0]].x() - ray.pos.x()) * ray.dinv.x();
  tmax = (aabb[1 - ray.sign[0]].x() - ray.pos.x()) * ray.dinv.x();
  tymin = (aabb[ray.sign[1]].y() - ray.pos.y()) * ray.dinv.y();
  tymax = (aabb[1 - ray.sign[1]].y() - ray.pos.y()) * ray.dinv.y();
  if ((tmin > tymax) || (tymin > tmax)) {
    return false;
  }
  if (tymin > tmin) {
    tmin = tymin;
  }
  if (tymax < tmax) {
    tmax = tymax;
  }
  tzmin = (aabb[ray.sign[2]].z() - ray.pos.z()) * ray.dinv.z();
  tzmax = (aabb[1 - ray.sign[2]].z() - ray.pos.z()) * ray.dinv.z();
  if ((tmin > tzmax) || (tzmin > tmax)) {
    return false;
  }
  if (tzmin > tmin) {
    tmin = tzmin;
  }
  if (tzmax < tmax) {
    tmax = tzmax;
  }
  return (tmin < currentIntersectT) && (ray.tnear < tmax) && (tmin < ray.tfar);
}