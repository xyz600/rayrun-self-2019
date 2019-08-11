﻿#pragma once

#include "ray.hpp"
#include "triangle.hpp"
#include "vec.hpp"

using vector::Vec3;

class AABB {
public:
	static constexpr float InvalidDistance =
		std::numeric_limits<float>::infinity();

	AABB() noexcept;
	void clear() noexcept;
	void enlarge(const Vec3 &point) noexcept;
	void enlarge(const AABB &aabb) noexcept;
	void enlarge(const MeshTriangle &aabb) noexcept;

	float area() const noexcept;
	bool contain(const AABB &aabb) const noexcept;
	bool contain(const MeshTriangle &triangle) const noexcept;

	void center(Vec3 *result) const noexcept;
	const Vec3 &min() const noexcept;
	const Vec3 &max() const noexcept;
	Vec3 size() const noexcept;
	const Vec3 &operator[](int32_t index) const noexcept;
	bool intersect(const RayExt &ray, float currentIntersectT) const noexcept;
	float intersect_distance(const RayExt &ray, float currentIntersectT) const
		noexcept;

private:
	Vec3 min_position;
	Vec3 max_position;
};

bool AABB::contain(const AABB &aabb) const noexcept {
	return (min().x() <= aabb.min().x() && min().y() <= aabb.min().y() &&
		min().z() <= aabb.min().z()) &&
		(aabb.max().x() <= max().x() && aabb.max().y() <= max().y() &&
			aabb.max().z() <= max().z());
}

bool AABB::contain(const MeshTriangle &triangle) const noexcept {
	AABB aabb_for_triangle;
	aabb_for_triangle.clear();
	aabb_for_triangle.enlarge(triangle);
	return contain(aabb_for_triangle);
}

float AABB::area() const noexcept {
	const float dx = max_position.x() - min_position.x();
	const float dy = max_position.y() - min_position.y();
	const float dz = max_position.z() - min_position.z();
	return 2.0f * (dx * dy + dy * dz + dz * dx);
}

AABB::AABB() noexcept { clear(); }
void AABB::clear() noexcept {
	min_position.fill(std::numeric_limits<float>::max() / 2);
	max_position.fill(-std::numeric_limits<float>::max() / 2);
}

void AABB::enlarge(const Vec3 &point) noexcept {
	min_position = vector::min(point, min_position);
	max_position = vector::max(point, max_position);
}

void AABB::enlarge(const MeshTriangle &triangle) noexcept {
	enlarge(triangle.vertex(0));
	enlarge(triangle.vertex(1));
	enlarge(triangle.vertex(2));
}

void AABB::center(Vec3 *result) const noexcept {
	*result = (min_position + max_position) * 0.5f;
}

const Vec3 &AABB::min() const noexcept { return min_position; }

const Vec3 &AABB::max() const noexcept { return max_position; }

Vec3 AABB::size() const noexcept { return max() - min(); }

void AABB::enlarge(const AABB &aabb) noexcept {
	min_position = vector::min(aabb.min_position, min_position);
	max_position = vector::max(aabb.max_position, max_position);
}

const Vec3 &AABB::operator[](int32_t index) const noexcept {
	return *(&min_position + index);
}

bool AABB::intersect(const RayExt &ray, float currentIntersectT) const noexcept {
	return intersect_distance(ray, currentIntersectT) != InvalidDistance;
}

float AABB::intersect_distance(const RayExt &ray, float currentIntersectT) const
noexcept {
	const AABB &aabb = *this;
	float tmin, tmax, tymin, tymax, tzmin, tzmax;
	tmin = (aabb[ray.sign[0]].x() - ray.pos.x()) * ray.dinv.x();
	tmax = (aabb[1 - ray.sign[0]].x() - ray.pos.x()) * ray.dinv.x();
	tymin = (aabb[ray.sign[1]].y() - ray.pos.y()) * ray.dinv.y();
	tymax = (aabb[1 - ray.sign[1]].y() - ray.pos.y()) * ray.dinv.y();
	if ((tmin > tymax) || (tymin > tmax)) {
		return InvalidDistance;
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
		return InvalidDistance;
	}
	if (tzmin > tmin) {
		tmin = tzmin;
	}
	if (tzmax < tmax) {
		tmax = tzmax;
	}
	if ((tmin < currentIntersectT) && (ray.tnear < tmax) && (tmin < ray.tfar)) {
		return tmin;
	}
	else {
		return InvalidDistance;
	}
}

class PackedAABBx8 {
public:

	static constexpr float InvalidDistance =
		AABB::InvalidDistance;

	static constexpr std::size_t InvalidIndex =
		std::numeric_limits<std::size_t>::max();

	PackedAABBx8();
	PackedAABBx8(PackedAABBx8 &&src);

	void operator=(PackedAABBx8 &&src);

	void construct(const std::vector<AABB *> &aabb_list);

	void intersect_distance(const RayExt &ray, float currentIntersectT, std::array<float, 8> &distance_array) const noexcept;

	Vec3x8 min_position;
	Vec3x8 max_position;
private:
	std::size_t m_size;
};

PackedAABBx8::PackedAABBx8() {}

PackedAABBx8::PackedAABBx8(PackedAABBx8 &&src) {
	min_position.copy_from(src.min_position);
	max_position.copy_from(src.max_position);
	m_size = src.m_size;
}

void PackedAABBx8::operator=(PackedAABBx8 &&src) {
	min_position.copy_from(src.min_position);
	max_position.copy_from(src.max_position);
	m_size = src.m_size;
}

void PackedAABBx8::construct(const std::vector<AABB *> &aabb_list) {
	m_size = aabb_list.size();
	for (std::size_t i = 0; i < 8; i++) {
		const std::size_t index = std::min(m_size - 1, i);
		const auto &aabb = *aabb_list[index];
		min_position.set(aabb.min(), i);
		max_position.set(aabb.max(), i);
	}
}

void PackedAABBx8::intersect_distance(const RayExt &ray, float currentIntersectT, std::array<float, 8> &distance_array) const noexcept {

	for (std::size_t i = 0; i < m_size; i++) {
		AABB aabb;
		aabb.clear();
		auto mins = min_position.get(i);
		aabb.enlarge(mins);
		auto maxs = max_position.get(i);
		aabb.enlarge(maxs);

		float tmin, tmax, tymin, tymax, tzmin, tzmax;
		tmin = (aabb[ray.sign[0]].x() - ray.pos.x()) * ray.dinv.x();
		tmax = (aabb[1 - ray.sign[0]].x() - ray.pos.x()) * ray.dinv.x();
		tymin = (aabb[ray.sign[1]].y() - ray.pos.y()) * ray.dinv.y();
		tymax = (aabb[1 - ray.sign[1]].y() - ray.pos.y()) * ray.dinv.y();
		if ((tmin > tymax) || (tymin > tmax)) {
			distance_array[i] = InvalidDistance;
			continue;
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
			distance_array[i] = InvalidDistance;
			continue;
		}
		if (tzmin > tmin) {
			tmin = tzmin;
		}
		if (tzmax < tmax) {
			tmax = tzmax;
		}
		if ((tmin < currentIntersectT) && (ray.tnear < tmax) && (tmin < ray.tfar)) {
			distance_array[i] = tmin;
		}
		else {
			distance_array[i] = InvalidDistance;
		}
	}
}
