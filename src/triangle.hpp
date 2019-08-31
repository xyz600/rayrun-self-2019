#pragma once

#include "bounding_box.hpp"
#include "matrix.hpp"
#include "vec.hpp"

#include <intrin.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

using vector::Vec3, vector::PackedVec3, vector::Vec3x8, vector::Vec3x16;

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

	bool validate() const noexcept;

private:
	std::array<std::uint32_t, 3> m_vertex_index;
	std::array<std::uint32_t, 3> m_normal_index;
	const Vec3 *m_vertex_root;
	const Vec3 *m_normal_root;
	std::uint32_t m_faceid;
};

bool MeshTriangle::validate() const noexcept {
	const auto &v0 = vertex(0);
	const auto &v1 = vertex(1);
	const auto &v2 = vertex(2);
	const Vec3 d1 = v1 - v0;
	const Vec3 d2 = v2 - v0;
	const auto ip = dot(d1, d2);
	const auto area = norm2(d1) * norm2(d2);
	// ip と area で乗算の順序が違うので、厳密に eq にならない
	return abs(abs(ip * ip) - area) / area > 1e-6;
}

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
		if (!m_mesh_list.back().validate()) {
			m_mesh_list.pop_back();
		}
	}
}

const MeshTriangle &MeshTriangleList::operator[](const std::size_t index) const
noexcept {
	return m_mesh_list[index];
}

MeshTriangle &MeshTriangleList::operator[](const std::size_t index) noexcept {
	return m_mesh_list[index];
}

class Triangle {
public:
	static constexpr float InvalidDistance =
		std::numeric_limits<float>::infinity();

	Triangle(const MeshTriangle &src) noexcept;
	bool intersect(RayExt &ray) const noexcept;
	float intersect_distance(RayExt &ray) const noexcept;

	std::array<Vec3, 3> ps;
	Vec3 offset;
};

Triangle::Triangle(const MeshTriangle &src) noexcept {
	Matrix4x4F p;
	const Vec3 d1 = src.vertex(1) - src.vertex(0);
	const Vec3 d2 = src.vertex(2) - src.vertex(0);
	const Vec3 normal = normalize(vector::cross(d1, d2));

	for (std::size_t i = 0; i < 3; i++) {
		p[i][0] = d1[i];
		p[i][1] = d2[i];
		p[i][2] = normal[i];
		p[i][3] = src.vertex(0)[i];
	}
	p[3][0] = 0;
	p[3][1] = 0;
	p[3][2] = 0;
	p[3][3] = 1;
	auto inverse = p.inverse();
	for (std::size_t i = 0; i < 3; i++) {
		ps[0][i] = inverse[0][i];
		ps[1][i] = inverse[1][i];
		ps[2][i] = inverse[2][i];
		offset[i] = inverse[i][3];
	}
}

bool Triangle::intersect(RayExt &ray) const noexcept {
	return intersect_distance(ray) != InvalidDistance;
}

float Triangle::intersect_distance(RayExt &ray) const noexcept {
	Vec3 o, n;
	for (std::size_t i = 0; i < 3; i++) {
		o[i] = vector::dot(ps[i], ray.pos);
		n[i] = vector::dot(ps[i], ray.dir);
	}
	o += offset;

	const float t = -o[2] / n[2];
	const float u = o[0] + t * n[0];
	const float v = o[1] + t * n[1];

	if (ray.tnear < t && t < ray.tfar && 0 <= u && 0 <= v && u + v <= 1) {
		ray.tfar = t;
		ray.u = u;
		ray.v = v;
		return t;
	}
	else {
		return InvalidDistance;
	}
}

template<std::size_t N>
class PackedTriangle {
public:

	using meshid_index_t = std::int32_t;

	static constexpr float InvalidDistance =
		std::numeric_limits<float>::infinity();

	static constexpr meshid_index_t InvalidIndex =
		std::numeric_limits<meshid_index_t>::max();

	PackedTriangle(
		MeshTriangleList::const_iterator begin,
		MeshTriangleList::const_iterator end) noexcept;

	std::size_t intersect_distance(RayExt &ray) const noexcept;

	std::size_t size() const noexcept;

private:
	std::array<PackedVec3<N>, 3> m_ps;
	PackedVec3<N> m_offset;
	std::size_t m_size;
};

template<std::size_t N>
std::size_t PackedTriangle<N>::size() const noexcept { return m_size; }

template<std::size_t N>
PackedTriangle<N>::PackedTriangle(
	MeshTriangleList::const_iterator begin,
	MeshTriangleList::const_iterator end) noexcept
	: m_size(std::distance(begin, end)) {
	for (std::size_t i = 0; i < 8; i++) {
		const std::size_t index = std::min(i, m_size - 1);
		Triangle triangle(*(begin + index));
		m_ps[0].set(triangle.ps[0], i);
		m_ps[1].set(triangle.ps[1], i);
		m_ps[2].set(triangle.ps[2], i);
		m_offset.set(triangle.offset, i);
	}
}

template<std::size_t N>
std::size_t PackedTriangle<N>::intersect_distance(RayExt &ray) const noexcept {
	static_assert(false);
}

template<>
std::size_t PackedTriangle<8>::intersect_distance(RayExt &ray) const noexcept {
	__m256 pos_xs = _mm256_set1_ps(ray.pos[0]);
	__m256 pos_ys = _mm256_set1_ps(ray.pos[1]);
	__m256 pos_zs = _mm256_set1_ps(ray.pos[2]);

	__m256 dir_xs = _mm256_set1_ps(ray.dir[0]);
	__m256 dir_ys = _mm256_set1_ps(ray.dir[1]);
	__m256 dir_zs = _mm256_set1_ps(ray.dir[2]);

	__m256 offset_zs = _mm256_loadu_ps(m_offset.zs().data());

	const auto dotx8 = [&](const __m256 &xs1, const __m256 &ys1,
		const __m256 &zs1, const __m256 &xs2,
		const __m256 &ys2, const __m256 &zs2) {
		__m256 ret = _mm256_mul_ps(xs1, xs2);
		ret = _mm256_fmadd_ps(ys1, ys2, ret);
		return _mm256_fmadd_ps(zs1, zs2, ret);
	};

	__m256 ps_xs = _mm256_loadu_ps(m_ps[2].xs().data());
	__m256 ps_ys = _mm256_loadu_ps(m_ps[2].ys().data());
	__m256 ps_zs = _mm256_loadu_ps(m_ps[2].zs().data());

	__m256 os = _mm256_add_ps(offset_zs,
		dotx8(ps_xs, ps_ys, ps_zs, pos_xs, pos_ys, pos_zs));
	__m256 ns = dotx8(ps_xs, ps_ys, ps_zs, dir_xs, dir_ys, dir_zs);
	__m256 ts = _mm256_sub_ps(_mm256_set1_ps(0.0), _mm256_div_ps(os, ns));

	ps_xs = _mm256_loadu_ps(m_ps[0].xs().data());
	ps_ys = _mm256_loadu_ps(m_ps[0].ys().data());
	ps_zs = _mm256_loadu_ps(m_ps[0].zs().data());

	__m256 offset_xs = _mm256_loadu_ps(m_offset.xs().data());
	os = _mm256_add_ps(offset_xs,
		dotx8(ps_xs, ps_ys, ps_zs, pos_xs, pos_ys, pos_zs));
	ns = dotx8(ps_xs, ps_ys, ps_zs, dir_xs, dir_ys, dir_zs);
	__m256 us = _mm256_fmadd_ps(ts, ns, os);

	ps_xs = _mm256_loadu_ps(m_ps[1].xs().data());
	ps_ys = _mm256_loadu_ps(m_ps[1].ys().data());
	ps_zs = _mm256_loadu_ps(m_ps[1].zs().data());

	__m256 offset_ys = _mm256_loadu_ps(m_offset.ys().data());
	os = _mm256_add_ps(offset_ys,
		dotx8(ps_xs, ps_ys, ps_zs, pos_xs, pos_ys, pos_zs));
	ns = dotx8(ps_xs, ps_ys, ps_zs, dir_xs, dir_ys, dir_zs);
	__m256 vs = _mm256_fmadd_ps(ts, ns, os);

	__m256 tnears = _mm256_set1_ps(ray.tnear);
	__m256 tfars = _mm256_set1_ps(ray.tfar);
	__m256 zeros = _mm256_set1_ps(0.0);
	__m256 ones = _mm256_set1_ps(1.0);

	constexpr int LESS_THAN = 2;

	__m256 mask = _mm256_cmp_ps(tnears, ts, LESS_THAN);
	mask = _mm256_and_ps(mask, _mm256_cmp_ps(ts, tfars, LESS_THAN));
	mask = _mm256_and_ps(mask, _mm256_cmp_ps(zeros, us, LESS_THAN));
	mask = _mm256_and_ps(mask, _mm256_cmp_ps(zeros, vs, LESS_THAN));
	mask = _mm256_and_ps(mask,
		_mm256_cmp_ps(_mm256_add_ps(us, vs), ones, LESS_THAN));

	const int packed_mask = _mm256_movemask_ps(mask);
	if (packed_mask == 0) {
		return InvalidIndex;
	}
	else {
		std::size_t index = InvalidIndex;
		std::array<float, 8> buf;
		_mm256_store_ps(buf.data(), ts);
		for (int i = 0; i < m_size; i++) {
			if (((packed_mask & (1 << i)) != 0) && buf[i] < ray.tfar) {
				index = i;
				ray.tfar = buf[i];
			}
		}
		if (index != InvalidIndex) {
			_mm256_store_ps(buf.data(), us);
			ray.u = buf[index];
			_mm256_store_ps(buf.data(), vs);
			ray.v = buf[index];
		}
		return index;
	}
}

template<>
std::size_t PackedTriangle<16>::intersect_distance(RayExt &ray) const noexcept {
	__m256 pos_xs = _mm256_set1_ps(ray.pos[0]);
	__m256 pos_ys = _mm256_set1_ps(ray.pos[1]);
	__m256 pos_zs = _mm256_set1_ps(ray.pos[2]);

	__m256 dir_xs = _mm256_set1_ps(ray.dir[0]);
	__m256 dir_ys = _mm256_set1_ps(ray.dir[1]);
	__m256 dir_zs = _mm256_set1_ps(ray.dir[2]);

	__m256 offset_zs = _mm256_loadu_ps(m_offset.zs().data());

	const auto dotx8 = [&](const __m256 &xs1, const __m256 &ys1,
		const __m256 &zs1, const __m256 &xs2,
		const __m256 &ys2, const __m256 &zs2) {
		__m256 ret = _mm256_mul_ps(xs1, xs2);
		ret = _mm256_fmadd_ps(ys1, ys2, ret);
		return _mm256_fmadd_ps(zs1, zs2, ret);
	};

	__m256 ps_xs = _mm256_loadu_ps(m_ps[2].xs().data());
	__m256 ps_ys = _mm256_loadu_ps(m_ps[2].ys().data());
	__m256 ps_zs = _mm256_loadu_ps(m_ps[2].zs().data());

	__m256 os = _mm256_add_ps(offset_zs,
		dotx8(ps_xs, ps_ys, ps_zs, pos_xs, pos_ys, pos_zs));
	__m256 ns = dotx8(ps_xs, ps_ys, ps_zs, dir_xs, dir_ys, dir_zs);
	__m256 ts = _mm256_sub_ps(_mm256_set1_ps(0.0), _mm256_div_ps(os, ns));

	ps_xs = _mm256_loadu_ps(m_ps[0].xs().data());
	ps_ys = _mm256_loadu_ps(m_ps[0].ys().data());
	ps_zs = _mm256_loadu_ps(m_ps[0].zs().data());

	__m256 offset_xs = _mm256_loadu_ps(m_offset.xs().data());
	os = _mm256_add_ps(offset_xs,
		dotx8(ps_xs, ps_ys, ps_zs, pos_xs, pos_ys, pos_zs));
	ns = dotx8(ps_xs, ps_ys, ps_zs, dir_xs, dir_ys, dir_zs);
	__m256 us = _mm256_fmadd_ps(ts, ns, os);

	ps_xs = _mm256_loadu_ps(m_ps[1].xs().data());
	ps_ys = _mm256_loadu_ps(m_ps[1].ys().data());
	ps_zs = _mm256_loadu_ps(m_ps[1].zs().data());

	__m256 offset_ys = _mm256_loadu_ps(m_offset.ys().data());
	os = _mm256_add_ps(offset_ys,
		dotx8(ps_xs, ps_ys, ps_zs, pos_xs, pos_ys, pos_zs));
	ns = dotx8(ps_xs, ps_ys, ps_zs, dir_xs, dir_ys, dir_zs);
	__m256 vs = _mm256_fmadd_ps(ts, ns, os);

	__m256 tnears = _mm256_set1_ps(ray.tnear);
	__m256 tfars = _mm256_set1_ps(ray.tfar);
	__m256 zeros = _mm256_set1_ps(0.0);
	__m256 ones = _mm256_set1_ps(1.0);

	constexpr int LESS_THAN = 2;

	__m256 mask = _mm256_cmp_ps(tnears, ts, LESS_THAN);
	mask = _mm256_and_ps(mask, _mm256_cmp_ps(ts, tfars, LESS_THAN));
	mask = _mm256_and_ps(mask, _mm256_cmp_ps(zeros, us, LESS_THAN));
	mask = _mm256_and_ps(mask, _mm256_cmp_ps(zeros, vs, LESS_THAN));
	mask = _mm256_and_ps(mask,
		_mm256_cmp_ps(_mm256_add_ps(us, vs), ones, LESS_THAN));

	const int packed_mask = _mm256_movemask_ps(mask);
	if (packed_mask == 0) {
		return InvalidIndex;
	}
	else {
		std::size_t index = InvalidIndex;
		std::array<float, 8> buf;
		_mm256_store_ps(buf.data(), ts);
		for (int i = 0; i < m_size; i++) {
			if (((packed_mask & (1 << i)) != 0) && buf[i] < ray.tfar) {
				index = i;
				ray.tfar = buf[i];
			}
		}
		if (index != InvalidIndex) {
			_mm256_store_ps(buf.data(), us);
			ray.u = buf[index];
			_mm256_store_ps(buf.data(), vs);
			ray.v = buf[index];
		}
		return index;
	}
}

using PackedTrianglex8 = PackedTriangle<8>;
using PackedTrianglex16 = PackedTriangle<16>;
