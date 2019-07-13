#pragma once

#include <algorithm>
#include <cstdint>

// =========================================
// declaration

class Vec3
{
public:
    Vec3() = default;
    Vec3(float x, float y, float z);
    Vec3(float xyz);
    Vec3(const float* ptr);
    Vec3(const Vec3& other);
    float x() const;
    float y() const;
    float z() const;
    float norm2() const;
    float norm() const;
    void normalize();
    Vec3 normalized() const;
    Vec3 operator+(Vec3 other) const;
    Vec3 operator-(Vec3 other) const;
    Vec3 operator*(Vec3 other) const;
    Vec3 operator/(Vec3 other) const;
    const float& operator[](std::int32_t index) const;

	static float dot(Vec3 rhs, Vec3 lhs);
    static Vec3 cross(Vec3 rhs, Vec3 lhs);
    static float distance(Vec3 rhs, Vec3 lhs);
    static Vec3 min(Vec3 lhs, Vec3 rhs);
    static Vec3 max(Vec3 lhs, Vec3 rhs);


private:
    float x_;
    float y_;
    float z_;
};


// ==========================================
// implementation

Vec3::Vec3(float x, float y, float z)
    : x_(x)
    , y_(y)
    , z_(z)
{
}

Vec3::Vec3(float xyz)
    : x_(xyz)
    , y_(xyz)
    , z_(xyz)
{
}

Vec3::Vec3(const float* ptr)
    : x_(ptr[0])
    , y_(ptr[1])
    , z_(ptr[2])
{
}

Vec3::Vec3(const Vec3& other)
    : x_(other.x_)
    , y_(other.y_)
    , z_(other.z_)
{
}

float Vec3::x() const { return x_; }
float Vec3::y() const { return y_; }
float Vec3::z() const { return z_; }

float Vec3::norm2() const
{
    return x_ * x_ + y_ * y_ + z_ * z_;
}

float Vec3::norm() const
{
    return std::sqrt(norm2());
}

void Vec3::normalize()
{
    const float il = 1.0f / norm();
    x_ *= il;
    y_ *= il;
    z_ *= il;
}

Vec3 Vec3::normalized() const
{
    Vec3 ret = *this;
    ret.normalize();
    return ret;
}

Vec3 Vec3::operator+(Vec3 other) const
{
    return Vec3(
        x_ + other.x_,
        y_ + other.y_,
        z_ + other.z_);
}

Vec3 Vec3::operator-(Vec3 other) const
{
    return Vec3(
        x_ - other.x_,
        y_ - other.y_,
        z_ - other.z_);
}

Vec3 Vec3::operator*(Vec3 other) const
{
    return Vec3(
        x_ * other.x_,
        y_ * other.y_,
        z_ * other.z_);
}

Vec3 Vec3::operator/(Vec3 other) const
{
    return Vec3(
        x_ / other.x_,
        y_ / other.y_,
        z_ / other.z_);
}

const float& Vec3::operator[](int32_t index) const
{
    return *(&x_ + index);
}

float Vec3::dot(Vec3 rhs, Vec3 lhs) {
    return rhs.x() * lhs.x() + rhs.y() * lhs.y() + rhs.z() * lhs.z();
}

Vec3 Vec3::cross(Vec3 rhs, Vec3 lhs) {
    return Vec3(
        rhs.y() * lhs.z() - rhs.z() * lhs.y(),
        rhs.z() * lhs.x() - rhs.x() * lhs.z(),
        rhs.x() * lhs.y() - rhs.y() * lhs.x());
}

float Vec3::distance(Vec3 rhs, Vec3 lhs) {
    return (rhs - lhs).norm();
}

Vec3 Vec3::min(Vec3 lhs, Vec3 rhs) {
    const float x = std::min(lhs.x(), rhs.x());
    const float y = std::min(lhs.y(), rhs.y());
    const float z = std::min(lhs.z(), rhs.z());
    return Vec3(x, y, z);
}

Vec3 Vec3::max(Vec3 lhs, Vec3 rhs) {
    const float x = std::max(lhs.x(), rhs.x());
    const float y = std::max(lhs.y(), rhs.y());
    const float z = std::max(lhs.z(), rhs.z());
    return Vec3(x, y, z);
}