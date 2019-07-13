#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>
#include <type_traits>

namespace vector {

template <typename T> struct binary_max {
  T operator()(const T &x, const T &y) const { return std::max<T>(x, y); }
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = T;
};

template <typename T> struct binary_min {
  T operator()(const T &x, const T &y) const { return std::min<T>(x, y); }
  using first_argument_type = T;
  using second_argument_type = T;
  using result_type = T;
};

/**
 * Point op Point
 */
template <typename LeftPointType, typename BinaryOperatorType,
          typename RightPointType>
class ExpressionBinaryPoint {
public:
  using value_type = typename LeftPointType::value_type;

  ExpressionBinaryPoint(const LeftPointType &left,
                        const RightPointType &right) noexcept
      : left_(left), right_(right) {}

  const value_type operator[](std::size_t index) const noexcept {
    return BinaryOperatorType()(left_[index], right_[index]);
  }

  constexpr std::size_t size() const noexcept { return left_.size(); }

private:
  const LeftPointType &left_;
  const RightPointType &right_;
};

template <typename LeftPointType, typename RightPointType>
using ExpressionAddPoint =
    ExpressionBinaryPoint<LeftPointType,
                          std::plus<typename LeftPointType::value_type>,
                          RightPointType>;

template <typename LeftPointType, typename RightPointType>
using ExpressionMinusPoint =
    ExpressionBinaryPoint<LeftPointType,
                          std::minus<typename LeftPointType::value_type>,
                          RightPointType>;

template <typename LeftPointType, typename RightPointType>
using ExpressionMultipliesPoint =
    ExpressionBinaryPoint<LeftPointType,
                          std::multiplies<typename LeftPointType::value_type>,
                          RightPointType>;

template <typename LeftPointType, typename RightPointType>
using ExpressionDividesPoint =
    ExpressionBinaryPoint<LeftPointType,
                          std::divides<typename LeftPointType::value_type>,
                          RightPointType>;

template <typename LeftPointType, typename RightPointType>
using ExpressionMaxPoint =
    ExpressionBinaryPoint<LeftPointType,
                          binary_max<typename LeftPointType::value_type>,
                          RightPointType>;

template <typename LeftPointType, typename RightPointType>
using ExpressionMinPoint =
    ExpressionBinaryPoint<LeftPointType,
                          binary_min<typename LeftPointType::value_type>,
                          RightPointType>;

/**
 * Point op value
 */
template <typename LeftPointType, typename BinaryOperatorType>
class ExpressionBinaryValue {
public:
  using value_type = typename LeftPointType::value_type;

  ExpressionBinaryValue(const LeftPointType &left, const value_type right)
      : left_(left), right_(right) {}

  const value_type operator[](std::size_t index) const noexcept {
    return BinaryOperatorType()(left_[index], right_);
  }

  constexpr std::size_t size() const noexcept { return left_.size(); }

private:
  const LeftPointType &left_;
  const value_type right_;
};

template <typename PointType>
using ExpressionAddValue =
    ExpressionBinaryValue<PointType, std::plus<typename PointType::value_type>>;

template <typename PointType>
using ExpressionMinusValue =
    ExpressionBinaryValue<PointType,
                          std::minus<typename PointType::value_type>>;

template <typename PointType>
using ExpressionMultipliesValue =
    ExpressionBinaryValue<PointType,
                          std::multiplies<typename PointType::value_type>>;

template <typename PointType>
using ExpressionDividesValue =
    ExpressionBinaryValue<PointType,
                          std::divides<typename PointType::value_type>>;

/**
 * op Point
 */

template <typename PointType, typename UnaryOperatorType>
class ExpressionUnary {
public:
  using value_type = typename PointType::value_type;

  ExpressionUnary(const PointType &point) : point_(point) {}

  const value_type operator[](std::size_t index) const noexcept {
    return UnaryOperatorType()(point_[index]);
  }

  constexpr std::size_t size() const noexcept { return point_.size(); }

private:
  const PointType &point_;
};

template <typename PointType>
using ExpressionNegate =
    ExpressionUnary<PointType, std::negate<typename PointType::value_type>>;

/**
 * Point
 */
template <typename T, std::size_t D> class Point {
public:
  using value_type = T;

  Point() noexcept;

  Point(std::initializer_list<T> init) noexcept;

  template <typename PointType> Point(const PointType &src);

  template <typename PointType> Point &operator=(const PointType &src) noexcept;

  template <typename PointType>
  Point &operator+=(const PointType &src) noexcept;

  template <typename PointType>
  Point &operator-=(const PointType &src) noexcept;

  Point &operator/=(const T &val) noexcept;
  Point &operator*=(const T &val) noexcept;

  T &operator[](std::size_t index) noexcept;

  const T &operator[](std::size_t index) const noexcept;

  const T &x() const noexcept;
  T &x() noexcept;

  const T &y() const noexcept;
  T &y() noexcept;

  const T &z() const noexcept;
  T &z() noexcept;

  constexpr std::size_t size() const noexcept;

private:
  std::array<T, D> data_;
};

template <typename T, std::size_t D>
std::ostream &operator<<(std::ostream &out, const Point<T, D> &point) {
  out << "(";
  for (std::size_t i = 0; i < point.size(); i++) {
    out << point[i] << " ";
  }
  out << ")";
  return out;
}

template <typename T, std::size_t D> Point<T, D>::Point() noexcept {
  std::fill(data_.begin(), data_.end(), 0);
}

template <typename T, std::size_t D>
Point<T, D>::Point(std::initializer_list<T> init) noexcept {
  assert(init.size() <= D);
  std::size_t index = 0;
  for (auto it = init.begin(); it != init.end(); it++) {
    data_[index] = *it;
    index++;
  }
}

template <typename T, std::size_t D>
template <typename PointType>
Point<T, D>::Point(const PointType &src) {
  for (std::size_t i = 0; i < size(); i++) {
    data_[i] = src[i];
  }
}

template <typename T, std::size_t D>
constexpr std::size_t Point<T, D>::size() const noexcept {
  return D;
}

template <typename T, std::size_t D>
T &Point<T, D>::operator[](std::size_t index) noexcept {
  return data_[index];
}

template <typename T, std::size_t D>
const T &Point<T, D>::operator[](std::size_t index) const noexcept {
  return data_[index];
}

template <typename T, std::size_t D> const T &Point<T, D>::x() const noexcept {
  return data_[0];
}

template <typename T, std::size_t D> T &Point<T, D>::x() noexcept {
  return data_[0];
}

template <typename T, std::size_t D> const T &Point<T, D>::y() const noexcept {
  return data_[1];
}

template <typename T, std::size_t D> T &Point<T, D>::y() noexcept {
  return data_[1];
}

template <typename T, std::size_t D> const T &Point<T, D>::z() const noexcept {
  return data_[2];
}

template <typename T, std::size_t D> T &Point<T, D>::z() noexcept {
  return data_[2];
}

template <typename T, std::size_t D>
template <typename PointType>
Point<T, D> &Point<T, D>::operator=(const PointType &src) noexcept {
  for (std::size_t i = 0; i < size(); i++) {
    this->data_[i] = src[i];
  }
  return *this;
}

template <typename T, std::size_t D>
template <typename PointType>
Point<T, D> &Point<T, D>::operator+=(const PointType &src) noexcept {
  for (std::size_t i = 0; i < size(); i++) {
    this->data_[i] += src[i];
  }
  return *this;
}

template <typename T, std::size_t D>
template <typename PointType>
Point<T, D> &Point<T, D>::operator-=(const PointType &src) noexcept {
  for (std::size_t i = 0; i < size(); i++) {
    this->data_[i] -= src[i];
  }
  return *this;
}

template <typename T, std::size_t D>
Point<T, D> &Point<T, D>::operator/=(const T &val) noexcept {
  for (std::size_t i = 0; i < size(); i++) {
    this->data_[i] /= val;
  }
  return *this;
}

template <typename T, std::size_t D>
Point<T, D> &Point<T, D>::operator*=(const T &val) noexcept {
  for (std::size_t i = 0; i < size(); i++) {
    this->data_[i] *= val;
  }
  return *this;
}

/**
 * operator
 */
template <typename PointType>
auto operator+(const PointType &point,
               const typename PointType::value_type value) {
  return ExpressionAddValue<PointType>(point, value);
}

template <typename LeftPointType, typename RightPointType>
std::enable_if_t<std::is_same<typename LeftPointType::value_type,
                              typename RightPointType::value_type>::value,
                 ExpressionAddPoint<LeftPointType, RightPointType>>
operator+(const LeftPointType &left, const RightPointType &right) {
  return ExpressionAddPoint<LeftPointType, RightPointType>(left, right);
}

template <typename PointType>
auto operator-(const PointType &point,
               const typename PointType::value_type value) {
  return ExpressionMinusValue<PointType>(point, value);
}

template <typename LeftPointType, typename RightPointType>
std::enable_if_t<std::is_same<typename LeftPointType::value_type,
                              typename RightPointType::value_type>::value,
                 ExpressionMinusPoint<LeftPointType, RightPointType>>
operator-(const LeftPointType &left, const RightPointType &right) {
  return ExpressionMinusPoint<LeftPointType, RightPointType>(left, right);
}

template <typename PointType>
auto operator*(const PointType &point,
               const typename PointType::value_type value) {
  return ExpressionMultipliesValue<PointType>(point, value);
}

template <typename LeftPointType, typename RightPointType>
std::enable_if_t<std::is_same<typename LeftPointType::value_type,
                              typename RightPointType::value_type>::value,
                 ExpressionMultipliesPoint<LeftPointType, RightPointType>>
operator*(const LeftPointType &left, const RightPointType &right) {
  return ExpressionMultipliesPoint<LeftPointType, RightPointType>(left, right);
}

template <typename PointType>
ExpressionDividesValue<PointType>
operator/(const PointType &point, const typename PointType::value_type value) {
  return ExpressionDividesValue<PointType>(point, value);
}

template <typename LeftPointType, typename RightPointType>
std::enable_if_t<std::is_same<typename LeftPointType::value_type,
                              typename RightPointType::value_type>::value,
                 ExpressionDividesPoint<LeftPointType, RightPointType>>
operator/(const LeftPointType &left, const RightPointType &right) {
  return ExpressionDividesPoint<LeftPointType, RightPointType>(left, right);
}

template <typename LeftPointType, typename RightPointType>
std::enable_if_t<std::is_same<typename LeftPointType::value_type,
                              typename RightPointType::value_type>::value,
                 ExpressionMinPoint<LeftPointType, RightPointType>>
min(const LeftPointType &left, const RightPointType &right) {
  return ExpressionMinPoint<LeftPointType, RightPointType>(left, right);
}

template <typename LeftPointType, typename RightPointType>
std::enable_if_t<std::is_same<typename LeftPointType::value_type,
                              typename RightPointType::value_type>::value,
                 ExpressionMaxPoint<LeftPointType, RightPointType>>
max(const LeftPointType &left, const RightPointType &right) {
  return ExpressionMaxPoint<LeftPointType, RightPointType>(left, right);
}

template <typename PointType>
ExpressionNegate<PointType> operator-(const PointType &src) {
  return ExpressionNegate<PointType>(src);
}

using Point3D = Point<double, 3>;
using Point3F = Point<float, 3>;

/**
 * utility function
 */

template <typename LeftPointType, typename RightPointType>
std::enable_if_t<std::is_same<typename LeftPointType::value_type,
                              typename RightPointType::value_type>::value,
                 typename LeftPointType::value_type>
dot(const LeftPointType &p1, const RightPointType &p2) {
  typename LeftPointType::value_type ret = 0;
  for (std::size_t i = 0; i < p1.size(); i++) {
    ret += p1[i] * p2[i];
  }
  return ret;
}

template <typename LeftPointType, typename RightPointType>
std::enable_if_t<std::is_same<typename LeftPointType::value_type,
                              typename RightPointType::value_type>::value,
                 typename LeftPointType::value_type>
dot(LeftPointType &&p1, RightPointType &&p2) {
  typename LeftPointType::value_type ret = 0;
  for (std::size_t i = 0; i < p1.size(); i++) {
    ret += p1[i] * p2[i];
  }
  return ret;
}

template <typename T>
Point<T, 3> cross(const Point<T, 3> &p1, const Point<T, 3> &p2) {
  Point<T, 3> ret{p1.y() * p2.z() - p2.y() * p1.z(),
                  p1.z() * p2.x() - p2.z() * p1.x(),
                  p1.x() * p2.y() - p2.x() * p1.y()};
  return ret;
}

template <typename PointType>
typename PointType::value_type norm2(const PointType &p) {
  typename PointType::value_type ret = 0;
  for (std::size_t i = 0; i < p.size(); i++) {
    const auto v = p[i];
    ret += v * v;
  }
  return ret;
}

template <typename PointType>
typename PointType::value_type norm2(PointType &&p) {
  typename PointType::value_type ret = 0;
  for (std::size_t i = 0; i < p.size(); i++) {
    const auto v = p[i];
    ret += v * v;
  }
  return ret;
}

template <typename PointType>
typename PointType::value_type distance(PointType &&p1, PointType &&p2) {
  return norm2(p1 - p2);
}

template <typename PointType>
typename PointType::value_type distance(const PointType &p1,
                                        const PointType &p2) {
  return norm2(p1 - p2);
}

template <typename PointType>
ExpressionDividesValue<PointType> normalize(const PointType &p) {
  const typename PointType::value_type norm = sqrt(norm2(p));
  return p / norm;
}

template <typename PointType>
ExpressionDividesValue<PointType> normalize(PointType &&p) {
  const typename PointType::value_type norm = sqrt(norm2(p));
  return p / norm;
}

using Vec3 = Point3F;

} // namespace vector