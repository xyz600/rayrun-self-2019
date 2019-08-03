#pragma once

#include "vec.hpp"

#include <array>
#include <cassert>
#include <cstddef>
#include <iomanip>
#include <iostream>

template <typename T> class Matrix4x4 {
public:
	using row_t = vector::Point<T, 4>;

	Matrix4x4();
	Matrix4x4(const Matrix4x4<T> &src);

	const typename Matrix4x4<T>::row_t &operator[](const std::size_t index) const
		noexcept;
	typename Matrix4x4<T>::row_t &operator[](const std::size_t index) noexcept;

	const typename Matrix4x4<T>::row_t &row(const std::size_t index) const
		noexcept;
	typename Matrix4x4<T>::row_t &row(const std::size_t index) noexcept;

	Matrix4x4 inverse() const noexcept;

	Matrix4x4 operator*(const Matrix4x4 &mat) const noexcept;

	void eye() noexcept;

	constexpr std::size_t size() const noexcept;

private:
	std::array<row_t, 4> m_data;
};

template <typename T>
constexpr std::size_t Matrix4x4<T>::size() const noexcept {
	return 4;
}

template <typename T> Matrix4x4<T>::Matrix4x4() {}

template <typename T> Matrix4x4<T>::Matrix4x4(const Matrix4x4<T> &src) {
	for (std::size_t index = 0; index < size(); index++) {
		m_data[index] = src.m_data[index];
	}
}

template <typename T>
typename Matrix4x4<T>::row_t &
Matrix4x4<T>::row(const std::size_t index) noexcept {
	return m_data[index];
}

template <typename T>
const typename Matrix4x4<T>::row_t &
Matrix4x4<T>::row(const std::size_t index) const noexcept {
	return m_data[index];
}

template <typename T>
typename Matrix4x4<T>::row_t &Matrix4x4<T>::
operator[](const std::size_t index) noexcept {
	return m_data[index];
}

template <typename T>
const typename Matrix4x4<T>::row_t &Matrix4x4<T>::
operator[](const std::size_t index) const noexcept {
	return m_data[index];
}

template <typename T> void Matrix4x4<T>::eye() noexcept {
	for (auto &r : m_data) {
		r.fill(0.0f);
	}
	for (std::size_t i = 0; i < size(); i++) {
		(*this)[i][i] = 1.0f;
	}
}

template <typename T> Matrix4x4<T> Matrix4x4<T>::inverse() const noexcept {
	Matrix4x4 self(*this);
	Matrix4x4 I;
	I.eye();

	for (std::size_t i = 0; i < size(); i++) {
		std::size_t r = i;
		while (r < size() && self[r][i] == 0.0f) {
			r++;
		}
		assert(r < 4);
		// 必要に応じて swap
		if (r != i) {
			std::swap(self[i], self[r]);
			std::swap(I[i], I[r]);
		}

		const float coef = self[i][i];
		I[i] /= coef;
		self[i] /= coef;
		// 一つ下からの行に対して、 col 番目の要素を消す
		for (std::size_t j = 0; j < size(); j++) {
			if (i != j) {
				const float coef = self[j][i];
				I[j] -= I[i] * coef;
				self[j] -= self[i] * coef;
			}
		}
	}
	return I;
}

template <typename T>
std::ostream &operator<<(std::ostream &out, const Matrix4x4<T> &mat) {
	out << "[";
	for (std::size_t i = 0; i < mat.size(); i++) {
		if (0 < i) {
			out << " ";
		}
		out << "[";
		for (std::size_t j = 0; j < mat.size(); j++) {
			if (0 < j) {
				out << ", ";
			}
			out << std::setw(8) << mat[i][j];
		}
		out << "]," << std::endl;
	}
	out << "]";
	return out;
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::operator*(const Matrix4x4 &mat) const noexcept {
	Matrix4x4<T> ret;
	for (auto &row : ret.m_data) {
		row.fill(0.0);
	}
	const auto &self = *this;

	for (std::size_t i = 0; i < size(); i++) {
		for (std::size_t j = 0; j < size(); j++) {
			for (std::size_t k = 0; k < size(); k++) {
				ret[i][j] += self[i][k] * mat[k][j];
			}
		}
	}
	return ret;
}

using Matrix4x4F = Matrix4x4<float>;
