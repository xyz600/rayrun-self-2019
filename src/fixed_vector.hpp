#pragma once

#include <cstddef>
#include <array>

template<typename T, std::size_t reserved>
class FixedVector
{
public:
	using iterator = typename std::array<T, reserved>::iterator;
	using const_iterator = typename std::array<T, reserved>::const_iterator;

	FixedVector() noexcept;

	iterator begin() noexcept;
	const_iterator begin() const noexcept;

	iterator end() noexcept;
	const_iterator end() const noexcept;

	std::size_t size() const noexcept;

	T &operator[](const std::size_t index) noexcept;
	T operator[](const std::size_t index) const noexcept;

	void push_back(T val) noexcept;

	void resize(std::size_t size) noexcept;

	T* data() noexcept;

	bool empty() const noexcept;

private:
	std::array<T, reserved> m_array;
	std::size_t m_size;
};

template<typename T, std::size_t reserved>
void FixedVector<T, reserved>::resize(std::size_t size) noexcept {
	m_size = size;
}

template<typename T, std::size_t reserved>
T* FixedVector<T, reserved>::data() noexcept {
	return m_array.data();
}

template<typename T, std::size_t reserved>
FixedVector<T, reserved>::FixedVector() noexcept {
	m_size = 0;
}

template<typename T, std::size_t reserved>
typename FixedVector<T, reserved>::iterator FixedVector<T, reserved>::begin() noexcept {
	return m_array.begin();
}

template<typename T, std::size_t reserved>
typename FixedVector<T, reserved>::const_iterator FixedVector<T, reserved>::begin() const noexcept {
	return m_array.begin();
}

template<typename T, std::size_t reserved>
typename FixedVector<T, reserved>::iterator FixedVector<T, reserved>::end() noexcept {
	return m_array.begin() + m_size;
}

template<typename T, std::size_t reserved>
typename FixedVector<T, reserved>::const_iterator FixedVector<T, reserved>::end() const noexcept {
	return m_array.begin() + m_size;
}

template<typename T, std::size_t reserved>
bool FixedVector<T, reserved>::empty() const noexcept {
	return m_size == 0;
}

template<typename T, std::size_t reserved>
void FixedVector<T, reserved>::push_back(T val) noexcept {
	m_array[m_size++] = val;
}

template<typename T, std::size_t reserved>
std::size_t FixedVector<T, reserved>::size() const noexcept {
	return m_size;
}

template<typename T, std::size_t reserved>
T &FixedVector<T, reserved>::operator[](const std::size_t index) noexcept {
	return m_array[index];
}

template<typename T, std::size_t reserved>
T FixedVector<T, reserved>::operator[](const std::size_t index) const noexcept {
	return m_array[index];
}
