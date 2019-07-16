#pragma once

#include <mutex>
#include <optional>
#include <queue>

template <typename T> class TaskQueue {
public:
  TaskQueue(const std::size_t target_counter);
  void push(const T val) noexcept;
  std::optional<T> front_pop() noexcept;
  void update(const std::size_t task) noexcept;
  bool terminate() noexcept;

private:
  std::queue<T> m_que;
  std::mutex m_mutex;
  std::size_t m_counter;
  std::size_t m_target_counter;
};

template <typename T>
TaskQueue<T>::TaskQueue(const std::size_t target_counter)
    : m_counter(0), m_target_counter(target_counter) {}

template <typename T> void TaskQueue<T>::push(const T val) noexcept {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_que.push(val);
}

template <typename T> std::optional<T> TaskQueue<T>::front_pop() noexcept {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_que.empty()) {
    return std::nullopt;
  } else {
    auto ret = m_que.front();
    m_que.pop();
    return ret;
  }
}

template <typename T>
void TaskQueue<T>::update(const std::size_t counter) noexcept {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_counter += counter;
}

template <typename T> bool TaskQueue<T>::terminate() noexcept {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_counter == m_target_counter;
}