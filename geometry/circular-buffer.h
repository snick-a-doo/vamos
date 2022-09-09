//  Copyright (C) 2008-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_GEOMETRY_CIRCULAR_BUFFER_H_INCLUDED
#define VAMOS_GEOMETRY_CIRCULAR_BUFFER_H_INCLUDED

#include <array>
#include <cassert>
#include <cstddef>

namespace Vamos_Geometry
{
/// A circular array-like container.
template <typename T, size_t N> class Circular_Buffer
{
public:
    /// @return The number of elements in the buffer.
    size_t size() const { return m_used; }
    /// @return True if nothing is in the buffer.
    bool empty() const { return m_used == 0; }
    /// Construct an element in the buffer. If the buffer was already full, the oldest
    /// element is lost.
    template <typename... Args> void emplace_back(Args&&... args);
    /// Return an element. The oldest element has index 0.
    const T& operator[](size_t index) const;
    /// Return the most recently added element.
    const T& back() const;

private:
    std::array<T, N> m_elements; ///< The underlying storage.
    size_t m_used{0}; ///< The number of filled slots.
    size_t m_start_index{0}; ///< The index of the next element.
};

template <typename T, size_t N> template <typename... Args>
void Circular_Buffer<T, N>::emplace_back(Args&&... args)
{
    m_elements[m_start_index] = std::move(T(args...));
    m_used = std::min(m_used + 1, N);
    m_start_index = (m_start_index + 1) % N;
}

template <typename T, size_t N>
const T& Circular_Buffer<T, N>::operator[](size_t index) const
{
    assert(index < m_used);
    return m_elements[(m_start_index + index) % m_used];
}

template <typename T, size_t N> const T& Circular_Buffer<T, N>::back() const
{
    return m_elements[(m_start_index == 0 ? N : m_start_index) - 1];
}

} // namespace Vamos_Geometry

#endif // VAMOS_GEOMETRY_CIRCULAR_BUFFER_H_INCLUDED
