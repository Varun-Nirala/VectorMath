#ifndef __VECTOR_2_H__
#define __VECTOR_2_H__

#include <cstdlib>
#include <cmath>
#include <type_traits>
#include <exception>
#include "helper.h"

struct Vector2
{
	template <typename T, ENABLE_IF_INTEGER<T>>
	friend Vector2 operator+(const T& lhs, const Vector2& rhs);

	template <typename T, ENABLE_IF_FLOAT<T>>
	friend Vector2 operator+(const T& lhs, const Vector2& rhs);

	template <typename T, ENABLE_IF_INTEGER<T>>
	friend Vector2 operator-(const T& lhs, const Vector2& rhs);

	template <typename T, ENABLE_IF_FLOAT<T>>
	friend Vector2 operator-(const T& lhs, const Vector2& rhs);

	template <typename T, ENABLE_IF_INTEGER<T>>
	friend Vector2 operator*(const T& lhs, const Vector2& rhs);

	template <typename T, ENABLE_IF_FLOAT<T>>
	friend Vector2 operator*(const T& lhs, const Vector2& rhs);

	friend inline bool isEqual(const Vector2& lhs, const Vector2& rhs)
	{
		return (lhs._x == rhs._x && lhs._y == rhs._y);
	}

	friend inline bool isLess(const Vector2& lhs, const Vector2& rhs)
	{
		return (lhs._x < rhs._x && lhs._y < rhs._y);
	}

	friend inline bool areOpposite(const Vector2& lhs, const Vector2& rhs)
	{
		return (lhs._x == -rhs._x && lhs._y == -rhs._y);
	}

	inline bool isZero() const { return _x == 0.0f && _y == 0.0f; }
	inline bool isIdentity() const { return _x == 1.0f && _y == 1.0f; }

	friend inline bool operator==(const Vector2& lhs, const Vector2& rhs) { return isEqual(lhs, rhs); }
	friend inline bool operator!=(const Vector2& lhs, const Vector2& rhs) { return !isEqual(lhs, rhs); }
	friend inline bool operator< (const Vector2& lhs, const Vector2& rhs) { return isLess(lhs, rhs); }
	friend inline bool operator> (const Vector2& lhs, const Vector2& rhs) { return isLess(rhs, lhs); }
	friend inline bool operator<=(const Vector2& lhs, const Vector2& rhs) { return !isLess(rhs, lhs); }
	friend inline bool operator>=(const Vector2& lhs, const Vector2& rhs) { return !isLess(lhs, rhs); }

	friend std::ostream& operator<<(std::ostream& out, const Vector2& vec)
	{
		printf("x = %4f , y = %4f", vec._x, vec._y);
		return out;
	}

	explicit Vector2(float x = 0.0f, float y = 0.0f)
		: _x(x)
		, _y(y)
	{}

	~Vector2() = default;
	Vector2(const Vector2 & v) = default;
	Vector2(Vector2 && v) = default;
	Vector2& operator=(const Vector2 & v) = default;
	Vector2& operator=(Vector2 && v) = default;

	void set(const Vector2& vec)
	{
		_x = vec._x;
		_y = vec._y;
	}

	void set(float x, float y)
	{
		_x = x;
		_y = y;
	}

	const float& operator[](size_t index) const
	{
		assert(index >= 0 && index < 2 && "Invalid Index!");
		return index == 0 ? _x : _y;
	}

	float& operator[](size_t index)
	{
		assert(index >= 0 && index < 2 && "Invalid Index!");
		return index == 0 ? _x : _y;
	}

	template <typename T, ENABLE_IF_INTEGER<T> = true>
	Vector2 operator+(const T &val) const
	{
		return Vector2{ _x + val, _y + val };
	}

	template <typename T, ENABLE_IF_FLOAT<T> = true>
	Vector2 operator+(const T& val) const
	{
		return Vector2{ _x + val, _y + val };
	}

	Vector2 operator+(const Vector2& vec) const
	{
		return Vector2{ _x + vec._x, _y + vec._y };
	}

	template <typename T, ENABLE_IF_INTEGER<T> = true>
	Vector2 operator-(const T& val) const
	{
		return Vector2{ _x - val, _y - val };
	}

	template <typename T, ENABLE_IF_FLOAT<T> = true>
	Vector2 operator-(const T& val) const
	{
		return Vector2{ _x - val, _y - val };
	}

	Vector2 operator-(const Vector2& vec) const
	{
		return Vector2{ _x - vec._x, _y - vec._y };
	}

	template <typename T, ENABLE_IF_INTEGER<T> = true>
	Vector2 operator*(const T& val) const
	{
		return Vector2{ _x * val, _y * val };
	}

	template <typename T, ENABLE_IF_FLOAT<T> = true>
	Vector2 operator*(const T& val) const
	{
		return Vector2{ _x * val, _y * val };
	}

	template <typename T, ENABLE_IF_INTEGER<T> = true>
	Vector2 operator/(const T& val) const
	{
		if (val == 0)
		{
			throw std::runtime_error("Math error: Divide by Zero\n");
		}
		return Vector2{ _x / val, _y / val };
	}

	template <typename T, ENABLE_IF_FLOAT<T>>
	Vector2 operator/(const T& val) const
	{
		if (val == 0.0f)
		{
			throw std::runtime_error("Math error: Divide by Zero\n");
		}
		return Vector2{ _x / val, _y / val };
	}

	Vector2 operator-()
	{
		return Vector2{-_x, -_y};
	}

	float	_x{};
	float	_y{};
};

template <typename T, ENABLE_IF_INTEGER<T> = true>
Vector2 operator+(const T& lhs, const Vector2& rhs)
{
	return Vector2{ rhs._x + lhs, rhs._y + lhs };
}

template <typename T, ENABLE_IF_FLOAT<T> = true>
Vector2 operator+(const T& lhs, const Vector2& rhs)
{
	return Vector2{ rhs._x + lhs, rhs._y + lhs };
}

template <typename T, ENABLE_IF_INTEGER<T> = true>
Vector2 operator-(const T& lhs, const Vector2& rhs)
{
	return Vector2{ rhs._x - lhs, rhs._y - lhs };
}

template <typename T, ENABLE_IF_FLOAT<T> = true>
Vector2 operator-(const T& lhs, const Vector2& rhs)
{
	return Vector2{ rhs._x - lhs, rhs._y - lhs };
}

template <typename T, ENABLE_IF_INTEGER<T> = true>
Vector2 operator*(const T& lhs, const Vector2& rhs)
{
	return Vector2{ rhs._x * lhs, rhs._y * lhs };
}

template <typename T, ENABLE_IF_FLOAT<T> = true>
Vector2 operator*(const T& lhs, const Vector2& rhs)
{
	return Vector2{ rhs._x * lhs, rhs._y * lhs };
}

#endif // #ifndef __VECTOR_2_H__