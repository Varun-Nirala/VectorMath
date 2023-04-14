#ifndef __VECTOR_3_H__
#define __VECTOR_3_H__

#include "helper.h"
#include <type_traits>
#include <exception>

struct Vector3
{
	template <typename T, ENABLE_IF_INTEGER<T>>
	friend Vector3 operator+(const T& lhs, const Vector3& rhs);

	template <typename T, ENABLE_IF_FLOAT<T>>
	friend Vector3 operator+(const T& lhs, const Vector3& rhs);

	template <typename T, ENABLE_IF_INTEGER<T>>
	friend Vector3 operator-(const T& lhs, const Vector3& rhs);

	template <typename T, ENABLE_IF_FLOAT<T>>
	friend Vector3 operator-(const T& lhs, const Vector3& rhs);

	template <typename T, ENABLE_IF_INTEGER<T>>
	friend Vector3 operator*(const T& lhs, const Vector3& rhs);

	template <typename T, ENABLE_IF_FLOAT<T>>
	friend Vector3 operator*(const T& lhs, const Vector3& rhs);

	friend inline bool isEqual(const Vector3& lhs, const Vector3& rhs)
	{
		return (lhs._x == rhs._x && lhs._y == rhs._y && lhs._z == rhs._z);
	}

	friend inline bool isLess(const Vector3& lhs, const Vector3& rhs)
	{
		return (lhs._x < rhs._x && lhs._y < rhs._y && lhs._z < rhs._z);
	}

	friend inline bool areOpposite(const Vector3& lhs, const Vector3& rhs)
	{
		return (lhs._x == -rhs._x && lhs._y == -rhs._y && lhs._z == -rhs._z);
	}

	inline bool isZero() const { return _x == 0.0f && _y == 0.0f && _z == 0.0f; }
	inline bool isIdentity() const { return _x == 1.0f && _y == 1.0f && _z == 1.0f; }

	friend inline bool operator==(const Vector3& lhs, const Vector3& rhs) { return isEqual(lhs, rhs); }
	friend inline bool operator!=(const Vector3& lhs, const Vector3& rhs) { return !isEqual(lhs, rhs); }
	friend inline bool operator< (const Vector3& lhs, const Vector3& rhs) { return isLess(lhs, rhs); }
	friend inline bool operator> (const Vector3& lhs, const Vector3& rhs) { return isLess(rhs, lhs); }
	friend inline bool operator<=(const Vector3& lhs, const Vector3& rhs) { return !isLess(rhs, lhs); }
	friend inline bool operator>=(const Vector3& lhs, const Vector3& rhs) { return !isLess(lhs, rhs); }

	friend std::ostream& operator<<(std::ostream &out, const Vector3& vec)
	{
		printf("{%4f, %4f, %4f}", vec._x, vec._y, vec._z);
		return out;
	}

	explicit Vector3(float x = 0.0f, float y = 0.0f, float z = 0.0f)
		: _x(x)
		, _y(y)
		, _z(z)
	{}

	~Vector3() = default;
	Vector3(const Vector3& v) = default;
	Vector3(Vector3&& v) = default;
	Vector3& operator=(const Vector3& v) = default;
	Vector3& operator=(Vector3&& v) = default;

	void set(const Vector3& vec)
	{
		_x = vec._x;
		_y = vec._y;
		_z = vec._z;
	}

	void set(float x, float y, float z)
	{
		_x = x;
		_y = y;
		_z = z;
	}

	const float& operator[](size_t index) const
	{
		assert(index >=0 && index < 3 && "Invalid Index!");
		if (index == 0)
			return _x;
		return index == 1 ? _y : _z;
	}

	float& operator[](size_t index)
	{
		assert(index >= 0 && index < 3 && "Invalid Index!");
		if (index == 0)
			return _x;
		return index == 1 ? _y : _z;
	}

	template <typename T, ENABLE_IF_INTEGER<T> = true>
	Vector3 operator+(const T &val) const
	{
		return Vector3{ _x + val, _y + val, _z + val };
	}

	template <typename T, ENABLE_IF_FLOAT<T> = true>
	Vector3 operator+(const T& val) const
	{
		return Vector3{ _x + val, _y + val, _z + val };
	}

	Vector3 operator+(const Vector3& vec) const
	{
		return Vector3{ _x + vec._x, _y + vec._y, _z + vec._z };
	}

	template <typename T, ENABLE_IF_INTEGER<T> = true>
	Vector3 operator-(const T& val) const
	{
		return Vector3{ _x - val, _y - val, _z - val };
	}

	template <typename T, ENABLE_IF_FLOAT<T> = true>
	Vector3 operator-(const T& val) const
	{
		return Vector3{ _x - val, _y - val, _z - val };
	}

	Vector3 operator-(const Vector3& vec) const
	{
		return Vector3{ _x - vec._x, _y - vec._y, _z - vec._z };
	}

	template <typename T, ENABLE_IF_INTEGER<T> = true>
	Vector3 operator*(const T& val) const
	{
		return Vector3{ _x * val, _y * val, _z * val };
	}

	template <typename T, ENABLE_IF_FLOAT<T>>
	Vector3 operator*(const T& val) const
	{
		return Vector3{ _x * val, _y * val, _z * val };
	}

	template <typename T, ENABLE_IF_INTEGER<T> = true>
	Vector3 operator/(const T& val) const
	{
		if (val == 0)
		{
			throw std::runtime_error("Math error: Divide by Zero\n");
		}
		return Vector3{ _x / val, _y / val, _z / val };
	}

	template <typename T, ENABLE_IF_FLOAT<T> = true>
	Vector3 operator/(const T& val) const
	{
		if (val == 0.0f)
		{
			throw std::runtime_error("Math error: Divide by Zero\n");
		}
		return Vector3{ _x / val, _y / val, _z / val };
	}

	Vector3 operator-()
	{
		return Vector3{-_x, -_y, -_z};
	}

	float	_x{};
	float	_y{};
	float	_z{};
};

template <typename T, ENABLE_IF_INTEGER<T> = true>
Vector3 operator+(const T& lhs, const Vector3& rhs)
{
	return Vector3{ rhs._x + lhs, rhs._y + lhs,  rhs._z + lhs };
}

template <typename T, ENABLE_IF_FLOAT<T> = true>
Vector3 operator+(const T& lhs, const Vector3& rhs)
{
	return Vector3{ rhs._x + lhs, rhs._y + lhs,  rhs._z + lhs };
}

template <typename T, ENABLE_IF_INTEGER<T> = true>
Vector3 operator-(const T& lhs, const Vector3& rhs)
{
	return Vector3{ rhs._x - lhs, rhs._y - lhs, rhs._z - lhs };
}

template <typename T, ENABLE_IF_FLOAT<T> = true>
Vector3 operator-(const T& lhs, const Vector3& rhs)
{
	return Vector3{ rhs._x - lhs, rhs._y - lhs, rhs._z - lhs };
}

template <typename T, ENABLE_IF_INTEGER<T> = true>
Vector3 operator*(const T& lhs, const Vector3& rhs)
{
	return Vector3{ rhs._x * lhs, rhs._y * lhs,  rhs._z * lhs };
}

template <typename T, ENABLE_IF_FLOAT<T> = true>
Vector3 operator*(const T& lhs, const Vector3& rhs)
{
	return Vector3{ rhs._x * lhs, rhs._y * lhs,  rhs._z * lhs };
}
#endif // #ifndef __VECTOR_3_H__
