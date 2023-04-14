#ifndef __VECTOR_MATH_H__
#define __VECTOR_MATH_H__

#include "vector2.h"
#include "vector3.h"
#include "helper.h"
#include <cassert>
#include <cmath>

DType constexpr M_PI_VALUE{ 3.14159265358979323846f };
DType constexpr M_2PI_VALUE{ 6.28318530717958647692f };
DType constexpr M_1_PI_VALUE{ 0.318309886183790671538f };

DType constexpr TO_RAD_MULT{ 57.295779513f };
DType constexpr TO_DEG_MULT{ 0.01745329252f };

inline void readAsVector2(const std::vector<std::string>& tokens, Vector2& vec)
{
	vec._x = readAsfloat(tokens[1]);
	vec._y = readAsfloat(tokens[2]);
}

inline void readAsVector3(const std::vector<std::string>& tokens, Vector3& vec)
{
	vec._x = readAsfloat(tokens[1]);
	vec._y = readAsfloat(tokens[2]);
	if (tokens.size() > 3)
	{
		vec._z = readAsfloat(tokens[3]);
	}
}

// a.b = |a| |b| cos(angle)
inline float dotProduct(const Vector2& a, const Vector2& b)
{
	return a._x * b._x + a._y * b._y;
}

inline float dotProduct(const Vector3& a, const Vector3& b)
{
	return a._x * b._x + a._y * b._y + a._z * b._z;
}

inline float determinent(const Vector2& a, const Vector2& b)
{
	return { a._x * b._y - a._y * b._x };
}

// for 2D vector it's just determinent
inline float crossProduct(const Vector2& a, const Vector2& b)
{
	return determinent(a, b);
}

// a x b = |a| |b| sin(angle) n ,  n is a unit vector perpendicular to both a and b which completes a right-handed system
// a × b can be interpreted as the area of the parallelogram having aand b as sides
inline Vector3 crossProduct(const Vector3& a, const Vector3& b)
{
	return Vector3{ a._y * b._z - a._z * b._y,
					-(a._x * b._z - a._z * b._x),
					a._x * b._y - a._y * b._x } ;
}

inline float magnitude(const Vector2& a)
{
	return std::sqrtf(std::powf(a._x, 2) + std::powf(a._y, 2));
}

inline float magnitude(const Vector3& a)
{
	return std::sqrtf(std::powf(a._x, 2) + std::powf(a._y, 2) + std::powf(a._z, 2));
}

// a.b = |a| |b| cos(angle)
// angle = arccos ((a.b)/(|a| |b|))
template <typename T>
inline float angle(const T& a, const T& b)
{
	return std::acosf(dotProduct(a, b) / (magnitude(a) * magnitude(b)));
}

template <typename T>
inline T unitVec(const T& a)
{
	float mag = magnitude(a);
	if (mag == 0)
		return T{};
	return T{ a / mag };
}

template<typename T>
inline T projection(const T& a, const T& b)
{
	T unit = unitVec(b);
	return T{ unit * dotProduct(a, unit) };
}

// i.e orthagonal
inline bool atRightAngle(const Vector2& a, const Vector2& b)
{
	return dotProduct(a, b) == 0.0f;
}

// A and B vector form the line segment vector, p1 and p2 are vectors
enum class SIDE
{
	COLINEAR,
	ON_LEFT,
	ON_RIGHT,
};

inline SIDE getSide(const Vector3& A, const Vector3& B, const Vector3& p)
{
	Vector3 lineSegment{ B - A };
	Vector3 cp = crossProduct(lineSegment, p - A);

	SIDE ret = SIDE::ON_RIGHT;
	float mag = magnitude(cp);

	if (mag == 0)
	{
		ret = SIDE::COLINEAR;
	}
	else if (mag > 0)
	{
		ret = SIDE::ON_LEFT;
	}
	return ret;
}

inline SIDE getSide(const Vector2& A, const Vector2& B, const Vector2& p)
{
	Vector2 lineAB{ B - A };
	Vector2 lineAP{ p - A };

	float det = determinent(lineAB, lineAP);

	SIDE ret = SIDE::ON_RIGHT;

	if (det == 0)
	{
		ret = SIDE::COLINEAR;
	}
	else if (det > 0)
	{
		ret = SIDE::ON_LEFT;
	}
	return ret;
}

template<typename T>
inline bool areOnSameSide(const T& A, const T& B, const T& p1, const T& p2)
{
	SIDE sideP1 = getSide(A, B, p1);
	SIDE sideP2 = getSide(A, B, p2);

	return sideP1 == sideP2;
}

// from https://blackpawn.com/texts/pointinpoly/
inline bool sameSide(const Vector3& A, const Vector3& B, const Vector3& p1, const Vector3& p2)
{
	Vector3 cp1 = crossProduct(Vector3{ B - A }, Vector3{ p1 - A } );
	Vector3 cp2 = crossProduct(Vector3{ B - A }, Vector3{ p2 - A });
	
	return dotProduct(cp1, cp2) >= 0.0f;
}

inline bool sameSide(const Vector2& A, const Vector2& B, const Vector2& p1, const Vector2& p2)
{
	float cp1 = crossProduct(Vector2{ B - A }, Vector2{ p1 - A });
	float cp2 = crossProduct(Vector2{ B - A }, Vector2{ p2 - A });

	return (cp1 * cp2) >= 0.0f;
}

// from https://blackpawn.com/texts/pointinpoly/
//For a point to be inside the traingle A B C [clock wise] it must be below ABand left of BCand right of AC.If any one of these tests fails we can return early.
template<typename T>
inline bool pointInsideTriangle(const T& A, const T& B, const T& C, const T& p)
{
	return sameSide(p, A, B, C) && sameSide(p, B, A, C) && sameSide(p, C, A, B);
}

inline Vector3 getNormalToTriangle(const Vector3& a, const Vector3& b, const Vector3& c)
{
	Vector3 v1{ b - a };
	Vector3 v2{ c - a };

	return crossProduct(v1, v2);
}

// cos (A+B) = cosA cosB - sinA sinB
// sin (A+B) = sinA cosB + cosA sinB
// 3D Translation :
//		y
//		|
//		|_____ x
//		/
//	  z/
// 
//		P' = P.T
//								[1		0		0	0]
//	[X' Y' Z'] =[X	Y	Z	1]	[0		1		0	0] => [X + tx		Y + ty		Z + tz	1]
//								[0		0	    1	0]
//								[tx		ty	   tz	1]
// 
// 3D Scaling :
// 		P' = P.S
//							[Sx		0		0	0]
//	[X' Y' Z'] =[X	Y	Z]	[0		Sy		0	0] => [X.Sx		Y.Sy	Z.Sz	1]
//							[0		0		Sz	0]
//							[0		0		0	1]
// 
// 3D rotation : 
//		Note : Rotation around different axis is NOT commutative, order do matter
//		Rotation around Z axis = 2D rotation around XY plane, with X = "X" and Y = "Y" -> slope of Y over X (Mxy) -> AngleZ = arctan(Mxy)
//		Rotation around X axis = 2D rotation around YZ plane, with Y = "X" and Z = "Y" -> slope of Z over Y (Myz) -> AngleX = arctan(Myz)
//		Rotation around Y axis = 2D rotation around ZX plane, with Z = "X" and X = "Y" -> slope of X over Z (Mzx) -> AngleY = arctan(Mzx)
//		[1		0		0		0]					[cos		0		sin		0]				[cos	-sin	0	0]
// Rx = [0		cos		-sin	0]			Ry =	[	0		1		0		0]		Rz = 	[sin	cos		0	0]
//		[0		sin		cos		0]					[-sin		0		cos		0]				[0		0		1	0]
//		[0		0		0		1]					[0			0		0		1]				[0		0		0	1]
//
//
//	3D Reflection : Against a reflection axis or reflection plane.
//					Reflections relative to a given axis is 180 rotation about that axis
//			[-1		0		0]					[0		0		0]				[1		0		0 ]
// RFx =	[0		1		0]			RFy =	[0		-1		0]		RFz = 	[0		1		0 ]
//			[0		0		1]					[0		0		1]				[0		0		-1]
//
//	3D Shear : Tilting
//			[1		a		b	0]					[1		0		0	0]				[1		0		0	0]
// SHx =	[0		1		0	0]			SHy =	[a		1		b	0]		SHz = 	[0		1		0	0]
//			[0		0		1	0]					[0		0		1	0]				[a		b		1	0]
//			[0		0		0	1]					[0		0		0	1]				[0		0		0	1]

struct Matrix2
{
	float	_mat[2][2]{};

	const float* operator[](size_t index) const
	{
		return _mat[index];
	}

	float* operator[](size_t index)
	{
		return _mat[index];
	}

	static Matrix2 getIdentityMatrix()
	{
		Matrix2 mat{};
		mat[0][0] = mat[1][1] =  1.0f;
		return mat;
	}
};

struct Matrix3
{
	float	_mat[3][3]{};

	const float* operator[](size_t index) const
	{
		return _mat[index];
	}

	float* operator[](size_t index)
	{
		return _mat[index];
	}

	static Matrix3 getIdentityMatrix()
	{
		Matrix3 mat{};
		mat[0][0] = mat[1][1] = mat[2][2] = 1.0f;
		return mat;
	}
};

struct Matrix4
{
	float	_mat[4][4]{};

	const float* operator[](size_t index) const
	{
		return _mat[index];
	}

	float* operator[](size_t index)
	{
		return _mat[index];
	}

	static Matrix4 getIdentityMatrix()
	{
		Matrix4 mat{};
		mat[0][0] = mat[1][1] = mat[2][2] = mat[3][3] = 1.0f;
		return mat;
	}
};

//For matrix multiplication, the number of columns in the first matrix must be equal to the number of rows in the second matrix.
//The result matrix has the number of rows of the first and the number of columns of the second matrix.
// (m x n) X (n x p) = n x p
inline Vector2 operator*(const Vector2& vec, const Matrix2& mat)
{
	//
	// [x	y] * [a  b]  = []
	//			 [c  d]
	Vector2 res{};
	res[0] = vec[0] * mat[0][0] + vec[1] * mat[1][0];
	res[1] = vec[0] * mat[0][1] + vec[1] * mat[1][1];
	return res;
}

inline Matrix2 operator*(const Matrix2& lhs, const Matrix2& rhs)
{
	Matrix2 res{};
	for (int r = 0; r < 2; ++r)
	{
		for (int c = 0; c < 2; ++c)
		{
			for (int k = 0; k < 2; ++k)
			{
				res[r][c] += lhs[r][k] * rhs[k][c];
			}
		}
	}
	return res;
}

inline Vector3 operator*(const Vector3& vec, const Matrix3& mat)
{
	Vector3 res{};

	for (int i = 0; i < 3; ++i)
	{
		res[i] = vec[0] * mat[0][i] + vec[1] * mat[1][i] + vec[2] * mat[2][i];
	}
	
	return res;
}

inline Matrix3 operator*(const Matrix3& lhs, const Matrix3& rhs)
{
	Matrix3 res{};
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			for (int k = 0; k < 3; ++k)
			{
				res[r][c] += lhs[r][k] * rhs[k][c];
			}
		}
	}
	return res;
}

inline Vector3 operator*(const Vector3& vec, const Matrix4& mat)
{
	Vector3 res{};

	for (int i = 0; i < 3; ++i)
	{
		res[i] = vec[0] * mat[0][i] + vec[1] * mat[1][i] + vec[2] * mat[2][i] + mat[3][i];
	}

	return res;
}

inline Matrix4 operator*(const Matrix4& lhs, const Matrix4& rhs)
{
	Matrix4 res{};
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			for (int k = 0; k < 4; ++k)
			{
				res[r][c] += lhs[r][k] * rhs[k][c];
			}
		}
	}
	return res;
}

struct Quaternions
{
	float		_a{};
	Vector3		_v3{};

	Quaternions(float a, const Vector3 &vec)
		:_a(a)
		,_v3(vec)
	{}
	// Quaternions : Rule
	//						[ i.i = j.j = k.k = i.j.k = -1 ]
	//						[i.j = k, j.i = -k]
	//						[j.k = i, k.j = -i]
	//						[k.i = j, i.k = -j]
	// q1 = (a1, v1)
	// q2 = (a2, v2)
	// q1 + q2 = (a1 + a2, v1 + v2)
	// q1.q2 = (a1.a2 - dotProduct(v1,v2), a1.v2 + a2.v1 + crossProduct(v1, v2))

	Quaternions operator+(const Quaternions& rhs) const
	{
		return Quaternions{_a + rhs._a, _v3 + rhs._v3};
	}

	Quaternions operator-(const Quaternions& rhs) const
	{
		return Quaternions{ _a - rhs._a, _v3 - rhs._v3 };
	}

	Quaternions operator*(const Quaternions& rhs) const
	{
		return Quaternions{ _a * rhs._a - dotProduct(_v3, rhs._v3), (_a * rhs._v3) + (rhs._a * _v3) + crossProduct(_v3, rhs._v3) };
	}

	Quaternions operator*(float val)
	{
		return Quaternions{val * _a, val * _v3 };
	}

	Quaternions operator/(float val)
	{
		return Quaternions{ _a / val, _v3 / val };
	}
};

namespace nsTransform
{
	// Translate
	inline Vector2 translate(const Vector2& vec, float x, float y = 0.0f)
	{
		Matrix2 mat(Matrix2::getIdentityMatrix());
		mat[2][0] = x;
		mat[2][1] = y;

		return vec * mat;
	}

	inline Vector3 translate(const Vector3 &vec, float x, float y = 0.0f, float z = 0.0f)
	{
		Matrix4 mat(Matrix4::getIdentityMatrix());
		mat[3][0] = x;
		mat[3][1] = y;
		mat[3][2] = z;
		return vec * mat;
	}

	// Scale
	inline Vector2 scale(const Vector2& vec, float x, float y = 1.0f)
	{
		Matrix2 mat(Matrix2::getIdentityMatrix());
		mat[2][0] = x;
		mat[2][1] = y;
		return vec * mat;
	}

	inline Vector3 scale(const Vector3& vec, float x, float y = 1.0f, float z = 1.0f)
	{
		Matrix4 mat(Matrix4::getIdentityMatrix());
		mat[0][0] = x;
		mat[1][1] = y;
		mat[2][2] = z;

		return vec * mat;
	}
	
	// Rotate
	// Vector2
	inline Vector2 rotateRad(const Vector2& vec, float angle)
	{
		//		[cos	sin]
		//Rz = 	[-sin	cos]
		Matrix2 mat;
		mat[1][1] = mat[0][0] = std::cosf(angle);
		mat[1][0] = std::sinf(angle);
		mat[0][1] = -mat[0][1];

		return vec * mat;
	}

	// Rotate
	// Vector3
	inline Vector3 rotateAroundXRad(const Vector3& vec, float angle)
	{
		//		[1		0		0		0]
		//Rx =	[0		cos		-sin	0]
		//		[0		sin		cos		0]
		//		[0		0		0		1]
		Matrix4 mat;
		mat[0][0] = 1;

		mat[2][2] = mat[1][1] = std::cosf(angle);

		mat[2][1] = std::sinf(angle);
		mat[1][2] = -mat[2][1];
		return vec * mat;
	}

	inline Vector3 rotateAroundYRad(const Vector3& vec, float angle)
	{
		//		[cos	0	sin		0]
		//Ry =	[0		1	  0		0]
		//		[-sin	0	cos		0]
		//		[0		0	0		1]
		Matrix4 mat;
		mat[1][1] = 1;

		mat[2][2] = mat[0][0] = std::cosf(angle);

		mat[0][2] = std::sinf(angle);
		mat[2][0] = -mat[0][2];

		return vec * mat;
	}

	inline Vector3 rotateAroundZRad(const Vector3& vec, float angle)
	{
		//		[cos	-sin	0	0]
		//Rz = 	[sin	cos		0	0]
		//		[0		0		1	0]
		//		[0		0		0	1]
		Matrix4 mat;
		mat[2][2] = 1;

		mat[1][1] = mat[0][0] = std::cosf(angle);

		mat[1][0] = std::sinf(angle);
		mat[0][1] = -mat[1][0];

		return vec * mat;
	}

	// P is Point to rotate angle radian around unit vector V
	inline Vector3 rotate(const Vector3& P, const Vector3 &U, float angle)
	{
		// V = <v1, v2, v3>
		// P = <x, y, z>
		// H = a + b.i + c.j + d.k
		// H_T = a - b.i - c.j - d.k
		// where	a = cos (angle / 2)
		//			b = v1 sin (angle /2)
		//			c = v2 sin (angle /2)
		//			d = v3 sin (angle /2)
		// P => x.i + y.j + z.k
		// Rotation = H.p.H_T
		
		// Rotation 
		float cosVal = std::cosf(angle / 2);
		float sinVal = std::sinf(angle / 2);

		Quaternions H{ cosVal, sinVal * U };
		Quaternions H_T{ cosVal, -(H._v3) };

		Quaternions res{ H * Quaternions{0.0f, P} *H_T };

		return res._v3;
	}

	// Reflect
	// Vector2
	inline Vector2 reflect(const Vector2& vec)
	{
		return Vector2{ -vec._x, -vec._y };
	}

	inline Vector2 reflectAlongX(const Vector2& vec)
	{
		return Vector2{ -vec._x, vec._y };
	}

	inline Vector2 reflectAlongY(const Vector2& vec)
	{
		return Vector2{ vec._x, -vec._y };
	}

	// Reflect
	// Vector3
	inline Vector3 reflectAlongX(const Vector3& vec)
	{
		return Vector3{ -vec._x, vec._y, vec._z };
	}

	inline Vector3 reflectAlongY(const Vector3& vec)
	{
		return Vector3{ vec._x, -vec._y, vec._z };
	}

	inline Vector3 reflectAlongZ(const Vector3& vec)
	{
		return Vector3{ vec._x, vec._y, -vec._z };
	}

	// Shear/Tilting
	inline Vector3 shearAlongX(const Vector3& vec, float a, float b)
	{
		//			[1		a		b	0]
		// SHx =	[0		1		0	0]
		//			[0		0		1	0]
		//			[0		0		0	1]
		Matrix4 mat(Matrix4::getIdentityMatrix());
		mat[0][1] = a;
		mat[0][2] = b;
		return vec * mat;
	}

	inline Vector3 shearAlongY(const Vector3& vec, float a, float b)
	{
		//			[1		0		0	0]
		// SHy =	[a		1		b	0]
		//			[0		0		1	0]
		//			[0		0		0	1]
		Matrix4 mat(Matrix4::getIdentityMatrix());
		mat[1][0] = a;
		mat[1][2] = b;
		return vec * mat;
	}

	inline Vector3 shearAlongZ(const Vector3& vec, float a, float b)
	{
		//			[1		0		0	0]
		// SHz =	[0		1		0	0]
		//			[a		b		1	0]
		//			[0		0		0	1]
		Matrix4 mat(Matrix4::getIdentityMatrix());
		mat[2][0] = a;
		mat[2][1] = b;
		return vec * mat;
	}
}
#endif //#ifndef __VECTOR_MATH_H__
