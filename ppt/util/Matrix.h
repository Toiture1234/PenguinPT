// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <nanovdb/io/IO.h>

#include <ppt/util/options.h>

#include <stdio.h>
#include <iostream>
#include <exception>

namespace penguinPT::math {
	class Mat3f {
	public:
		Mat3f() {}
		Mat3f(float a, float b, float c, float d, float e, float f, float g, float h, float i) {
			data[0][0] = a, data[0][1] = b, data[0][2] = c;
			data[1][0] = d, data[1][1] = e, data[1][2] = f;
			data[2][0] = g, data[2][1] = h, data[2][2] = i;
		}
		~Mat3f() {}

		Mat3f inverse() const;
		float data[3][3] = {};
	};
	class Mat4f {
	public:
		__hostdev__ Mat4f() {}
		__hostdev__ Mat4f(
			float a, float b, float c, float d,
			float e, float f, float g, float h,
			float i, float j, float k, float l,
			float m, float n, float o, float p
		) {
			data[0][0] = a, data[0][1] = b, data[0][2] = c, data[0][3] = d;
			data[1][0] = e, data[1][1] = f, data[1][2] = g, data[1][3] = h;
			data[2][0] = i, data[2][1] = j, data[2][2] = k, data[2][3] = l;
			data[3][0] = m, data[3][1] = n, data[3][2] = o, data[3][3] = p;
		}
		__hostdev__ ~Mat4f() {}

		__hostdev__ Mat4f inverse() const;

		__hostdev__ static Mat4f rotationX(float angle);
		__hostdev__ static Mat4f rotationY(float angle);
		__hostdev__ static Mat4f rotationZ(float angle);

		__hostdev__ static Mat4f scale(nanovdb::Vec3f s);
		__hostdev__ static Mat4f scale(float s);

		__hostdev__ static Mat4f translation(nanovdb::Vec3f t);

		__hostdev__ nanovdb::Vec3f transformVector(const nanovdb::Vec3f V);
		__hostdev__ nanovdb::Vec3f transformPoint(const nanovdb::Vec3f P);

		template <typename floatType>
		__hostdev__ nanovdb::math::Ray<floatType> transformRay(const nanovdb::math::Ray<floatType> ray);

		float data[4][4] = { {1.f, 0.f, 0.f, 0.f}, {0.f, 1.f, 0.f, 0.f}, {0.f, 0.f, 1.f, 0.f}, {0.f, 0.f, 0.f, 1.f} };

		__hostdev__ nanovdb::Vec4f operator[](size_t i) { return { data[i][0], data[i][1] ,data[i][2] ,data[i][3] }; };
	};

	void printMatrix(Mat4f& m);
	void copyMatrix(Mat4f* src, Mat4f* dst);
}

/*
 * Copyright (c), Recep Aslantas.
 *
 * MIT License (MIT), http://opensource.org/licenses/MIT
 * Full license can be found in the LICENSE file
 */
penguinPT::math::Mat3f penguinPT::math::Mat3f::inverse() const {
	float a = data[0][0], b = data[0][1], c = data[0][2],
		d = data[1][0], e = data[1][1], f = data[1][2],
		g = data[2][0], h = data[2][1], i = data[2][2],

		c1 = e * i - f * h, c2 = d * i - g * f, c3 = d * h - g * e,
		idt = 1.0f / (a * c1 - b * c2 + c * c3), ndt = -idt;

	Mat3f dst;
	dst.data[0][0] = idt * c1;
	dst.data[0][1] = ndt * (b * i - h * c);
	dst.data[0][2] = idt * (b * f - e * c);
	dst.data[1][0] = ndt * c2;
	dst.data[1][1] = idt * (a * i - g * c);
	dst.data[1][2] = ndt * (a * f - d * c);
	dst.data[2][0] = idt * c3;
	dst.data[2][1] = ndt * (a * h - g * b);
	dst.data[2][2] = idt * (a * e - d * b);
	return dst;
}
__hostdev__ penguinPT::math::Mat4f penguinPT::math::Mat4f::inverse() const {
	float a = data[0][0], b = data[0][1], c = data[0][2], d = data[0][3],
		e = data[1][0], f = data[1][1], g = data[1][2], h = data[1][3],
		i = data[2][0], j = data[2][1], k = data[2][2], l = data[2][3],
		m = data[3][0], n = data[3][1], o = data[3][2], p = data[3][3],

		c1 = k * p - l * o, c2 = c * h - d * g, c3 = i * p - l * m,
		c4 = a * h - d * e, c5 = j * p - l * n, c6 = b * h - d * f,
		c7 = i * n - j * m, c8 = a * f - b * e, c9 = j * o - k * n,
		c10 = b * g - c * f, c11 = i * o - k * m, c12 = a * g - c * e,

		idt = 1.0f / (c8 * c1 + c4 * c9 + c10 * c3 + c2 * c7 - c12 * c5 - c6 * c11), ndt = -idt;

	Mat4f dst;
	dst.data[0][0] = (f * c1 - g * c5 + h * c9) * idt;
	dst.data[0][1] = (b * c1 - c * c5 + d * c9) * ndt;
	dst.data[0][2] = (n * c2 - o * c6 + p * c10) * idt;
	dst.data[0][3] = (j * c2 - k * c6 + l * c10) * ndt;

	dst.data[1][0] = (e * c1 - g * c3 + h * c11) * ndt;
	dst.data[1][1] = (a * c1 - c * c3 + d * c11) * idt;
	dst.data[1][2] = (m * c2 - o * c4 + p * c12) * ndt;
	dst.data[1][3] = (i * c2 - k * c4 + l * c12) * idt;

	dst.data[2][0] = (e * c5 - f * c3 + h * c7) * idt;
	dst.data[2][1] = (a * c5 - b * c3 + d * c7) * ndt;
	dst.data[2][2] = (m * c6 - n * c4 + p * c8) * idt;
	dst.data[2][3] = (i * c6 - j * c4 + l * c8) * ndt;

	dst.data[3][0] = (e * c9 - f * c11 + g * c7) * ndt;
	dst.data[3][1] = (a * c9 - b * c11 + c * c7) * idt;
	dst.data[3][2] = (m * c10 - n * c12 + o * c8) * ndt;
	dst.data[3][3] = (i * c10 - j * c12 + k * c8) * idt;
	return dst;
}

__hostdev__ penguinPT::math::Mat4f penguinPT::math::Mat4f::rotationX(float angle) {
	Mat4f m;
	m.data[1][1] = cosf(angle);
	m.data[1][2] = -sinf(angle);
	m.data[2][1] = sinf(angle);
	m.data[2][2] = cosf(angle);
	return m;
}
__hostdev__ penguinPT::math::Mat4f penguinPT::math::Mat4f::rotationY(float angle) {
	Mat4f m;
	m.data[0][0] = cosf(angle);
	m.data[0][2] = -sinf(angle);
	m.data[2][0] = sinf(angle);
	m.data[2][2] = cosf(angle);
	return m;
}
__hostdev__ penguinPT::math::Mat4f penguinPT::math::Mat4f::rotationZ(float angle) {
	Mat4f m;
	m.data[0][0] = cosf(angle);
	m.data[0][1] = -sinf(angle);
	m.data[1][0] = sinf(angle);
	m.data[1][1] = cosf(angle);
	return m;
}

__hostdev__ penguinPT::math::Mat4f penguinPT::math::Mat4f::scale(nanovdb::Vec3f s) {
	Mat4f m;
	m.data[0][0] = s[0];
	m.data[1][1] = s[1];
	m.data[2][2] = s[2];
	return m;
}

__hostdev__ penguinPT::math::Mat4f penguinPT::math::Mat4f::scale(float s) {
	Mat4f m;
	m.data[0][0] = s;
	m.data[1][1] = s;
	m.data[2][2] = s;
	return m;
}

__hostdev__ penguinPT::math::Mat4f penguinPT::math::Mat4f::translation(nanovdb::Vec3f t) {
	Mat4f m;
	m.data[0][3] = t[0];
	m.data[1][3] = t[1];
	m.data[2][3] = t[2];
	return m;
}

__hostdev__ nanovdb::Vec3f penguinPT::math::Mat4f::transformVector(const nanovdb::Vec3f V) {
	return nanovdb::Vec3f(
		data[0][0] * V[0] + data[0][1] * V[1] + data[0][2] * V[2],
		data[1][0] * V[0] + data[1][1] * V[1] + data[1][2] * V[2],
		data[2][0] * V[0] + data[2][1] * V[1] + data[2][2] * V[2]);
}
__hostdev__ nanovdb::Vec3f penguinPT::math::Mat4f::transformPoint(const nanovdb::Vec3f P) {
	nanovdb::Vec3f res = nanovdb::Vec3f(
		data[0][0] * P[0] + data[0][1] * P[1] + data[0][2] * P[2] + data[0][3],
		data[1][0] * P[0] + data[1][1] * P[1] + data[1][2] * P[2] + data[1][3],
		data[2][0] * P[0] + data[2][1] * P[1] + data[2][2] * P[2] + data[2][3]);
	float w = data[3][0] * P[0] + data[3][1] * P[1] + data[3][2] * P[2] + data[3][3];
	if (w == 1.f) return res;
	return res * (1.f / w);
}

template <typename floatType>
__hostdev__ nanovdb::math::Ray<floatType> penguinPT::math::Mat4f::transformRay(const nanovdb::math::Ray<floatType> ray) {
	nanovdb::Vec3f p_transformed = transformPoint(ray.eye());
	nanovdb::Vec3f d_transformed = transformVector(ray.dir());
	return nanovdb::math::Ray<floatType>(p_transformed, d_transformed);

}

void penguinPT::math::printMatrix(Mat4f& m) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++)
			std::cout << m.data[i][j] << " ";
		std::cout << "\n";
	}
}
void penguinPT::math::copyMatrix(Mat4f* src, Mat4f* dst) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++)
			dst->data[i][j] = src->data[i][j];
	}
}

__hostdev__ penguinPT::math::Mat4f operator*(penguinPT::math::Mat4f m1, penguinPT::math::Mat4f m2) {
	penguinPT::math::Mat4f dst;

	// first line
	dst.data[0][0] = m1.data[0][0] * m2.data[0][0] + m1.data[0][1] * m2.data[1][0] + m1.data[0][2] * m2.data[2][0] + m1.data[0][3] * m2.data[3][0];
	dst.data[0][1] = m1.data[0][0] * m2.data[0][1] + m1.data[0][1] * m2.data[1][1] + m1.data[0][2] * m2.data[2][1] + m1.data[0][3] * m2.data[3][1];
	dst.data[0][2] = m1.data[0][0] * m2.data[0][2] + m1.data[0][1] * m2.data[1][2] + m1.data[0][2] * m2.data[2][2] + m1.data[0][3] * m2.data[3][2];
	dst.data[0][3] = m1.data[0][0] * m2.data[0][3] + m1.data[0][1] * m2.data[1][3] + m1.data[0][2] * m2.data[2][3] + m1.data[0][3] * m2.data[3][3];

	// second line
	dst.data[1][0] = m1.data[1][0] * m2.data[0][0] + m1.data[1][1] * m2.data[1][0] + m1.data[1][2] * m2.data[2][0] + m1.data[1][3] * m2.data[3][0];
	dst.data[1][1] = m1.data[1][0] * m2.data[0][1] + m1.data[1][1] * m2.data[1][1] + m1.data[1][2] * m2.data[2][1] + m1.data[1][3] * m2.data[3][1];
	dst.data[1][2] = m1.data[1][0] * m2.data[0][2] + m1.data[1][1] * m2.data[1][2] + m1.data[1][2] * m2.data[2][2] + m1.data[1][3] * m2.data[3][2];
	dst.data[1][3] = m1.data[1][0] * m2.data[0][3] + m1.data[1][1] * m2.data[1][3] + m1.data[1][2] * m2.data[2][3] + m1.data[1][3] * m2.data[3][3];

	// third line
	dst.data[2][0] = m1.data[2][0] * m2.data[0][0] + m1.data[2][1] * m2.data[1][0] + m1.data[2][2] * m2.data[2][0] + m1.data[2][3] * m2.data[3][0];
	dst.data[2][1] = m1.data[2][0] * m2.data[0][1] + m1.data[2][1] * m2.data[1][1] + m1.data[2][2] * m2.data[2][1] + m1.data[2][3] * m2.data[3][1];
	dst.data[2][2] = m1.data[2][0] * m2.data[0][2] + m1.data[2][1] * m2.data[1][2] + m1.data[2][2] * m2.data[2][2] + m1.data[2][3] * m2.data[3][2];
	dst.data[2][3] = m1.data[2][0] * m2.data[0][3] + m1.data[2][1] * m2.data[1][3] + m1.data[2][2] * m2.data[2][3] + m1.data[2][3] * m2.data[3][3];

	// fourth line
	dst.data[3][0] = m1.data[3][0] * m2.data[0][0] + m1.data[3][1] * m2.data[1][0] + m1.data[3][2] * m2.data[2][0] + m1.data[3][3] * m2.data[3][0];
	dst.data[3][1] = m1.data[3][0] * m2.data[0][1] + m1.data[3][1] * m2.data[1][1] + m1.data[3][2] * m2.data[2][1] + m1.data[3][3] * m2.data[3][1];
	dst.data[3][2] = m1.data[3][0] * m2.data[0][2] + m1.data[3][1] * m2.data[1][2] + m1.data[3][2] * m2.data[2][2] + m1.data[3][3] * m2.data[3][2];
	dst.data[3][3] = m1.data[3][0] * m2.data[0][3] + m1.data[3][1] * m2.data[1][3] + m1.data[3][2] * m2.data[2][3] + m1.data[3][3] * m2.data[3][3];

	return dst;
}
