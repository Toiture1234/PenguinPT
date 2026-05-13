// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <curand_kernel.h>

// nano VDB
#include <nanovdb/io/IO.h>
#include <nanovdb/cuda/DeviceBuffer.h>
#include <nanovdb/tools/GridBuilder.h>
#include <nanovdb/math/Ray.h>
#include <nanovdb/math/HDDA.h>

// openVDB

// main libraries
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <fstream>
#include <filesystem>
#include <exception>

#include <ppt/util/options.h>

typedef curandStatePhilox4_32_10_t Rand_state;
#define randC(state) curand_uniform(state)
#define rand01 ((float)rand() / (float)RAND_MAX)


#define __hostdev__ __device__ __host__

#define PI 3.1415926535897932384626433832795f
#define TWO_PI 6.283185307179586476925286766559f
#define INV_TWO_PI 0.15915494309189533576888376337251f
#define INV_PI 0.31830988618379067153776752674503f
#define INV_4_PI 0.07957747154594766788444188168626f
#define PI_OVER_4 0.78539816339744830961566084581988f
#define PI_OVER_2 1.5707963267948966192313216916398f



// NEED TO MOVE THIS TO UTILITY
#define CUDA_CHECK(expr) { cudaError_t err = expr; if(err != CUDA_SUCCESS) { printf("CUDA ERROR AT LINE %i IN %s : %s \n", __LINE__, __FILE__, cudaGetErrorString(err)); exit(99); }}


template <typename vec3T> __hostdev__ inline void safe_both(vec3T normal, nanovdb::math::Ray<float>& ray, bool through) {
	float k = through ? -1.f : 1.f;
	ray.setEye(ray.eye() + normal * SAFE_OFFSET * k);
}

// clamp any scalar
#define CLAMP(x, v_min, v_max) x > v_min ? x < v_max ? x : v_max : v_min

// max and min, faster than fminf and fmaxf
#define MAX_3(a, b) {a[0] > b[0] ? a[0] : b[0], a[1] > b[1] ? a[1] : b[1], a[2] > b[2] ? a[2] : b[2]}
#define MIN_3(a, b) {a[0] < b[0] ? a[0] : b[0], a[1] < b[1] ? a[1] : b[1], a[2] < b[2] ? a[2] : b[2]}
#define SIGN(x) (x > 0.f ? 1.f : x < 0.f ? -1.f : 0.f )


// WARNING : THIS STARTS AT 1 FOR COORDS
#define gotoxy(x, y) printf("%c[%d;%df", 0x1B, y, x);

namespace penguinPT::util {
	template <typename T>
	__hostdev__ inline void swapCUDA(T& a, T& b) {
		T temp = a;
		a = b;
		b = temp;
	}
	__hostdev__ inline float clamp(float a, float x, float y) {
		return a < x ? x : a > y ? y : a;
	}
	__hostdev__ inline int clamp(int a, int x, int y) {
		return a < x ? x : a > y ? y : a;
	}
	__hostdev__ inline nanovdb::Vec3f clamp(const nanovdb::Vec3f& a, const nanovdb::Vec3f& l, const nanovdb::Vec3f& h) {
		return { clamp(a[0], l[0], h[0]), clamp(a[1], l[1], h[1]), clamp(a[2], l[2], h[3]) };
	}
	__hostdev__ inline nanovdb::Vec3f log2_3(nanovdb::Vec3f x) {
		return { log2f(x[0]), log2f(x[1]), log2f(x[2]) };
	}
	__hostdev__ inline nanovdb::Vec3f pow3f(nanovdb::Vec3f x, float power) {
		return { powf(x[0], power), powf(x[1], power), powf(x[2], power) };
	}
	__hostdev__ inline float avg(const nanovdb::Vec3f x) {
		return 1.f / 3.f * (x[0] + x[1] + x[2]);
	}
	__hostdev__ inline nanovdb::Vec3f clamp3(nanovdb::Vec3f x, float v_min, float v_max) {
		return { CLAMP(x[0], v_min, v_max),CLAMP(x[1], v_min, v_max) ,CLAMP(x[2], v_min, v_max) };
	}
	__hostdev__ inline nanovdb::Vec3f aces(const nanovdb::Vec3f& x) {
		const float a = 2.51;
		const float b = 0.03;
		const float c = 2.43;
		const float d = 0.59;
		const float e = 0.14;
		return clamp3((x * (a * x + nanovdb::Vec3f(b))) / (x * (c * x + nanovdb::Vec3f(d)) + nanovdb::Vec3f(e)), 0.0, 1.0);
	}
	__hostdev__ inline void Onb(nanovdb::Vec3f N, nanovdb::Vec3f& T, nanovdb::Vec3f& B)
	{
		nanovdb::Vec3f up = abs(N[2]) < 0.999 ? nanovdb::Vec3f(0, 0, 1) : nanovdb::Vec3f(1, 0, 0);
		T = up.cross(N).normalize();
		B = N.cross(T);
	}

	template <typename vec3Type> 
	__hostdev__ inline vec3Type refract(vec3Type I, vec3Type N, float ior) { // actually not IOR but the ratio of IOR
		float k = 1.0f - ior * ior * (1.0f - N.dot(I) * N.dot(I));
		if (k < 0.f) return vec3Type(0.f);
		else return ior * I - (ior * N.dot(I) + sqrtf(k)) * N;
	}
	template <typename vec3T> 
	__hostdev__ inline vec3T reflect(vec3T I, vec3T N) {
		return I - 2.0f * N.dot(I) * N;
	}

	template <typename T>
	__hostdev__ inline T mix(T a, T b, float m) {
		return a * (1.f - m) + b * m;
	}
	template <typename T>
	__hostdev__ inline T exp3f(T a) {
		return { expf(a[0]), expf(a[1]), expf(a[2]) };
	}

	template <typename vec3T>
	__hostdev__ inline void safe(vec3T normal, nanovdb::math::Ray<float>& ray) {
		//ray.reset(ray.eye() + normal * SAFE_OFFSET, ray.dir());
		ray.setEye(ray.eye() + normal * SAFE_OFFSET);
	}
	__hostdev__ inline float fresnelAmount(float n1, float n2, nanovdb::Vec3f normal, nanovdb::Vec3f I, float f0, float f90) {
		float r0 = (n1 - n2) / (n1 + n2);
		r0 *= r0;
		float cosX = -normal.dot(I);
		if (n1 > n2) {
			float n = n1 / n2;
			float sinT2 = n * n * (1.0f - cosX * cosX);
			if (sinT2 > 1.0f) return f90;
			cosX = sqrtf(1.0f - sinT2);
		}
		float x = 1.0f - cosX;
		float ret = r0 + (1.0f - r0) * x * x * x * x * x;
		return mix(f0, f90, ret);
	}

	__device__ inline nanovdb::Vec3f generateUniformSample(Rand_state& rand_state) {
		float z = randC(&rand_state) * 2.0f - 1.0f;
		float a = randC(&rand_state) * PI * 2.f;
		float r = sqrtf(1.0f - z * z);
		float x = r * cosf(a);
		float y = r * sinf(a);
		return { x, y, z };
	}
	__hostdev__ inline nanovdb::Vec3f cosineSampleHemisphere(float u1, float u2) {
		u1 = u1 * 2.f - 1.f, u2 = u2 * 2.f - 1.f;
		if (u1 == 0.f && u2 == 0.f) return nanovdb::Vec3f(0.f, 0.f, 1.f);

		float theta, r;
		if (fabsf(u1) > fabsf(u2)) {
			r = u1;
			theta = PI_OVER_4 * (u2 / u1);
		}
		else {
			r = u2;
			theta = PI_OVER_2 - PI_OVER_4 * (u1 / u2);
		}
		float cos_theta = r * cosf(theta), sin_theta = r * sinf(theta);
		float z = sqrtf(fmaxf(1.f - cos_theta * cos_theta - sin_theta * sin_theta, 0.f));
		return nanovdb::Vec3f(cos_theta, sin_theta, z);
	}
	// omg Inigo Quilez you saved my life
	__hostdev__ inline nanovdb::Vec3f refIfNeg(nanovdb::Vec3f v, nanovdb::Vec3f r) {
		float k = v.dot(r);
		return k > 0.0f ? v : v - 2.0f * r * k;
	}
	__hostdev__ inline nanovdb::Vec3f clipVecNoLength(nanovdb::Vec3f v, nanovdb::Vec3f r)
	{
		float k = v.dot(r);
		return (k > 0.0) ? v : v - r * k;
	}

	__hostdev__ nanovdb::Vec3f sphericalDirection(float sinT, float cosT, float phi) {
		return nanovdb::Vec3f(sinT * cosf(phi), sinT * sinf(phi), cosT);
	}
	__hostdev__ float CosTheta(const nanovdb::Vec3f& w) { return w[2]; }
	__hostdev__ float Cos2Theta(const nanovdb::Vec3f& w) { return w[2] * w[2]; }

	__hostdev__ float Sin2Theta(const nanovdb::Vec3f& w) { return CLAMP(1.f - CosTheta(w), 0.f, 1.f); }
	__hostdev__ float SinTheta(const nanovdb::Vec3f& w) { return sqrtf(Sin2Theta(w)); }

	__hostdev__ float TanTheta(const nanovdb::Vec3f& w) { return SinTheta(w) / CosTheta(w); }
	__hostdev__ float Tan2Theta(const nanovdb::Vec3f& w) { return Sin2Theta(w) / Cos2Theta(w); }

	__hostdev__ float CosPhi(const nanovdb::Vec3f& w) {
		float sinTheta = SinTheta(w);
		return sinTheta == 0.f ? 1.f : CLAMP(w[0] / sinTheta, -1.f, 1.f);
	}
	__hostdev__ float SinPhi(const nanovdb::Vec3f& w) {
		float sinTheta = SinTheta(w);
		return sinTheta == 0.f ? 0.f : CLAMP(w[1] / sinTheta, -1.f, 1.f);
	}

	__hostdev__ float Cos2Phi(const nanovdb::Vec3f& w) { return CosPhi(w) * CosPhi(w); }
	__hostdev__ float Sin2Phi(const nanovdb::Vec3f& w) { return SinPhi(w) * SinPhi(w); }

	__hostdev__ inline nanovdb::Vec3f ToWorld(nanovdb::Vec3f& X, nanovdb::Vec3f& Y, nanovdb::Vec3f& Z, nanovdb::Vec3f V) 
	{
		return V[0] * X + V[1] * Y + V[2] * Z;
	}
	__hostdev__ inline nanovdb::Vec3f ToLocal(nanovdb::Vec3f X, nanovdb::Vec3f Y, nanovdb::Vec3f Z, nanovdb::Vec3f V)
	{
		return nanovdb::Vec3f(V.dot(X), V.dot(Y), V.dot(Z));
	}
	__hostdev__ inline float luminance(const nanovdb::Vec3f& l) {
		return l.dot(nanovdb::Vec3f(0.21f, 0.72f, 0.07f));
	}
	__hostdev__ inline float SchlickWeight(float u)
	{
		float m = CLAMP(1.0f - u, 0.0f, 1.0f);
		float m2 = m * m;
		return m2 * m2 * m;
	}
	__hostdev__ inline float powerHeuristic(float a, float b) {
		float t = a * a;
		return t / (b * b + t);
	}
}
namespace penguinPT::file_util {
	bool is_char_in_list(std::vector<char> c_list, char c) {
		if (c_list.size() == 0) return false;
		for (int i = 0; i < c_list.size(); i++) {
			if (c_list.at(i) == c) return true;
		}
		return false;
	}
	bool is_word_in_list(std::string word, std::vector<std::string> list, int& i) {
		if (list.size() == 0) return false;
		for (i = 0; i < list.size(); i++) if (word == list.at(i)) { return true; }
		return false;
	}

	std::string read_to_escape(std::string line, unsigned int& i, std::vector<char> c_list) {
		std::string result = "";

		for (i; i < line.length(); i++) {
			char c = line[i];
			if (c == '\n' || is_char_in_list(c_list, c)) return result;
			else result += c;
		}
		return result;
	}
	std::vector<std::string> get_line_tokens(std::string line, std::vector<char> c_list) {
		unsigned int i = 0;
		std::vector<std::string> tokens;

		while (i < line.length()) {
			char c = line[i];

			if (is_char_in_list(c_list, c)) i++;
			else if (c == '\n') break;
			else {
				tokens.push_back(read_to_escape(line, i, c_list));
			}
		}
		return tokens;
	}
	std::string remove_quote(std::string s) {
		std::string f;
		for (int i = 0; i < s.size(); i++) if (s[i] != '\"') f += s[i];
		return f;
	}
}