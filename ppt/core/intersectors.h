// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/core/volumes.h>
#include <ppt/core/Material.h>
#include <ppt/core/envmap.h>
#include <ppt/util/Matrix.h>

namespace penguinPT {
	class hit_info {
	public:
		nanovdb::Vec3f normal;
		nanovdb::Vec3f trueNormal;
		float t;
		float2 uv = make_float2(0.f, 0.f);
		nanovdb::Vec3u debug;
		int BSDF_index;

		Volume* all_volumes[VOLUME_STACK_SIZE];
		int nb_vol;	

		__hostdev__ hit_info() : normal(0.f), t(MAX_DISTANCE), debug(0), BSDF_index(0), trueNormal(0.f), nb_vol(0) {}
	};
	class SceneHit {
	public:
		__hostdev__ SceneHit() :
			m_t(MAX_DISTANCE),
			m_uv({ 0.f, 0.f }),
			m_normal(0.f),
			m_surface_bsdf(nullptr),
			m_volume_stack_size(0),
			m_medium_stack_size(0),
			m_medium_chosen(false)
		{
		}
		__hostdev__ ~SceneHit() {}

		float m_t;
		float2 m_uv;

		nanovdb::Vec3f m_normal;

		principled_BSDF* m_surface_bsdf;

		Volume* m_volume_stack[VOLUME_STACK_SIZE];
		unsigned int m_volume_stack_size;

		Medium* m_medium_stack[MEDIUM_STACK_SIZE];
		unsigned int m_medium_stack_size;
		bool m_medium_chosen;
	private:
	};

	class Sphere {
	public:
		nanovdb::Vec3f center;
		float radius;

		__hostdev__ Sphere() : center(0.f), radius(0.f) {}
		__hostdev__ ~Sphere() {}
	};
	// simple triangle class, need to be smaller and have normals and verticies in an other array
	class Triangle {
	public:
		nanovdb::Vec3f A, B, C, Origin;

#if MODE_TRIANGLE == 0
		nanovdb::Vec3f nA, nB, nC;
		unsigned int BSDF_index;
		__hostdev__ Triangle() : A(0.f), B(0.f), C(0.f), Origin(0.f), nA(0.f), nB(0.f), nC(0.f), BSDF_index(BASE_BSDF) {}
		__hostdev__ Triangle(nanovdb::Vec3f a, nanovdb::Vec3f b, nanovdb::Vec3f c) : A(a), B(b), C(c), Origin(0.f), nA(0.f), nB(0.f), nC(0.f), BSDF_index(BASE_BSDF) {}
#else 
		__hostdev__ Triangle() : A(0.f), B(0.f), C(0.f), Origin(0.f) {}
		__hostdev__ Triangle(nanovdb::Vec3f a, nanovdb::Vec3f b, nanovdb::Vec3f c) : A(a), B(b), C(c), Origin(0.f) {}
#endif

		__hostdev__ ~Triangle(){}
	};
	class Triangle_data {
	public:
		nanovdb::Vec3f nA, nB, nC;
		float2 uvA, uvB, uvC;
#if BSDF_DATA_HOLDER == 0
		unsigned int BSDF_index;
		__hostdev__ Triangle_data() : nA(0.f), nB(0.f), nC(0.f), BSDF_index(BASE_BSDF), uvA({ 0.f, 0.f }), uvB({ 0.f, 0.f }), uvC({ 0.f, 0.f }) {};
#else
		principled_BSDF* bsdf_ptr;
		__hostdev__ Triangle_data() : nA(0.f), nB(0.f), nC(0.f), bsdf_ptr(nullptr), uvA({ 0.f, 0.f }), uvB({ 0.f, 0.f }), uvC({ 0.f, 0.f }) {};
#endif
		__hostdev__ ~Triangle_data() {}
	};
	class BVH_node {
	public:
		nanovdb::Vec3f boxMin, boxMax;

		int leftFirst, triangleCount;
		__hostdev__ BVH_node() : boxMin(0.f), boxMax(0.f), leftFirst(0), triangleCount(0) {}
		__hostdev__ ~BVH_node() {}
	};
	class AABB {
	public:
		nanovdb::Vec3f box_min = nanovdb::Vec3f(1e30f), box_max = nanovdb::Vec3f(-1e30f);
		__hostdev__ void grow(nanovdb::Vec3f p) {
			box_min = MIN_3(box_min, p);
			box_max = MAX_3(box_max, p);
		}
		__hostdev__ void grow(AABB& b) {
			if (b.box_min[0] != 1e30f) {
				grow(b.box_min);
				grow(b.box_max);
			}
		}
		__hostdev__ float area() const {
			nanovdb::Vec3f e = box_max - box_min;
			return e[0] * e[1] + e[1] * e[2] + e[2] * e[0];
		}

		__hostdev__ AABB() {}
		__hostdev__ ~AABB() {}
	};
	class BIN {
	public:
		AABB bounds;
		int triangle_count = 0;

		__hostdev__ BIN() {}
		__hostdev__ ~BIN() {}
	};

	// -----------------------------------------------------------------------
	//                         Primitive  intersectors
	// -----------------------------------------------------------------------


	// basic triangle intersector that returns the distance of the surface, along as normal and uv coordinates
	// returns -1 if no surface is hit
	__hostdev__ float intersect_triangle(nanovdb::math::Ray<float> ray, nanovdb::Vec3f& normal, Triangle tri, float2& uv) {
		nanovdb::Vec3f v1v0 = tri.B - tri.A;
		nanovdb::Vec3f v2v0 = tri.C - tri.A;
		nanovdb::Vec3f rov0 = ray.eye() - tri.A;
		nanovdb::Vec3f  n = v1v0.cross(v2v0);
		nanovdb::Vec3f  q = rov0.cross(ray.dir());
		float d = 1.0f / ray.dir().dot(n);
		float u = d * (-q).dot(v2v0);
		float v = d * q.dot(v1v0);
		float t = d * (-n).dot(rov0);
		if (u < 0.0f || v < 0.0f || (u + v) > 1.0f) t = -1.0f;
		float w = 1.0f - u - v;

		uv = make_float2(u, v);
		normal = (tri.B - tri.A).cross(tri.C - tri.A).normalize();
#if MODE_TRIANGLE == 0
		normal = tri.nA.dot(tri.nA) > 0.f ? (tri.nA * w + tri.nB * u + tri.nC * v).normalize() : normal;
#endif
		return t;
	}
	__hostdev__ bool intersect_triangle_uv(nanovdb::math::Ray<float> ray, Triangle& tri, float2& uv, float& t_0) {
		nanovdb::Vec3f v1v0 = tri.B - tri.A;
		nanovdb::Vec3f v2v0 = tri.C - tri.A;
		nanovdb::Vec3f rov0 = ray.eye() - tri.A;
		nanovdb::Vec3f  n = v1v0.cross(v2v0);
		nanovdb::Vec3f  q = rov0.cross(ray.dir());
		float d = 1.0f / ray.dir().dot(n);
		float u = d * (-q).dot(v2v0);
		float v = d * q.dot(v1v0);
		float t = d * (-n).dot(rov0);
		if (u < 0.0f || v < 0.0f || (u + v) > 1.0f || t > t_0 || t < SAFE_OFFSET) return false;
		float w = 1.0f - u - v;

		uv = make_float2(u, v);
		t_0 = t;
		//normal = (tri.B - tri.A).cross(tri.C - tri.A).normalize();
		//normal = tri.nA.dot(tri.nA) > 0.f ? (tri.nA * w + tri.nB * u + tri.nC * v).normalize() : normal;
		return true;
	}
	// triangle intersector that takes a distance input and replace it if the ray collides with triangle and 
	// if the distance to that triangle is smaller than the previous distance
	__hostdev__ bool intersect_triangle_replace(nanovdb::math::Ray<float> ray, nanovdb::Vec3f& normal, nanovdb::Vec3f& trueNormal, Triangle tri, float2& uv, float& t_out, int& BSDF_i) {
		nanovdb::Vec3f v1v0 = tri.B - tri.A;
		nanovdb::Vec3f v2v0 = tri.C - tri.A;
		nanovdb::Vec3f rov0 = ray.eye() - tri.A;
		nanovdb::Vec3f  n = v1v0.cross(v2v0);
		nanovdb::Vec3f  q = rov0.cross(ray.dir());
		float d = 1.0f / ray.dir().dot(n);
		float u = d * (-q).dot(v2v0);
		float v = d * q.dot(v1v0);
		float t = d * (-n).dot(rov0);
		if (u < 0.0f || v < 0.0f || (u + v) > 1.0f || t > t_out || t < SAFE_OFFSET) return false;
		float w = 1.f - u - v;

		uv = make_float2(u, v);
		t_out = t;
		trueNormal = (tri.B - tri.A).cross(tri.C - tri.A).normalize();
		float u_dot = trueNormal.dot(ray.dir());
		float mult = -SIGN(u_dot);
#if MODE_TRIANGLE == 0
		normal = (tri.nA.dot(tri.nA) > 0.f ? (tri.nA * w + tri.nB * u + tri.nC * v).normalize() : trueNormal) * mult;
		BSDF_i = tri.BSDF_index;
#else 
		normal = trueNormal;
#endif
		return true;
	}
	__hostdev__ bool intersectTriangleMethodB(
		nanovdb::math::Ray<float> ray, 
		Triangle& tri, 
		Triangle_data& tri_info,
		SceneHit& hinfo) 
	{
#if BSDF_DATA_HOLDER == 0
		return false;
#else
		nanovdb::Vec3f v1v0 = tri.B - tri.A;
		nanovdb::Vec3f v2v0 = tri.C - tri.A;
		nanovdb::Vec3f rov0 = ray.eye() - tri.A;
		nanovdb::Vec3f  n = v1v0.cross(v2v0);
		nanovdb::Vec3f  q = rov0.cross(ray.dir());
		float det = ray.dir().dot(n);
		float d = 1.0f / det;
		float u = d * (-q).dot(v2v0);
		float v = d * q.dot(v1v0);
		float t = d * (-n).dot(rov0);
		if (u < 0.0f || v < 0.0f || (u + v) > 1.0f || t < SAFE_OFFSET) return false;
		if (hinfo.m_t < t) {
			return false;
		}
		float w = 1.0f - u - v;

		hinfo.m_t = t;
		hinfo.m_normal = n * -det; // no smooth normal rn
		hinfo.m_surface_bsdf = tri_info.bsdf_ptr;
		hinfo.m_uv = { tri_info.uvA.x * w + tri_info.uvB.x * u + tri_info.uvC.x * v,
			tri_info.uvA.y * w + tri_info.uvB.y * u + tri_info.uvC.y * v };

		return true;
#endif
	}

	// box intersector, returns the nearest distance to the ray, returns 1e30 if the ray is inside box
	__hostdev__ inline float boxIntersect_float(nanovdb::math::Ray<float> ray, nanovdb::Vec3f aabbMin, nanovdb::Vec3f aabbMax) {
		nanovdb::Vec3f planesMin = -1.0 * (ray.eye() - aabbMin) * ray.invDir();
		nanovdb::Vec3f planesMax = -1.0 * (ray.eye() - aabbMax) * ray.invDir();

		nanovdb::Vec3f planesNear = MIN_3(planesMin, planesMax);
		nanovdb::Vec3f planesFar = MAX_3(planesMin, planesMax);

		float tNear = planesNear.max();
		float tFar = planesFar.min();

		if (tFar > tNear && tFar > 0.f) return tNear;
		else return 1e30f;
	}
	__hostdev__ inline bool boxIntersect_replace(nanovdb::math::Ray<float> ray, nanovdb::Vec3f aabbMin, nanovdb::Vec3f aabbMax, nanovdb::Vec3f& normal, float& t_out, bool& isInside) {
		nanovdb::Vec3f planesMin = -1.0 * (ray.eye() - aabbMin) * ray.invDir();
		nanovdb::Vec3f planesMax = -1.0 * (ray.eye() - aabbMax) * ray.invDir();

		nanovdb::Vec3f planesNear = MIN_3(planesMin, planesMax);
		nanovdb::Vec3f planesFar = MAX_3(planesMin, planesMax);

		float tNear = planesNear.max();
		float tFar = planesFar.min();

		if (tFar < 0.f) return false;
		isInside = tNear < 0.f;
		normal = { 0.f, 0.f, 0.f }; // need to change this !
		t_out = isInside ? tFar : tNear;

		return true;
	}
}