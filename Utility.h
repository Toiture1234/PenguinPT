#pragma once


// clamp any scalar
#define CLAMP(x, v_min, v_max) x > v_min ? x < v_max ? x : v_max : v_min

// max and min, faster than fminf and fmaxf
#define MAX_3(a, b) {a[0] > b[0] ? a[0] : b[0], a[1] > b[1] ? a[1] : b[1], a[2] > b[2] ? a[2] : b[2]}
#define MIN_3(a, b) {a[0] < b[0] ? a[0] : b[0], a[1] < b[1] ? a[1] : b[1], a[2] < b[2] ? a[2] : b[2]}
#define SIGN(x) (x > 0.f ? 1.f : x < 0.f ? -1.f : 0.f )

namespace penguinPT::util {
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

	template <typename vec3T>
	__hostdev__ inline void safe(vec3T normal, nanovdb::math::Ray<float>& ray) {
		//ray.reset(ray.eye() + normal * SAFE_OFFSET, ray.dir());
		ray.setEye(ray.eye() + normal * SAFE_OFFSET);
	}

	__device__ inline nanovdb::Vec3f generateUniformSample(Rand_state& rand_state) {
		float z = randC(&rand_state) * 2.0f - 1.0f;
		float a = randC(&rand_state) * PI * 2.f;
		float r = sqrtf(1.0f - z * z);
		float x = r * cosf(a);
		float y = r * sinf(a);
		return { x, y, z };
	}
	// omg Inigo Quilez you saved my life
	__hostdev__ inline nanovdb::Vec3f refIfNeg(nanovdb::Vec3f v, nanovdb::Vec3f r) {
		float k = v.dot(r);
		return k > 0.0f ? v : v - 2.0f * r * k;
	}
	
}
namespace penguinPT {
	// stores data from hit surface to lighten intersection functions
	class hit_info {
	public:
		nanovdb::Vec3f normal;
		nanovdb::Vec3f trueNormal;
		float t;
		float2 uv = make_float2(0.f, 0.f);
		nanovdb::Vec3u debug;
		int BSDF_index;

		// to sample volume density
		bool insideVolume;
		int volumeIndex;

		__hostdev__ hit_info() : normal(0.f), t(0.f), debug(0), BSDF_index(0), insideVolume(false), volumeIndex(0), trueNormal(0.f) {}
	};
}