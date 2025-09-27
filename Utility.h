#pragma once


// clamp any scalar
#define CLAMP(x, v_min, v_max) x > v_min ? x < v_max ? x : v_max : v_min

// max and min, faster than fminf and fmaxf
#define MAX_3(a, b) {a[0] > b[0] ? a[0] : b[0], a[1] > b[1] ? a[1] : b[1], a[2] > b[2] ? a[2] : b[2]}
#define MIN_3(a, b) {a[0] < b[0] ? a[0] : b[0], a[1] < b[1] ? a[1] : b[1], a[2] < b[2] ? a[2] : b[2]}
#define SIGN(x) (x > 0.f ? 1.f : x < 0.f ? -1.f : 0.f )

// WARNING : THIS STARTS AT 1 FOR COORDS
#define gotoxy(x, y) printf("%c[%d;%df", 0x1B, y, x);

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

	__hostdev__ inline nanovdb::Vec3f ToWorld(nanovdb::Vec3f& X, nanovdb::Vec3f& Y, nanovdb::Vec3f& Z, nanovdb::Vec3f& V) 
	{
		return V[0] * X + V[1] * Y + V[2] * Z;
	}
	__device__ inline nanovdb::Vec3f ToLocal(nanovdb::Vec3f X, nanovdb::Vec3f Y, nanovdb::Vec3f Z, nanovdb::Vec3f V)
	{
		return nanovdb::Vec3f(V.dot(X), V.dot(Y), V.dot(Z));
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