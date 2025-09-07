#pragma once
namespace penguinPT {
	enum BSDF_types {
		BSDF_null = 0,
		BSDF_diffuse,
		BSDF_specular,
		BSDF_glass,
		BSDF_blinn_phong,
		BSDF_trough
	};

#define BSDF_EVAL_PARAMS nanovdb::math::Ray<float> ray, nanovdb::Vec3f N, nanovdb::Vec3f L_dir, float& pdf
#define BSDF_SAMPLE_PARAMS nanovdb::math::Ray<float> ray, nanovdb::Vec3f N, nanovdb::Vec3f& L_out, Rand_state& rng_state, float& pdf
#define BSDF_EVAL_PARAMS_FUNC ray, N, L_dir, pdf
#define BSDF_SAMPLE_PARAMS_FUNC ray, N, L_out, rng_state, pdf

	// Class for handling different BSDF
	// contains two functions : eval and sample,
	// these functions will depend on which BSDF is used
	class BSDF {
	public:
		nanovdb::Vec3f albedo;
		float roughness;
		float metalness;
		float IOR;
		nanovdb::Vec3f emission;

		unsigned int bsdf_type = BSDF_null;
	public:
		__hostdev__ BSDF() : albedo(0.f), roughness(0.f), metalness(0.f), IOR(0.f), emission(0.f) {}
		__hostdev__ ~BSDF() {}

		// Eval BSDF response based on incoming and exiting rays along with the normal of the surface,
		// returns the response and the pdf can be obtained in parameters.
		//__device__ nanovdb::Vec3f eval(nanovdb::math::Ray<float> ray, nanovdb::Vec3f N, nanovdb::Vec3f L_dir, float& pdf, Material_params params);
		__device__ inline nanovdb::Vec3f eval(BSDF_EVAL_PARAMS);

		// Sample BSDF based on the normal, incoming ray direction and rng values,
		// returns the BSDF response at sampled direction, the latter is stored in L_out, 
		// pdf is also returned in pdf parameter.
		//__device__ nanovdb::Vec3f sample(nanovdb::math::Ray<float> ray, nanovdb::Vec3f N, nanovdb::Vec3f& L_out, Rand_state& rng_state, float& pdf, Material_params params);
		__device__ inline nanovdb::Vec3f sample(BSDF_SAMPLE_PARAMS);


		// Simple diffuse BSDF, the response is cosine weighted based on the normal of the surface
		__device__ inline nanovdb::Vec3f eval_diffuse(BSDF_EVAL_PARAMS);
		__device__ inline nanovdb::Vec3f sample_diffuse(BSDF_SAMPLE_PARAMS);

		// null BSDF
		__device__ inline nanovdb::Vec3f eval_null(BSDF_EVAL_PARAMS);
		__device__ inline nanovdb::Vec3f sample_null(BSDF_SAMPLE_PARAMS);

		// trough BSDF
		__hostdev__ inline nanovdb::Vec3f eval_trough(BSDF_EVAL_PARAMS);
		__hostdev__ inline nanovdb::Vec3f sample_trough(BSDF_SAMPLE_PARAMS);

		// specular BSDF
		__device__ inline nanovdb::Vec3f eval_specular(BSDF_EVAL_PARAMS);
		__device__ inline nanovdb::Vec3f sample_specular(BSDF_SAMPLE_PARAMS);
	};

	// diffuse BSDF
	__device__ inline nanovdb::Vec3f BSDF::eval_diffuse(BSDF_EVAL_PARAMS)  {
		float cosine_term = N.dot(L_dir);
		pdf = INV_PI * cosine_term;
		return INV_PI * albedo * cosine_term;
	}

	__device__ inline nanovdb::Vec3f BSDF::sample_diffuse(BSDF_SAMPLE_PARAMS) {
#ifndef QUICK_DIFFUSE
		float u1 = randC(&rng_state);
		float u2 = randC(&rng_state);

		float phi = TWO_PI * u2;
		float cos_theta = sqrtf(u1);
		float sin_theta = sqrtf(1.f - cos_theta * cos_theta);

		nanovdb::Vec3f T, B;
		util::Onb(N, T, B);

		L_out = (sin_theta * cosf(phi) * T + sin_theta * sinf(phi) * B + cos_theta * N).normalize();
		pdf = INV_PI * cos_theta;
		return INV_PI * albedo * cos_theta;
#else
		L_out = (N + util::generateUniformSample(rng_state)).normalize();
		return eval_diffuse(ray, N, L_out, pdf);
#endif
	}
	
	// null bsdf
	__device__ inline nanovdb::Vec3f BSDF::eval_null(BSDF_EVAL_PARAMS) {
		pdf = 0.f;
		return { 0.f, 0.f, 0.f };
	}
	__device__ inline nanovdb::Vec3f BSDF::sample_null(BSDF_SAMPLE_PARAMS) {
		pdf = 0.f;
		L_out = nanovdb::Vec3f(0.f);
		return { 0.f, 0.f, 0.f };
	}

	// trough bsdf
	__hostdev__ inline nanovdb::Vec3f BSDF::eval_trough(BSDF_EVAL_PARAMS) {
		float k = ray.dir().dot(L_dir);
		pdf = k == 1.f ? 1.f : 0.f;
		return nanovdb::Vec3f(pdf);
	}
	__hostdev__ inline nanovdb::Vec3f BSDF::sample_trough(BSDF_SAMPLE_PARAMS) {
		L_out = ray.dir();
		pdf = 1.f;
		return nanovdb::Vec3f(1.f);
	}

	// specular bsdf
	__device__ inline nanovdb::Vec3f BSDF::eval_specular(BSDF_EVAL_PARAMS) {
		nanovdb::Vec3f Reflected = util::reflect(ray.dir(), N);
		float n = 1.f + 1000.f * (1.f - roughness) * (1.f - roughness);

		float cosAlpha = CLAMP(Reflected.dot(L_dir), 0.f, 1.f);
		float G = N.dot(L_dir);
		return powf(cosAlpha, n) * util::mix(nanovdb::Vec3f(1.f), albedo, metalness) * (n + 1.f) * INV_TWO_PI * G;
	}
	__device__ inline nanovdb::Vec3f BSDF::sample_specular(BSDF_SAMPLE_PARAMS) {
		nanovdb::Vec3f Reflected = util::reflect(ray.dir(), N);
		float n = 1.f + 1000.f * (1.f - roughness) * (1.f - roughness);

		float u1 = randC(&rng_state);
		float u2 = randC(&rng_state);

		float phi = TWO_PI * u2;
		float cos_alpha = powf(u1, 1.f / (n + 1.f));
		float sin_alpha = sqrtf(1.f - cos_alpha * cos_alpha);

		nanovdb::Vec3f T, B;
		util::Onb(Reflected, T, B);

		L_out = (sin_alpha * cosf(phi) * T + sin_alpha * sinf(phi) * B + cos_alpha * Reflected).normalize();
		float G = N.dot(L_out);
		pdf = powf(cos_alpha, n) * (n + 1.f) * INV_TWO_PI * G;

		if (L_out.dot(N) <= 0.f) pdf = 0.f;
		return pdf * util::mix(nanovdb::Vec3f(1.f), albedo, metalness);
	}

	__device__ inline nanovdb::Vec3f BSDF::eval(BSDF_EVAL_PARAMS) {

		switch (bsdf_type)
		{
		case BSDF_null :
			return eval_null(BSDF_EVAL_PARAMS_FUNC);
		case BSDF_diffuse :
			return eval_diffuse(BSDF_EVAL_PARAMS_FUNC);
		case BSDF_trough:
			return eval_trough(BSDF_EVAL_PARAMS_FUNC);
		case BSDF_specular:
			return eval_specular(BSDF_EVAL_PARAMS_FUNC);
		}
	}

	__device__ inline nanovdb::Vec3f BSDF::sample(BSDF_SAMPLE_PARAMS) {

		switch (bsdf_type)
		{
		case BSDF_null:
			return sample_null(BSDF_SAMPLE_PARAMS_FUNC);
		case BSDF_diffuse:
			return sample_diffuse(BSDF_SAMPLE_PARAMS_FUNC);
		case BSDF_trough:
			return sample_trough(BSDF_SAMPLE_PARAMS_FUNC);
		case BSDF_specular:
			return sample_specular(BSDF_SAMPLE_PARAMS_FUNC);
		}
	}
}