#pragma once

#include <ppt/core/mesh_manager.h>

namespace penguinPT {
	class Scene {
	public:
		Scene() : volume_count(0), bsdf_count(0), volume_list(nullptr), bsdf_list(nullptr) {}
		~Scene() {}

		TLAS solid_geometry;

		Volume* volume_list;
		unsigned int volume_count;

		principled_BSDF* bsdf_list;
		unsigned int bsdf_count;

		Envmap environnement_map;

		__hostdev__ bool intersectScene(nanovdb::math::Ray<float> ray, hit_info& info);
		void clearScene(bool is_gpu_available = false);

		void setTLAS(std::vector<Mesh>& mesh_src, bool is_gpu_available = false);
		void clearTLAS(bool is_gpu_available = false);

		void setVolumeList(std::vector<Volume>& volume_src, bool is_gpu_available = false);
		void clearVolumeList(bool is_gpu_available = false);

		void setBSDFList(std::vector<principled_BSDF> bsdf_src, bool is_gpu_available = false);
		void clearBSDFList(bool is_gpu_available = false);

		void clearEnvmap(bool is_gpu_available = false);
	private:

	};
}

void penguinPT::Scene::setTLAS(std::vector<Mesh>& mesh_src, bool is_gpu_available) {
	solid_geometry.setMeshList(mesh_src, is_gpu_available);
	solid_geometry.build();
}
void penguinPT::Scene::clearTLAS(bool is_gpu_available) {
	solid_geometry.clean(is_gpu_available);
}

void penguinPT::Scene::setVolumeList(std::vector<Volume>& volume_src, bool is_gpu_available) {
	// clear previous volumes
	clearVolumeList(is_gpu_available);

	// initalize list
	volume_count = volume_src.size();
	if (is_gpu_available) {
		cudaMallocManaged((void**)&volume_list, volume_count * sizeof(Volume));
	}
	else {
		volume_list = (Volume*)malloc(volume_count * sizeof(Volume));
	}

	// copy data
	for (int i = 0; i < volume_count; i++) volume_list[i] = volume_src.at(i);
}
void penguinPT::Scene::clearVolumeList(bool is_gpu_available) {
	if (volume_count == 0) return;
	if (is_gpu_available) {
		cudaFree(volume_list);
	}
	else {
		free(volume_list);
	}
	volume_list = nullptr;
	volume_count = 0;
}

void penguinPT::Scene::setBSDFList(std::vector<principled_BSDF> bsdf_src, bool is_gpu_available) {
	// clear previous BSDF
	clearBSDFList(is_gpu_available);

	// initalize list
	bsdf_count = bsdf_src.size();
	if (is_gpu_available) {
		cudaMallocManaged((void**)&bsdf_list, bsdf_count * sizeof(principled_BSDF));
	}
	else {
		bsdf_list = (principled_BSDF*)malloc(bsdf_count * sizeof(principled_BSDF));
	}

	// copy data
	for (int i = 0; i < bsdf_count; i++) bsdf_list[i] = bsdf_src.at(i);
}
void penguinPT::Scene::clearBSDFList(bool is_gpu_available) {
	if (bsdf_count == 0) return;
	if (is_gpu_available) {
		cudaFree(bsdf_list);
	}
	else {
		free(bsdf_list);
	}
	bsdf_list = nullptr;
	bsdf_count = 0;
}

void penguinPT::Scene::clearEnvmap(bool is_gpu_available) {
	if (is_gpu_available) {
		CUDA_CHECK(cudaDestroyTextureObject(environnement_map.image));
		CUDA_CHECK(cudaDestroyTextureObject(environnement_map.cdf));
	}
}

void penguinPT::Scene::clearScene(bool is_gpu_available) {
	clearTLAS(is_gpu_available);
	clearVolumeList(is_gpu_available);
	clearBSDFList(is_gpu_available);
}

__hostdev__ bool penguinPT::Scene::intersectScene(nanovdb::math::Ray<float> ray, hit_info& info) {
	bool hit = false;
	hit = solid_geometry.intersectTLAS(ray, info);

	// intersect volumes
	for (unsigned int i = 0; i < volume_count; i++) {
		if (volume_list[i].intersect_volume_replace(ray, info.t, info.normal, info.all_volumes, info.nb_vol)) {
			hit = true;
			info.BSDF_index = BSDF_TROUGH_ID;
		}
	}

	return hit;
}
