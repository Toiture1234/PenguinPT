#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/util/Matrix.h>

#include <ppt/core/Mesh.h>

namespace penguinPT {
	class BVHLoader {
	public:
		BVHLoader() {}
		~BVHLoader() {}

		// Loads a 3D model and stores the data in a BVH,
		// does nothing if the BVH already exists.
		void loadModel(std::string file_path, bool is_gpu_availble = false);

		// Loads a 3D model and stores the data in a BVH,
		// returns a pointer to the model's BVH.
		// Only creates a new BVH if the model hasn't been previouly loaded
		BVH* getModelBVH(std::string file_path, bool is_gpu_availble = false);

		void clearBVHLoader(bool is_gpu_availble = false);

		// Get a pointer to the BVH 
		BVH* getBVH(unsigned int index);
		BVH* getBVH(std::string BVH_name);

		bool doesBVHExists(std::string BVH_name);
	private:
		// Stores the data needed for BVH on the CPU or the GPU.
		//  -> CPU : element is created with malloc() and released with free()
		//  -> GPU : element is created with cudaMallocMananged() (accesible from CPU) and released with cudaFree()
		// NB : the GPU version will not work on devices without CUDA.
		std::vector<BVH*> BVH_data_list;

		// Each BVH is assigned a name so we can know if a BVH has already been loaded,
		// the i-th name corresponds to the i-th BVH.
		std::vector<std::string> BVH_name_list;
	};
}

bool penguinPT::BVHLoader::doesBVHExists(std::string BVH_name) {
	int i;
	return file_util::is_word_in_list(BVH_name, BVH_name_list, i);
}

penguinPT::BVH* penguinPT::BVHLoader::getBVH(unsigned int index) {
	if (index < 0 || index >= BVH_data_list.size()) return nullptr;
	return BVH_data_list.at(index);
}
penguinPT::BVH* penguinPT::BVHLoader::getBVH(std::string BVH_name) {
	int i;
	if (!file_util::is_word_in_list(BVH_name, BVH_name_list, i)) return nullptr;
	return getBVH(i);
}

void penguinPT::BVHLoader::clearBVHLoader(bool is_gpu_availble) {
	for (BVH* ptr : BVH_data_list) {
		if (is_gpu_availble) cudaFree(ptr);
		else free(ptr);
	}

	BVH_data_list.clear();
	BVH_name_list.clear();
}