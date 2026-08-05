// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>

#include <ppt/core/intersectors.h>
#include <ppt/core/Mesh.h>

namespace penguinPT {
	struct TLASNode {
		nanovdb::Vec3f aabb_min;
		nanovdb::Vec3f aabb_max;

		unsigned int left_right;
		unsigned int BLAS;
		
		__hostdev__ bool isLeaf() const { return left_right == 0; };
	};
	// TLAS
	class TLAS {
	public:
		TLAS() = default;

		void setMeshList(std::vector<Mesh>& src, bool is_gpu_available = false);

		// Does not clear every Mesh contained in the TLAS
		void clean(bool is_gpu_available = false);

		void build();
		__hostdev__ bool intersectTLAS(nanovdb::math::Ray<float> ray, hit_info& info);
		__hostdev__ bool intersectTLAS(nanovdb::math::Ray<float> ray, SceneHit& hinfo);
		__hostdev__ Mesh* getIntersectedMesh(nanovdb::math::Ray<float> ray);
		
	public:
		int findBestMatch(int* list, int N, int A);
		TLASNode* tlas_node_list = nullptr;
		Mesh* blas_list = nullptr;
		unsigned int node_used = 0, blas_count = 0;
	};
}
void penguinPT::TLAS::setMeshList(std::vector<Mesh>& src, bool is_gpu_available) {
	clean(is_gpu_available);

	blas_count = src.size();
	node_used = 2U;
	if (is_gpu_available) {
		cudaMallocManaged((void**)&blas_list, blas_count * sizeof(Mesh));
		cudaMallocManaged((void**)&tlas_node_list, blas_count * 2 * sizeof(TLASNode));
	}
	else {
		blas_list = (Mesh*)malloc(blas_count * sizeof(Mesh));
		tlas_node_list = (TLASNode*)malloc(blas_count * 2 * sizeof(TLASNode));
	}

	for (int i = 0; i < blas_count; i++) {
		blas_list[i] = src.at(i);
		math::copyMatrix(&src.at(i).transform_matrix, &blas_list[i].transform_matrix);
		math::copyMatrix(&src.at(i).transform_matrix_inverse, &blas_list[i].transform_matrix_inverse);
	}
}
void penguinPT::TLAS::clean(bool is_gpu_available) {
	if (node_used == 0) return;
	if (is_gpu_available) {
		CUDA_CHECK(cudaFree(tlas_node_list));
		CUDA_CHECK(cudaFree(blas_list));
	}
	else {
		free(tlas_node_list);
		free(blas_list);
	}
	tlas_node_list = nullptr;
	blas_list = nullptr;
	node_used = 0, blas_count = 0;
}
int penguinPT::TLAS::findBestMatch(int* list, int N, int A) {
	float smallest = MAX_DISTANCE;
	int bestB = -1;
	for (int B = 0; B < N; B++) if (B != A)
	{
		nanovdb::Vec3f b_max = MAX_3(tlas_node_list[list[A]].aabb_max, tlas_node_list[list[B]].aabb_max);
		nanovdb::Vec3f b_min = MIN_3(tlas_node_list[list[A]].aabb_min, tlas_node_list[list[B]].aabb_min);
		nanovdb::Vec3f e = b_max - b_min;
		float surface_area = e[0] * e[1] + e[1] * e[2] + e[2] * e[0];
		if (surface_area < smallest) smallest = surface_area, bestB = B;
	}
	return bestB;
}
void penguinPT::TLAS::build() {
	int* node_index = (int*)malloc(blas_count * sizeof(int));
	int node_indicies = blas_count;
	node_used = 1;
	for (unsigned int i = 0; i < blas_count; i++) {
		node_index[i] = node_used;
		tlas_node_list[node_used].aabb_min = blas_list[i].bounding_box.box_min;
		tlas_node_list[node_used].aabb_max = blas_list[i].bounding_box.box_max;
		tlas_node_list[node_used].BLAS = i;
		tlas_node_list[node_used++].left_right = 0;
	}

	int A = 0, B = findBestMatch(node_index, node_indicies, A);
	while (node_indicies > 1)
	{
		int C = findBestMatch(node_index, node_indicies, B);
		if (A == C)
		{
			int nodeIdxA = node_index[A], nodeIdxB = node_index[B];
			TLASNode& nodeA = tlas_node_list[nodeIdxA];
			TLASNode& nodeB = tlas_node_list[nodeIdxB];
			TLASNode& newNode = tlas_node_list[node_used];
			newNode.left_right = nodeIdxA + (nodeIdxB << 16);
			newNode.aabb_min = MIN_3(nodeA.aabb_min, nodeB.aabb_min);
			newNode.aabb_max = MAX_3(nodeA.aabb_max, nodeB.aabb_max);
			node_index[A] = node_used++;
			node_index[B] = node_index[node_indicies - 1];
			B = findBestMatch(node_index, --node_indicies, A);
		}
		else A = B, B = C;
	}
	tlas_node_list[0] = tlas_node_list[node_index[A]];

	free(node_index);
	std::cout << "Nodes used : " << node_used << "\n";
}
__hostdev__ bool penguinPT::TLAS::intersectTLAS(nanovdb::math::Ray<float> ray, hit_info& info) {
	if (blas_count == 0) return false;
	TLASNode* node = &tlas_node_list[0], *stack[64];
	unsigned int stackPtr = 0;
	bool hit = false;
	while (1)
	{
		if (node->isLeaf())
		{
			hit = hit || blas_list[node->BLAS].intersectMesh(ray, info);
				
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		TLASNode* child1 = &tlas_node_list[node->left_right & 0xffff];
		TLASNode* child2 = &tlas_node_list[node->left_right >> 16];
		float dist1 = boxIntersect_float(ray, child1->aabb_min, child1->aabb_max);
		float dist2 = boxIntersect_float(ray, child2->aabb_min, child2->aabb_max);
		if (dist1 > dist2) { 
			util::swapCUDA(dist1, dist2); 
			util::swapCUDA(child1, child2); 
		}
		if (dist1 >= MAX_DISTANCE)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = child1;
			if (dist2 <= MAX_DISTANCE) stack[stackPtr++] = child2;
		}
	}
	return hit;
	
	/*bool hit = false;
	for (int i = 0; i < blas_count; i++) {
		hit = hit || blas_list[i].intersectMesh(ray, info);
	}
	return hit;*/
}
__hostdev__ bool penguinPT::TLAS::intersectTLAS(nanovdb::math::Ray<float> ray, SceneHit& hinfo) {
	if (blas_count == 0) return false;
	TLASNode* node = &tlas_node_list[0], * stack[64];
	unsigned int stackPtr = 0;
	bool hit = false;
	while (1)
	{
		if (node->isLeaf())
		{
			hit = hit || blas_list[node->BLAS].intersectMesh(ray, hinfo);

			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		TLASNode* child1 = &tlas_node_list[node->left_right & 0xffff];
		TLASNode* child2 = &tlas_node_list[node->left_right >> 16];
		float dist1 = boxIntersect_float(ray, child1->aabb_min, child1->aabb_max);
		float dist2 = boxIntersect_float(ray, child2->aabb_min, child2->aabb_max);
		if (dist1 > dist2) {
			util::swapCUDA(dist1, dist2);
			util::swapCUDA(child1, child2);
		}
		if (dist1 >= MAX_DISTANCE)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = child1;
			if (dist2 <= MAX_DISTANCE) stack[stackPtr++] = child2;
		}
	}
	return hit;
}

__hostdev__ penguinPT::Mesh* penguinPT::TLAS::getIntersectedMesh(nanovdb::math::Ray<float> ray) {
	if (blas_count == 0) return nullptr;
	TLASNode* node = &tlas_node_list[0], *stack[TLAS_INTERSECTOR_STACK_SIZE];
	unsigned int stackPtr = 0;
	hit_info info;
	Mesh* ptr;
	while (1)
	{
		if (node->isLeaf())
		{
			if (blas_list[node->BLAS].intersectMesh(ray, info)) ptr = &blas_list[node->BLAS];

			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		TLASNode* child1 = &tlas_node_list[node->left_right & 0xffff];
		TLASNode* child2 = &tlas_node_list[node->left_right >> 16];
		float dist1 = boxIntersect_float(ray, child1->aabb_min, child1->aabb_max);
		float dist2 = boxIntersect_float(ray, child2->aabb_min, child2->aabb_max);
		if (dist1 > dist2) {
			float temp = dist1;
			dist1 = dist2;
			dist2 = temp;
			TLASNode* temp1 = child1;
			child1 = child2;
			child2 = temp1;
		}
		if (dist1 >= MAX_DISTANCE)
		{
			if (stackPtr == 0) break; else node = stack[--stackPtr];
		}
		else
		{
			node = child1;
			if (dist2 <= MAX_DISTANCE) stack[stackPtr++] = child2;
		}
	}
	return ptr;
}
