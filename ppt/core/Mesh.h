// Copyright 2026 Toiture1234
// 
// SPDX-License-Identifier : MIT

#pragma once

#include <cassert>

#include <ppt/util/options.h>
#include <ppt/util/Utility.h>
#include <ppt/core/intersectors.h>

#include <ppt/util/Matrix.h>

namespace penguinPT {
	class BVH {
	public:
		BVH() :
			triangle_list(nullptr),
			triangle_data_list(nullptr),
			triangle_indicies_list(nullptr),
			node_list(nullptr),
			triangle_number(0),
			root_node_index(0),
			nodes_used(1) {
		}
		~BVH() {}

		void fillBVH(std::vector<Triangle>& src, std::vector<Triangle_data>& src_data, bool is_gpu_available = false);
		void clearBVH(bool is_gpu_available = false);
		void buildBVH();

		void zeroValues() {
			triangle_list = nullptr, triangle_data_list = nullptr, triangle_indicies_list = nullptr, triangle_number = 0, root_node_index = 0, nodes_used = 1, node_list = nullptr;
		}
	
		// intersect the BVH in BVH space
		__hostdev__ bool intersectBVH(nanovdb::math::Ray<float> ray, hit_info& info);

	private:
		// internal functions
		void updateNodesBounds(int node_idx);
		float findBestPlane(BVH_node& node, int& axis, float& split);
		float evalSAH(BVH_node& node, int axis, float pos);
		void subdivide(int node_idx);
	public:
		Triangle* triangle_list = nullptr;
		Triangle_data* triangle_data_list = nullptr;
		unsigned int* triangle_indicies_list = nullptr;

		unsigned int triangle_number = 0;

		BVH_node* node_list = nullptr;
		int root_node_index = 0;
		int nodes_used = 1;

	};

	class Mesh {
	public:
		Mesh(BVH* r_g) : raw_geometry(r_g) {}
		~Mesh() {}

		void setTransforms(math::Mat4f T);
		
		// Caution : this function may lead to incorrect results due to a scaling concern
		__hostdev__ bool intersectMesh(nanovdb::math::Ray<float> ray, hit_info& info);

		AABB bounding_box;
		BVH* raw_geometry;
	public:
		// Transforms
		math::Mat4f transform_matrix;
		math::Mat4f transform_matrix_inverse;
	};
}
void penguinPT::Mesh::setTransforms(math::Mat4f T) {
	transform_matrix = T;
	transform_matrix_inverse = T.inverse();
	bounding_box = AABB();

	if (raw_geometry->triangle_number == 0) {
		bounding_box.box_min = nanovdb::Vec3f(0.f);
		bounding_box.box_max = nanovdb::Vec3f(0.f);
	}
	else {
		nanovdb::Vec3f b_min = raw_geometry->node_list[0].boxMin, b_max = raw_geometry->node_list[0].boxMax;
		for (int i = 0; i < 8; i++) {
			bounding_box.grow(T.transformPoint(nanovdb::Vec3f(i & 1 ? b_max[0] : b_min[0],
				i & 2 ? b_max[1] : b_min[1], i & 4 ? b_max[2] : b_min[2])));
		}
	}
}
__hostdev__ bool penguinPT::Mesh::intersectMesh(nanovdb::math::Ray<float> ray, hit_info& info) {
	ray = transform_matrix_inverse.transformRay(ray);
	bool hit = raw_geometry->intersectBVH(ray, info);

	info.normal = transform_matrix.transformVector(info.normal).normalize();
	return hit;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void penguinPT::BVH::buildBVH() {
	if (triangle_number == 0) return;
	auto timer_start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < triangle_number; i++) {
		triangle_list[i].Origin = (triangle_list[i].A + triangle_list[i].B + triangle_list[i].C) * 0.3333f;
	}
	BVH_node& root = node_list[root_node_index];
	root.leftFirst = 0.;
	root.triangleCount = triangle_number;

	updateNodesBounds(root_node_index);
	subdivide(root_node_index);
	auto timer_end = std::chrono::high_resolution_clock::now();
	auto exe_duration = std::chrono::duration_cast<std::chrono::milliseconds>(timer_end - timer_start).count();

	printf("BVH created, took %lld milliseconds.\n", exe_duration);
	printf("BVH size : %i, Nodes used : %i\n", 2 * triangle_number - 1, nodes_used);
	std::cout << "-------------------------------------------------\n\n";
}
#define BIAS_GROW 0.01f
void penguinPT::BVH::updateNodesBounds(int node_idx) {
	BVH_node& node = node_list[node_idx];
	node.boxMin = nanovdb::Vec3f(1e30f);
	node.boxMax = nanovdb::Vec3f(-1e30f);
	for (int first = node.leftFirst, i = 0; i < node.triangleCount; i++) {
		int leafTriIdx = triangle_indicies_list[first + i];
		Triangle& leafTri = triangle_list[leafTriIdx];
		node.boxMin = node.boxMin.minComponent(leafTri.A - nanovdb::Vec3f(BIAS_GROW));
		node.boxMin = node.boxMin.minComponent(leafTri.B - nanovdb::Vec3f(BIAS_GROW));
		node.boxMin = node.boxMin.minComponent(leafTri.C - nanovdb::Vec3f(BIAS_GROW));
		node.boxMax = node.boxMax.maxComponent(leafTri.A + nanovdb::Vec3f(BIAS_GROW));
		node.boxMax = node.boxMax.maxComponent(leafTri.B + nanovdb::Vec3f(BIAS_GROW));
		node.boxMax = node.boxMax.maxComponent(leafTri.C + nanovdb::Vec3f(BIAS_GROW));
	}
}
#define NUM_TEST_SPLIT 50
float penguinPT::BVH::findBestPlane(BVH_node& node, int& axis, float& split) {
	float bCost = 1e30;
	for (int a = 0; a < 3; a++) {
		float bMin = 1e30f, bMax = -1e30f;

		// smaller box
		for (int i = 0; i < node.triangleCount; i++) {
			Triangle& tri = triangle_list[triangle_indicies_list[node.leftFirst + i]];

			bMin = std::min(bMin, tri.Origin[a]);
			bMax = std::max(bMax, tri.Origin[a]);
		}

		// bins
		BIN bin[NUM_TEST_SPLIT];
		float scale = NUM_TEST_SPLIT / (bMax - bMin);
		for (unsigned int i = 0; i < node.triangleCount; i++) {
			Triangle& tri = triangle_list[triangle_indicies_list[node.leftFirst + i]];
			int binIDX = CLAMP((int)((tri.Origin[a] - bMin) * scale), 0, NUM_TEST_SPLIT - 1);
			bin[binIDX].triangle_count++;
			bin[binIDX].bounds.grow(tri.A);
			bin[binIDX].bounds.grow(tri.B);
			bin[binIDX].bounds.grow(tri.C);
		}

		float leftArea[NUM_TEST_SPLIT - 1], rightArea[NUM_TEST_SPLIT - 1];
		int leftCount[NUM_TEST_SPLIT - 1], rightCount[NUM_TEST_SPLIT - 1];
		AABB leftBox, rightBox;
		int leftSum = 0, rightSum = 0;
		for (int i = 0; i < NUM_TEST_SPLIT - 1; i++)
		{
			leftSum += bin[i].triangle_count;
			leftCount[i] = leftSum;
			leftBox.grow(bin[i].bounds);
			leftArea[i] = leftBox.area();

			rightSum += bin[NUM_TEST_SPLIT - 1 - i].triangle_count;
			rightCount[NUM_TEST_SPLIT - 2 - i] = rightSum;
			rightBox.grow(bin[NUM_TEST_SPLIT - 1 - i].bounds);
			rightArea[NUM_TEST_SPLIT - 2 - i] = rightBox.area();
		}

		scale = (float)(bMax - bMin) / (float)NUM_TEST_SPLIT;
		for (int i = 0; i < NUM_TEST_SPLIT - 1; i++)
		{
			float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
			if (planeCost < bCost) {
				axis = a, split = bMin + scale * (i + 1), bCost = planeCost;
			}
		}
	}
	return bCost;
}
float penguinPT::BVH::evalSAH(BVH_node& node, int axis, float pos) {
	AABB leftB, rightB;
	int leftCount = 0, rightCount = 0;
	for (unsigned int i = 0; i < node.triangleCount; i++) {
		Triangle& tri = triangle_list[triangle_indicies_list[node.leftFirst + i]];
		if (tri.Origin[axis] < pos) {
			leftCount++;
			leftB.grow(tri.A);
			leftB.grow(tri.B);
			leftB.grow(tri.C);
		}
		else {
			rightCount++;
			rightB.grow(tri.A);
			rightB.grow(tri.B);
			rightB.grow(tri.C);
		}
	}
	float cost = leftCount * leftB.area() + rightCount * rightB.area();
	return cost > 0.f ? cost : 1e30f;
}
void penguinPT::BVH::subdivide(int node_idx) {
	BVH_node& node = node_list[node_idx];

	nanovdb::Vec3f e = node.boxMax - node.boxMin;
	float parentArea = e[0] * e[1] + e[1] * e[2] + e[2] * e[0];
	float parentCost = node.triangleCount * parentArea;

	int axis;
	float splitPos;
	float splitCost = findBestPlane(node, axis, splitPos);
	if (splitCost >= parentCost) return;

	int i = node.leftFirst;
	int j = i + node.triangleCount - 1;
	while (i <= j)
	{
		if (triangle_list[triangle_indicies_list[i]].Origin[axis] < splitPos) i++;
		else {
			std::swap(triangle_indicies_list[i], triangle_indicies_list[j--]);
		}
	}

	int leftCount = i - node.leftFirst;
	if (leftCount == 0 || leftCount == node.triangleCount) return;
	int leftChildIdx = nodes_used++;
	int rightChildIdx = nodes_used++;

	node_list[leftChildIdx].leftFirst = node.leftFirst;
	node_list[leftChildIdx].triangleCount = leftCount;
	node_list[rightChildIdx].leftFirst = i;
	node_list[rightChildIdx].triangleCount = node.triangleCount - leftCount;
	node.leftFirst = leftChildIdx;
	node.triangleCount = 0;

	updateNodesBounds(leftChildIdx);
	updateNodesBounds(rightChildIdx);

	subdivide(leftChildIdx);
	subdivide(rightChildIdx);
}
void penguinPT::BVH::clearBVH(bool is_gpu_available) {
	if (triangle_number != 0) {
		if (is_gpu_available) {
			CUDA_CHECK(cudaFree(triangle_list));
			CUDA_CHECK(cudaFree(triangle_data_list));
			CUDA_CHECK(cudaFree(triangle_indicies_list));
			CUDA_CHECK(cudaFree(node_list));
		}
		else {
			free(triangle_list);
			free(triangle_data_list);
			free(triangle_indicies_list);
			free(node_list);
		}
		triangle_number = 0;
		nodes_used = 1;
		root_node_index = 0;

		triangle_list = nullptr;
		triangle_data_list = nullptr;
		triangle_indicies_list = nullptr;
		node_list = nullptr;
	}
}
void penguinPT::BVH::fillBVH(std::vector<Triangle>& src, std::vector<Triangle_data>& src_data, bool is_gpu_available) {
	clearBVH(is_gpu_available);
	triangle_number = src.size();

	if (triangle_number == 0) return;

	if (is_gpu_available) {
		CUDA_CHECK(cudaMallocManaged((void**)&triangle_list, triangle_number * sizeof(Triangle)));
		CUDA_CHECK(cudaMallocManaged((void**)&triangle_data_list, triangle_number * sizeof(Triangle_data)));
		CUDA_CHECK(cudaMallocManaged((void**)&triangle_indicies_list, triangle_number * sizeof(unsigned int)));
		CUDA_CHECK(cudaMallocManaged((void**)&node_list, (2 * triangle_number - 1) * sizeof(BVH_node)));
	}
	else {
		triangle_list = (Triangle*)malloc(triangle_number * sizeof(Triangle));
		triangle_data_list = (Triangle_data*)malloc(triangle_number * sizeof(Triangle_data));
		triangle_indicies_list = (unsigned int*)malloc(triangle_number * sizeof(unsigned int));
		node_list = (BVH_node*)malloc((static_cast<unsigned long long>(2) * triangle_number - 1) * sizeof(BVH_node));
	}

	for (int i = 0; i < triangle_number; i++) {
		triangle_list[i] = src.at(i);
		triangle_data_list[i] = src_data.at(i);
		triangle_indicies_list[i] = i;
	}
}
__hostdev__ bool penguinPT::BVH::intersectBVH(nanovdb::math::Ray<float> ray, hit_info& info)
{
	// intersect BVH
	int stack[BVH_INTERSECTOR_STACK_SIZE];
	int stackIdx = 0;
	stack[stackIdx++] = 0;

	float t = 1e30f;
	bool hit = false;

	if (triangle_number == 0) return false;

	float2 uv = make_float2(0.f, 0.f);
	unsigned int index = 0;

	while (stackIdx > 0)
	{
		BVH_node node = node_list[stack[--stackIdx]];
		if (boxIntersect_float(ray, node.boxMin, node.boxMax) < t) {
			if (node.triangleCount > 0) { // leaf node
				for (int i = 0; i < node.triangleCount; i++) { // leaf node
#if MODE_TRIANGLE == 0
					if (intersect_triangle_replace(ray, info.normal, info.trueNormal, triangle_list[triangle_indicies_list[i + node.leftFirst]], info.uv, info.t, info.BSDF_index)) {
						info.debug[0]++;
						hit = true;
					}
#else 
					if (intersect_triangle_uv(ray, triangle_list[triangle_indicies_list[i + node.leftFirst]], uv, info.t)) {
						info.debug[0]++;
						hit = true;
						index = triangle_indicies_list[i + node.leftFirst];
					}
#endif
				}
			}
			else {
				BVH_node childLeft = node_list[node.leftFirst];
				BVH_node childRight = node_list[node.leftFirst + 1];

				float dstLeft = boxIntersect_float(ray, childLeft.boxMin, childLeft.boxMax);
				float dstRight = boxIntersect_float(ray, childRight.boxMin, childRight.boxMax);
				int left = node.leftFirst, right = node.leftFirst + 1;

				if (dstLeft > dstRight) {
					if (dstLeft < info.t) stack[stackIdx++] = left;
					if (dstRight < info.t) stack[stackIdx++] = right;
				}
				else {
					if (dstRight < info.t) stack[stackIdx++] = right;
					if (dstLeft < info.t) stack[stackIdx++] = left;
				}

			}
			info.debug[2]++;
		}
	}
#if MODE_TRIANGLE != 0
	if (hit) {
		Triangle& intersected = triangle_list[index];
		Triangle_data& intersected_data = triangle_data_list[index];
		float u = uv.x;
		float v = uv.y;
		float w = 1.f - u - v;

		info.normal = (intersected.B - intersected.A).cross(intersected.C - intersected.A).normalize();
		float u_dot = info.normal.dot(ray.dir());
		float mult = -SIGN(u_dot);
		info.normal = (intersected_data.nA.dot(intersected_data.nA) > 0.f ? (intersected_data.nA * w + intersected_data.nB * u + intersected_data.nC * v).normalize() : info.normal) * mult;
		info.BSDF_index = intersected_data.BSDF_index;

		// blend info.uv between everything
		info.uv = make_float2(intersected_data.uvA.x * w + intersected_data.uvB.x * u + intersected_data.uvC.x * v,
			intersected_data.uvA.y * w + intersected_data.uvB.y * u + intersected_data.uvC.y * v);
	}
#endif

	return hit;
}