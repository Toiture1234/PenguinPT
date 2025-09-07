#pragma once

#define BASE_BSDF 1
#define BSDF_TROUGH_ID 0

namespace penguinPT {

	// simple triangle class, maybe would grow up later with normals and textures
	class Triangle {
	public:
		nanovdb::Vec3f A, B, C, Origin;
		nanovdb::Vec3f nA, nB, nC;
		unsigned int BSDF_index;
		__hostdev__ Triangle() : A(0.f), B(0.f), C(0.f), Origin(0.f), nA(0.f), nB(0.f), nC(0.f), BSDF_index(BASE_BSDF) {}
		__hostdev__ Triangle(nanovdb::Vec3f a, nanovdb::Vec3f b, nanovdb::Vec3f c) : A(a), B(b), C(c), Origin(0.f), nA(0.f), nB(0.f), nC(0.f), BSDF_index(BASE_BSDF) {}
		__hostdev__ ~Triangle(){}
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
		void grow(nanovdb::Vec3f p) {
			box_min = MIN_3(box_min, p), box_max = MAX_3(box_max, p);
		}
		float area() {
			nanovdb::Vec3f e = box_max - box_min;
			return e[0] * e[1] + e[1] * e[2] + e[2] * e[0];
		}

		__hostdev__ AABB() {}
		__hostdev__ ~AABB() {}
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
		normal = tri.nA.dot(tri.nA) > 0.f ? (tri.nA * w + tri.nB * u + tri.nC * v).normalize() : normal;
		return t;
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
		if (u < 0.0f || v < 0.0f || (u + v) > 1.0f || t > t_out || t < 0.f) return false;
		float w = 1.f - u - v;

		uv = make_float2(u, v);
		trueNormal = (tri.B - tri.A).cross(tri.C - tri.A).normalize();
		float u_dot = trueNormal.dot(ray.dir());
		float mult = -SIGN(u_dot);
		normal = (tri.nA.dot(tri.nA) > 0.f ? (tri.nA * w + tri.nB * u + tri.nC * v).normalize() : trueNormal) * mult;
		t_out = t;
		BSDF_i = tri.BSDF_index;
		return true;
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

	// Scene class, contains an array of triangles and support for BVH, be carefull if you use the BVH to have initialized triangles
	class Scene_data {
	public:
		__hostdev__ Scene_data() : triangles(nullptr), 
			num_of_triangles(0), 
			triangle_indicies(nullptr), 
			nodes(nullptr), 
			root_node_idx(0), 
			nodes_used(1), 
			bsdf_list(nullptr), 
			volumes(nullptr), 
			num_of_volumes(0) {}

		__hostdev__ Scene_data(int n_tr) : num_of_triangles(n_tr), 
			root_node_idx(0), 
			nodes_used(1), 
			bsdf_list(nullptr), 
			volumes(nullptr), 
			num_of_volumes(0) 
		{
			triangles = (Triangle*)malloc(n_tr * sizeof(Triangle));
			triangle_indicies = (unsigned int*)malloc(n_tr * sizeof(unsigned int));
			nodes = (BVH_node*)malloc((2 * n_tr - 1) * sizeof(BVH_node)); // maybe too much, to rewind when making BVH
		}
		__hostdev__ ~Scene_data() {}

		// list of triangles in this scene
		Triangle* triangles;
		unsigned int num_of_triangles;

		// BVH
		unsigned int* triangle_indicies;
		BVH_node* nodes;
		int root_node_idx, nodes_used;

		// list of volumes in this scene
		Volume* volumes;
		int num_of_volumes;

		// volumes BVH ? 

		// BSDF
		BSDF* bsdf_list;

		// CUDA transfer to GPU memory, only transfer BVH and triangles information 
		void send_to_gpu_solid() {
			// create temporary buffers
			Triangle* temp_triangles = triangles;
			BVH_node* temp_nodes = nodes;
			unsigned int* temp_triangle_indicies = triangle_indicies;

			// GPU memory allocation
			CUDA_CHECK(cudaMalloc((void**)&triangles, num_of_triangles * sizeof(Triangle)));
			CUDA_CHECK(cudaMalloc((void**)&nodes, (2 * num_of_triangles - 1) * sizeof(BVH_node)));
			CUDA_CHECK(cudaMalloc((void**)&triangle_indicies, num_of_triangles * sizeof(unsigned int)));
			
			// Send data to gpu
			CUDA_CHECK(cudaMemcpy(triangles, temp_triangles, num_of_triangles * sizeof(Triangle), cudaMemcpyHostToDevice));
			CUDA_CHECK(cudaMemcpy(nodes, temp_nodes, (2 * num_of_triangles - 1) * sizeof(BVH_node), cudaMemcpyHostToDevice));
			CUDA_CHECK(cudaMemcpy(triangle_indicies, temp_triangle_indicies, num_of_triangles * sizeof(unsigned int), cudaMemcpyHostToDevice));
			
			// delete temporary buffers
			free(temp_triangles);
			free(temp_nodes);
			free(temp_triangle_indicies);
		}
		void send_to_gpu_volumes() {
			Volume* temp_volumes = volumes;

			cudaMalloc((void**)&volumes, num_of_volumes * sizeof(Volume));
			cudaMemcpy(volumes, temp_volumes, num_of_volumes * sizeof(Volume), cudaMemcpyHostToDevice);
			free(temp_volumes);
		}

		// create BVH
		void build_BVH();
		void update_nodes_bounds(int node_idx);
		float find_best_plane(BVH_node& node, int& axis, float& split);
		float eval_SAH(BVH_node& node, int axis, float pos);
		void subdivide(int node_idx);

		// create volumes
		void empty_volumes(int v_num);

		// surface only
		__hostdev__ bool intersectScene_BVH(nanovdb::math::Ray<float> ray, hit_info& info);
		__hostdev__ bool intersectScene_noBVH(nanovdb::math::Ray<float> ray, hit_info& info);

		// volumes only
		__hostdev__ bool intersectScene_volumes(nanovdb::math::Ray<float> ray, hit_info& info);
		// complete scene data intersector
		__hostdev__ bool intersectScene_full(nanovdb::math::Ray<float> ray, hit_info& info);
	};

	__hostdev__ bool Scene_data::intersectScene_noBVH(nanovdb::math::Ray<float> ray, hit_info& info) {
		bool hit = false;
		for (unsigned int i = 0; i < num_of_triangles; i++) {
			if (intersect_triangle_replace(ray, info.normal, info.trueNormal, triangles[i], info.uv, info.t, info.BSDF_index)) {
				hit = true;
				info.debug[0]++;
			}
		}
		return hit;
	}

	// --------------------------------------------------------------------------------
	//                                  BVH Building
	// --------------------------------------------------------------------------------
	void Scene_data::build_BVH() {
		for (int i = 0; i < num_of_triangles; i++) {
			triangles[i].Origin = (triangles[i].A + triangles[i].B + triangles[i].C) * 0.3333f;
		}
		BVH_node& root = nodes[root_node_idx];
		root.leftFirst = 0.;
		root.triangleCount = num_of_triangles;

		update_nodes_bounds(root_node_idx);
		subdivide(root_node_idx);

		printf("BVH size : %i, Nodes used : %i\n", 2 * num_of_triangles - 1, nodes_used);
	}
#define BIAS_GROW 0.01f
	void Scene_data::update_nodes_bounds(int node_idx) {
		BVH_node& node = nodes[node_idx];
		node.boxMin = nanovdb::Vec3f(1e30f);
		node.boxMax = nanovdb::Vec3f(-1e30f);
		for (int first = node.leftFirst, i = 0; i < node.triangleCount; i++) {
			int leafTriIdx = triangle_indicies[first + i];
			Triangle& leafTri = triangles[leafTriIdx];
			node.boxMin = MIN_3(node.boxMin, (leafTri.A - nanovdb::Vec3f(BIAS_GROW)));
			node.boxMin = MIN_3(node.boxMin, (leafTri.B - nanovdb::Vec3f(BIAS_GROW)));
			node.boxMin = MIN_3(node.boxMin, (leafTri.C - nanovdb::Vec3f(BIAS_GROW)));
			node.boxMax = MAX_3(node.boxMax, (leafTri.A + nanovdb::Vec3f(BIAS_GROW)));
			node.boxMax = MAX_3(node.boxMax, (leafTri.B + nanovdb::Vec3f(BIAS_GROW)));
			node.boxMax = MAX_3(node.boxMax, (leafTri.C + nanovdb::Vec3f(BIAS_GROW)));
		}
	}
#define NUM_TEST_SPLIT 500
	float Scene_data::find_best_plane(BVH_node& node, int& axis, float& split) {
		float bCost = 1e30;
		for (int a = 0; a < 3; a++) {
			float bMin = node.boxMin[a];
			float bMax = node.boxMax[a];

			if (bMin == bMax) continue;
			float scale = (bMax - bMin) / (float)NUM_TEST_SPLIT;
			for (int i = 1; i < NUM_TEST_SPLIT; i++) {
				float cP = bMin + i * scale;
				float cost = eval_SAH(node, a, cP);
				if (cost < bCost)
					split = cP, axis = a, bCost = cost;
			}
		}
		return bCost;
	}
	float Scene_data::eval_SAH(BVH_node& node, int axis, float pos) {
		AABB leftB, rightB;
		int leftCount = 0, rightCount = 0;
		for (unsigned int i = 0; i < node.triangleCount; i++) {
			Triangle& tri = triangles[triangle_indicies[node.leftFirst + i]];
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
	void Scene_data::subdivide(int node_idx) {
		BVH_node& node = nodes[node_idx];

		nanovdb::Vec3f e = node.boxMax - node.boxMin;
		float parentArea = e[0] * e[1] + e[1] * e[2] + e[2] * e[0];
		float parentCost = node.triangleCount * parentArea;

		int axis;
		float splitPos;
		float splitCost = find_best_plane(node, axis, splitPos);
		if (splitCost >= parentCost) return;

		int i = node.leftFirst;
		int j = i + node.triangleCount - 1;
		while (i <= j)
		{
			if (triangles[triangle_indicies[i]].Origin[axis] < splitPos) i++;
			else std::swap(triangle_indicies[i], triangle_indicies[j--]);
		}

		int leftCount = i - node.leftFirst;
		if (leftCount == 0 || leftCount == node.triangleCount) return;
		int leftChildIdx = nodes_used++;
		int rightChildIdx = nodes_used++;

		nodes[leftChildIdx].leftFirst = node.leftFirst;
		nodes[leftChildIdx].triangleCount = leftCount;
		nodes[rightChildIdx].leftFirst = i;
		nodes[rightChildIdx].triangleCount = node.triangleCount - leftCount;
		node.leftFirst = leftChildIdx;
		node.triangleCount = 0;

		update_nodes_bounds(leftChildIdx);
		update_nodes_bounds(rightChildIdx);
		
		subdivide(leftChildIdx);
		subdivide(rightChildIdx);
	}

	// --------------------------------------------------------------------------------
	//                                BVH Intersection
	// --------------------------------------------------------------------------------
	// scene intersection with BVH, only intersects with solid triangles
	__hostdev__ bool Scene_data::intersectScene_BVH(nanovdb::math::Ray<float> ray, hit_info& info) {
		int stack[32];
		int stackIdx = 0;
		stack[stackIdx++] = 0;

		float t = 1e30f;
		bool hit = false;

		while (stackIdx > 0)
		{
			BVH_node node = nodes[stack[--stackIdx]];
			if (boxIntersect_float(ray, node.boxMin, node.boxMax) < t) {
				if (node.triangleCount > 0) { // leaf node
					for (int i = 0; i < node.triangleCount; i++) { // leaf node
						if (intersect_triangle_replace(ray, info.normal, info.trueNormal, triangles[triangle_indicies[i + node.leftFirst]], info.uv, info.t, info.BSDF_index)) {
							info.debug[0]++;
							hit = true;
						}
					}
				}
				else {
					BVH_node childLeft = nodes[node.leftFirst];
					BVH_node childRight = nodes[node.leftFirst + 1];

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
		return hit;
	}
	__hostdev__ bool Scene_data::intersectScene_volumes(nanovdb::math::Ray<float> ray, hit_info& info) {
		bool hit = false;

		// basic iterator loop, no BVH now, maybe later
		for (unsigned int i = 0; i < num_of_volumes; i++) {
			if (volumes[i].intersect_volume_replace(ray, info.t, info.normal, info.insideVolume)) { 
				hit = true;
				info.BSDF_index = BSDF_TROUGH_ID;
				info.volumeIndex = i; // this works now but need to be changed later when multiple volumes
			}
		}
		return hit;
	}
	__hostdev__ bool Scene_data::intersectScene_full(nanovdb::math::Ray<float> ray, hit_info& info) {
		bool hit = intersectScene_BVH(ray, info);

		// volumes intersection
		if (intersectScene_volumes(ray, info)) hit = true;
		return hit;
	}

	void Scene_data::empty_volumes(int v_num) {
		if (num_of_volumes != 0) free(volumes);

		volumes = (Volume*)malloc(v_num * sizeof(Volume));
		num_of_volumes = v_num;
	}

}