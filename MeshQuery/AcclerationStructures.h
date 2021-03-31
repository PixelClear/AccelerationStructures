#pragma once
#include <vector>
#include <memory>
#include <algorithm>

#include "Utility.h"

namespace MeshQuery
{

	struct OctreeNode
	{
		static const size_t NUM_CHILDREN = 8;
		static const size_t DEFAULT_DEPTH = 3;
		static const size_t DEFAULT_THRESHOLD = 100;

		AABB aabb_;
		size_t depth_;
		bool isLeaf_;
		OctreeNode* parent_;
		std::unique_ptr<OctreeNode> child_[NUM_CHILDREN];
		std::vector<Triangle> objectList_;
	};

	class AccelerationStructure
	{
	public:
		
		bool intersect(const Ray& r, const AABB& aabb) const;
		
		bool intersect(const Ray& r, const Triangle& triangle, float& t) const;
		
		inline bool intersect(const AABB &a, const AABB &b) const
		{
			if (a.max_.x < b.min_.x || a.min_.x > b.max_.x) return false; 
			if (a.max_.y < b.min_.y || a.min_.y > b.max_.y) return false;
			if (a.max_.z < b.min_.z || a.min_.z > b.max_.z) return false;
			
			return true;

		};

	};

	class NoAccelerationStructure : public AccelerationStructure
	{

	};

	class Octree : public AccelerationStructure
	{
	public:
		
		void buildTree(OctreeNode* node);

		void insertTriangle(OctreeNode* node, Triangle t);

		AABB GetOctaSplit(const AABB& B, size_t Idx)
		{
#define x0 B.min_.x
#define x1 B.max_.x
#define y0 B.min_.y
#define y1 B.max_.y
#define z0 B.min_.z
#define z1 B.max_.z
#define xc ((B.min_.x+B.max_.x) * 0.5f)
#define yc ((B.min_.y+B.max_.y) * 0.5f)
#define zc ((B.min_.z+B.max_.z) * 0.5f)
			switch (Idx & 7)
			{
			case 0: return AABB(glm::vec3(x0, yc, z0), glm::vec3(xc, y1, zc));
			case 1: return AABB(glm::vec3(xc, yc, z0), glm::vec3(x1, y1, zc));
			case 2: return AABB(glm::vec3(xc, y0, z0), glm::vec3(x1, yc, zc));
			case 3: return AABB(glm::vec3(x0, y0, z0), glm::vec3(xc, yc, zc));
			case 4: return AABB(glm::vec3(x0, yc, zc), glm::vec3(xc, y1, z1));
			case 5: return AABB(glm::vec3(xc, yc, zc), glm::vec3(x1, y1, z1));
			case 6: return AABB(glm::vec3(xc, y0, zc), glm::vec3(x1, yc, z1));
			case 7: return AABB(glm::vec3(x0, y0, zc), glm::vec3(xc, yc, z1));
			}

			return B;
#undef x0
#undef x1
#undef y0
#undef y1
#undef z0
#undef z1
#undef xc
#undef yc
#undef zc
		}
	};


	enum BvhStrategy
	{
		Sah,
		Hlbvh,
		Middle,
		EqualCountes
	};

	struct PrimitiveInfo
	{
		PrimitiveInfo() = default;
		PrimitiveInfo(size_t primNum, const AABB& aabb) :
		primNum_(primNum), aabb_(aabb), centroid_(0.5f * aabb_.min_ + 0.5f * aabb_.max_)
		{ }

		size_t primNum_;
		AABB aabb_;
		glm::vec3 centroid_;
	};

	inline glm::vec3 min(glm::vec3& a, glm::vec3& b) {
		return glm::vec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
	}

	inline glm::vec3 max(glm::vec3& a, glm::vec3& b) {
		return glm::vec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
	}

	inline glm::vec3 min(glm::vec3& a, glm::vec4& b) {
		return glm::vec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
	}

	inline glm::vec3 max(glm::vec3& a, glm::vec4& b) {
		return glm::vec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
	}

	inline AABB Union(AABB& a, AABB& b)
	{
		AABB c;
		c.min_ = min(a.min_, b.min_);
		c.max_ = max(a.max_, b.max_);
		return c;
	}

	inline AABB Union(AABB& a, glm::vec3& b)
	{
		AABB c;
		c.min_ = min(a.min_, b);
		c.max_ = max(a.max_, b);
		return c;
	}

	inline AABB Union(AABB& a, glm::vec4& b)
	{
		AABB c;
		c.min_ = min(a.min_, b);
		c.max_ = max(a.max_, b);
		return c;
	}

	struct BvhNode
	{
		BvhNode() = default;

		void initLeaf(int offset, int n, const AABB& b){
			firstPrimOffset_ = offset;
			nPrims_ = n;
			aabb_ = b;
			children_[0] = children_[1] = nullptr;
		}

		void initInterior(int axis, BvhNode* c0, BvhNode* c1){
			children_[0] = c0;
			children_[1] = c1;
			aabb_ = Union(c0->aabb_, c1->aabb_);
			splitAxis_ = axis;
			nPrims_ = 0;
		}

		AABB aabb_;
		BvhNode* children_[2];
		size_t splitAxis_;
		size_t firstPrimOffset_;
		size_t nPrims_;
	};

	class BVH : public AccelerationStructure
	{
	public:
		BVH() = delete;
		BVH(const std::vector<Triangle>& prims, BvhStrategy strategy);
		BvhNode* recursiveBuild(std::vector<PrimitiveInfo>& primInfo, int start, int end, int* totalNodes, std::vector<Triangle>& orderedPrims);
		BvhNode* root_;
	private:

		std::vector<Triangle> prims_;
		BvhStrategy strategy_;
		
	};
}

