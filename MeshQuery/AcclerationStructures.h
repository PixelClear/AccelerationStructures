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


		//Get closest point on triangle
		void getClosestPoint(const glm::vec3 p, const Triangle t, glm::vec3& closestPoint) const;

		//Get closest point on AABB
		void getClosestPoint(const glm::vec3 p, const AABB t, glm::vec3& closestPoint) const;

		//Find closesd point on to p on all triangles and distance to that point 
		std::pair<glm::vec3, float> getClosestPoint(const glm::vec3 p, const std::vector<Triangle> triList) const;

		//Find closesd point to AABB and distance to that point
		std::pair<glm::vec3, float> getClosestPoint(const glm::vec3 p, OctreeNode* root) const;

		float SqDistPointAABB(glm::vec3 p, AABB b) const;
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
}

