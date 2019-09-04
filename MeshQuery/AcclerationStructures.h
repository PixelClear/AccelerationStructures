#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include <algorithm>

namespace MeshQuery
{
	constexpr float kEpsilon = 1e-8;
	
	class Ray
	{
	public:
		Ray() = default;

		Ray(const glm::vec3& o, const glm::vec3& d) : origin_(o), direction_(d)
		{

		}

		Ray(const glm::vec3& rhs)
		{
			*this = rhs;
		}

		Ray(glm::vec3&& rhs)
		{
			*this = std::move(rhs);
		}

		Ray& operator=(const Ray& rhs)
		{
			*this = rhs;
		}

		Ray& operator=(Ray&& rhs)
		{
			*this = std::move(rhs);
		}

		~Ray() = default;

		glm::vec3 pointAtParameter(float t) const
		{
			return origin_ + t * direction_;
		}

		glm::vec3 origin_;
		glm::vec3 direction_;
	};

	class AABB
	{
	public:
		AABB() = default;
		AABB(const glm::vec3 min, const glm::vec3 max) : min_(min), max_(max) {}
		AABB(const AABB& rhs)
		{
			min_ = rhs.min_;
			max_ = rhs.max_;
		}

		AABB(AABB&& rhs)
		{
			min_ = std::move(rhs.min_);
			max_ = std::move(rhs.max_);
		}

		AABB& operator=(const AABB& rhs)
		{
			min_ = rhs.min_;
			max_ = rhs.max_;
			return *this;
		}

		AABB& operator=(AABB&& rhs)
		{
			min_ = std::move(rhs.min_);
			max_ = std::move(rhs.max_);
			return *this;
		}

		~AABB() = default;

		AABB& extendBy(const glm::vec3& p)
		{
			if (p.x < min_.x) min_.x = p.x;
			if (p.y < min_.y) min_.y = p.y;
			if (p.z < min_.z) min_.z = p.z;
			if (p.x > max_.x) max_.x = p.x;
			if (p.y > max_.y) max_.y = p.y;
			if (p.z > max_.z) max_.z = p.z;

			return *this;
		}

		glm::vec3 min_;
		glm::vec3 max_;
	};

	struct Triangle
	{
		glm::vec3 vertices_[3];
		glm::vec3 normal_[3];
		AABB aabb_;

		bool operator==(const Triangle& rhs)
		{
			return vertices_[0] == rhs.vertices_[0] && vertices_[1] == rhs.vertices_[1] && vertices_[2] == rhs.vertices_[2];
		}
	};

	class AccelerationStructure
	{
	public:
		
		bool intersect(const Ray& r, const AABB& aabb) const;
		bool intersect(const Ray& r, const Triangle& triangle, float& t) const;
		bool intersect(const AABB &a, const AABB &b)
		{
			if (a.max_.x < b.min_.x || a.min_.x > b.max_.x) return false; 
			if (a.max_.y < b.min_.y || a.min_.y > b.max_.y) return false;
			if (a.max_.z < b.min_.z || a.min_.z > b.max_.z) return false;
			
			return true;

		};
	};

	class NoAcclerationStructure : public AccelerationStructure
	{
	public:

	    //Get closest point on triangle
		void getClosestPointOnTri(glm::vec3 p, Triangle t, glm::vec3& closestPoint);

		//Find closesd point on to p on all triangles and distance to that point to p
		float findMinDistance(glm::vec3 p,  const std::vector<Triangle> triList);
	};

	struct OctreeNode :public AccelerationStructure
	{
		static const size_t NUM_CHILDREN = 8;
		static const size_t DEFAULT_DEPTH = 1;
		static const size_t DEFAULT_THRESHOLD = 100;

		AABB aabb_;
		int depth_;
		bool isLeaf_;
		OctreeNode* parent_;
		std::unique_ptr<OctreeNode> child_[NUM_CHILDREN];
		std::vector<Triangle> objectList_;
	};

	class Octree : public OctreeNode
	{
	public:
		
		void buildTree(OctreeNode* node)
		{
			for (size_t i = 0; i != NUM_CHILDREN; i++)
			{
				node->child_[i] = nullptr;
			}

			if (node->depth_ >= DEFAULT_DEPTH)
			{
				return;
			}

			for (size_t i = 0; i != NUM_CHILDREN; i++)
			{
				node->child_[i] = std::make_unique<OctreeNode>();
				const AABB ChildBox = GetOctaSplit(node->aabb_, i);

				node->child_[i]->parent_ = node;
				node->child_[i]->aabb_ = ChildBox;
				node->child_[i]->depth_ = node->depth_ + 1;

				buildTree(node->child_[i].get());
			}
		}

		void insertTriangle(OctreeNode* node, Triangle t)
		{
			if (node == nullptr)
			{
				return;
			}

			if (node->depth_ >= DEFAULT_DEPTH)
			{
				return;
			}

			for (size_t i = 0; i != NUM_CHILDREN; i++)
			{
				if (intersect(t.aabb_, node->child_[i]->aabb_))
				{
					node->child_[i]->objectList_.push_back(t);

					if(!node->objectList_.empty())
					   node->objectList_.erase(std::remove_if(node->objectList_.begin(), node->objectList_.end() - 1, [t](auto temp) { return temp == t; }));

					insertTriangle(node->child_[i].get(), t);
				}
			}
		}

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

