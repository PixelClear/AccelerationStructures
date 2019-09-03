#pragma once
#include <glm/glm.hpp>
#include <vector>

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

		glm::vec3 min_;
		glm::vec3 max_;
	};

	struct Triangle
	{
		glm::vec3 vertices_[3];
		glm::vec3 normal_[3];
		glm::vec3 uv_[3];
	};

	struct Mesh
	{
		std::vector<float> vertices_;
		std::vector<float> normals_;
		std::vector<uint32_t> faces_;
		std::vector<Triangle> triangles_;

		uint32_t indexVbo_;
		uint32_t vertexVbo_;
		uint32_t normalVbo_;

		AABB aabb_;
	}mesh;

	class AccelerationStructure
	{
	public:
		

		bool intersect(const Ray& r, const AABB& aabb) const
		{
			float tMin = (aabb.min_.x - r.origin_.x) / r.direction_.x;
			float tMax = (aabb.max_.x - r.origin_.x) / r.direction_.x;

			if (tMin > tMax)
			{
				std::swap(tMin, tMax);
			}

			float tyMin = (aabb.min_.y - r.origin_.y) / r.direction_.y;
			float tyMax = (aabb.max_.y - r.origin_.y) / r.direction_.y;

			if (tyMin > tyMax)
			{
				std::swap(tyMin, tyMax);
			}

			if ((tMin > tyMax) || (tyMin > tMax))
				return false;

			if (tyMin > tMin)
				tMin = tyMin;

			if (tyMax < tMax)
				tMax = tyMax;

			float tzMin = (aabb.min_.z - r.origin_.z) / r.direction_.z;
			float tzMax = (aabb.max_.z - r.origin_.z) / r.direction_.z;

			if (tzMin > tzMax)
			{
				std::swap(tzMin, tzMax);
			}

			if ((tMin > tzMax) || (tzMin > tMax))
				return false;

			if (tzMin > tMin)
				tMin = tzMin;

			if (tzMax < tMax)
				tMax = tzMax;

			return true;
		}

		bool intersect(const Ray& r, const Triangle& triangle, float& t) const
		{
			const glm::vec3 v0v1 = triangle.vertices_[1] - triangle.vertices_[0];
			const glm::vec3 v0v2 = triangle.vertices_[2] - triangle.vertices_[0];
			glm::vec3 pVec = glm::cross(r.direction_, v0v2);
			const float det = glm::dot(v0v1, pVec); 

			if (std::fabs(det) < kEpsilon)
			{
				return false;
			}

			const float invDet = 1.0f / det;

			const glm::vec3 tVec = r.origin_ - triangle.vertices_[0];
			const float u = glm::dot(tVec,pVec) * invDet;
			
			if (u < 0.0f || u > 1.0f)
			{
				return false;
			}

			const glm::vec3 qVec = glm::cross(tVec, v0v1);
			const float v =  glm::dot(r.direction_, qVec) * invDet;
			
			if (v < 0.0f || u + v > 1.0f)
			{
				return false;
			}

			t = glm::dot(v0v2, qVec) * invDet;

			return true;
		}
	};
}

