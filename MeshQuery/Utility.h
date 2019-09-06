#pragma once
#include <glm/glm.hpp>

namespace MeshQuery
{
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
}
