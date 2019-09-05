#include "AcclerationStructures.h"
#include <algorithm>

bool MeshQuery::AccelerationStructure::intersect(const Ray & r, const AABB & aabb) const
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

bool MeshQuery::AccelerationStructure::intersect(const Ray & r, const Triangle & triangle, float & t) const
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
	const float u = glm::dot(tVec, pVec) * invDet;

	if (u < 0.0f || u > 1.0f)
	{
		return false;
	}

	const glm::vec3 qVec = glm::cross(tVec, v0v1);
	const float v = glm::dot(r.direction_, qVec) * invDet;

	if (v < 0.0f || u + v > 1.0f)
	{
		return false;
	}

	t = glm::dot(v0v2, qVec) * invDet;

	return true;
}

void MeshQuery::NoAcclerationStructure::getClosestPoint(glm::vec3 p, Triangle t, glm::vec3& closestPoint)
{
	glm::vec3 ab = t.vertices_[1] - t.vertices_[0];
	glm::vec3 ac = t.vertices_[2] - t.vertices_[0];

	glm::vec3 ap = p - t.vertices_[0];
	float d1 = glm::dot(ab, ap);
	float d2 = glm::dot(ac, ap);

	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		closestPoint = t.vertices_[0];
		return;
	}

	glm::vec3 bp = p - t.vertices_[1];
	float d3 = glm::dot(ab, bp);
	float d4 = glm::dot(ac, bp);

	if (d3 >= 0.0f && d4 <= d3)
	{
		closestPoint = t.vertices_[1];
		return;
	}

	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		float v = d1 / (d1 - d3);
		closestPoint = t.vertices_[0] + v * ab;
		return;
	}

	glm::vec3 cp = p - t.vertices_[2];
	float d5 = glm::dot(ab, cp);
	float d6 = glm::dot(ac, cp);

	if (d6 >= 0.0f && d5 <= d6)
	{
		closestPoint = t.vertices_[2];
		return;
	}

	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		float w = d2 / (d2 - d6);
		closestPoint = t.vertices_[0] + w * ac;
		return;
	}

	float va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closestPoint = t.vertices_[1] + w * (t.vertices_[2] - t.vertices_[1]);
		return;
	}

	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	closestPoint = t.vertices_[0] + ab * v + ac * w;
	return;
}

void MeshQuery::NoAcclerationStructure::getClosestPoint(glm::vec3 p, const AABB aabb, glm::vec3 & closestPoint)
{
	for (int i = 0; i < 3; i++)
	{
		float v = p[i];
		
		const auto x = aabb.min_[i];
		const auto y = aabb.max_[i];

		if (v < aabb.min_[i])
		{
			v = aabb.min_[i];
		}
		if (v > aabb.max_[i])
		{
			v = aabb.max_[i];
		}

		closestPoint[i] = v;
	}
}

float MeshQuery::NoAcclerationStructure::findMinDistance(glm::vec3 p, const std::vector<Triangle> triList)
{
	std::vector<float> distance;
	glm::vec3 closestPoint;

	for (auto t : triList)
	{
		getClosestPoint(p, t, closestPoint);
		distance.push_back(glm::distance(p, closestPoint));
	}

	return *std::min_element(distance.begin(), distance.end(), [](float a, float b) {return (a < b); });
}

float MeshQuery::NoAcclerationStructure::SqDistPointAABB(glm::vec3 p, AABB b)
{
	float sqDist = 0.0f;

	for (int i = 0; i < 3; i++) 
	{ 
		// For each axis count any excess distance outside box extents 
		float v = p[i]; 
		if (v < b.min_[i]) sqDist += (b.min_[i] - v) * (b.min_[i] - v); 
		if (v > b.max_[i]) sqDist += (v - b.max_[i]) * (v - b.max_[i]); 
	}

	return sqDist;
}

float MeshQuery::NoAcclerationStructure::findMinDistance(glm::vec3 p, OctreeNode * root)
{
	OctreeNode* node = root;
	while (node != nullptr && node->depth_ <= OctreeNode::DEFAULT_DEPTH)
	{
		std::vector<float> distance;

		for (uint32_t i = 0; i < OctreeNode::NUM_CHILDREN; i++)
		{
			distance.push_back(SqDistPointAABB(p, node->child_[i]->aabb_));
		}

		const auto index = std::distance(distance.begin(), std::min_element(distance.begin(), distance.end(), [](float a, float b) {return (a < b); }));
		node = node->child_[index].get();

		//store distance value processed 
		// remove it from distance vector
		//if removed value is same as stored value we
		if (node->isLeaf_)
			break;
	}

	return findMinDistance(p, node->objectList_);
}
