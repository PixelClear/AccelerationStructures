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

	if (std::fabs(det) < std::numeric_limits<float>::max())
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

void MeshQuery::Octree::buildTree(OctreeNode * node)
{
	for (size_t i = 0; i != OctreeNode::NUM_CHILDREN; i++)
	{
		node->child_[i] = nullptr;
	}

	if (node->depth_ >= OctreeNode::DEFAULT_DEPTH)
	{
		node->isLeaf_ = true;
		return;
	}

	for (size_t i = 0; i != OctreeNode::NUM_CHILDREN; i++)
	{
		node->child_[i] = std::make_unique<OctreeNode>();
		const AABB ChildBox = GetOctaSplit(node->aabb_, i);

		node->child_[i]->parent_ = node;
		node->child_[i]->aabb_ = ChildBox;
		node->child_[i]->depth_ = node->depth_ + 1;
		node->isLeaf_ = false;

		buildTree(node->child_[i].get());
	}
}

void MeshQuery::Octree::insertTriangle(OctreeNode * node, Triangle t)
{
	if (node == nullptr)
	{
		return;
	}

	if (node->depth_ >= OctreeNode::DEFAULT_DEPTH)
	{
		return;
	}

	for (size_t i = 0; i != OctreeNode::NUM_CHILDREN; i++)
	{
		if (intersect(t.aabb_, node->child_[i]->aabb_))
		{
			node->child_[i]->objectList_.push_back(t);

			if (!node->objectList_.empty())
				node->objectList_.erase(std::remove_if(node->objectList_.begin(), node->objectList_.end() - 1, [t](auto temp) { return temp == t; }));

			insertTriangle(node->child_[i].get(), t);
		}
	}
}

MeshQuery::BVH::BVH(const std::vector<Triangle>& prims, BvhStrategy strategy) :prims_(prims), strategy_(strategy)
{
	std::vector<PrimitiveInfo> primInfo(prims.size());
	for (size_t i = 0; i < prims.size(); i++)
	{
		primInfo[i] = { i, prims[i].aabb_ };
	}

	std::vector<Triangle> orderedPrims(prims.size());
	int totalNodes = 0;

	root_ = recursiveBuild(primInfo, 0, prims.size(), &totalNodes, orderedPrims);
}

MeshQuery::BvhNode * MeshQuery::BVH::recursiveBuild(std::vector<PrimitiveInfo>& primInfo, int start, int end, int* totalNodes, std::vector<Triangle>& orderedPrims)
{
	if (start == end)
		return nullptr;

	BvhNode* node = new BvhNode();
	(*totalNodes)++;

	AABB bounds;
	for (int i = start; i < end; i++) {
		bounds = Union(bounds, primInfo[i].aabb_);
	}

	int numOfPrims = end - start;
	if (numOfPrims == 1) {
		int offset = orderedPrims.size();
		for (int i = start; i < end; i++) {
			orderedPrims.push_back(prims_[primInfo[i].primNum_]);
		}
		node->initLeaf(offset, numOfPrims, bounds);
		return node;
	}
	else {
		AABB centroidBounds;
		for (int i = start; i < end; i++) {
			centroidBounds = Union(centroidBounds, primInfo[i].centroid_);
		}

		int axis = centroidBounds.getDominantAxis();
		int mid = (start + end) / 2;
		//We dont have any volume so we should stop the recursion
		if (centroidBounds.min_[axis] == centroidBounds.max_[axis]) {
			int offset = orderedPrims.size();
			for (int i = start; i < end; i++) {
				orderedPrims.push_back(prims_[primInfo[i].primNum_]);
			}
			node->initLeaf(offset, numOfPrims, bounds);
			return node;
		}
		else {
			//ToDo add other methods later on currently only midpoint
			float midPt = (centroidBounds.min_[axis] + centroidBounds.max_[axis]) / 2;

			PrimitiveInfo* midPtr = std::partition(&primInfo[start],
				&primInfo[end - 1] + 1,
				[axis, midPt](const PrimitiveInfo& prim) {
				return prim.centroid_[axis] < midPt;
			});

			mid = midPtr - &primInfo[0];

			if (mid == start || mid == end) {
				//ToDo currently undesired till we implement another methods
				printf("I shouldnt come here!");
			}

			node->initInterior(axis, recursiveBuild(primInfo, start, mid, totalNodes, orderedPrims),
				recursiveBuild(primInfo, mid, end, totalNodes, orderedPrims));
		}
	}

	return node;
}
