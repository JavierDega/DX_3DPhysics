#ifndef BROADPHASE_H_
#define BROADPHASE_H_
#include "RigidbodyComponent.h"

struct AABBNode {
public:
	AABBNode(DirectX::SimpleMath::Vector3 halfExtents, DirectX::SimpleMath::Vector3 centre)
		: m_centre(centre) {
		m_AABB.m_halfExtents = halfExtents;
	}
	AABB m_AABB;
	DirectX::SimpleMath::Vector3 m_centre;
	std::vector<AABBNode> m_children;
	std::vector<RigidbodyComponent*> m_containing;
};
class BroadPhase
{
public:
	BroadPhase();
	~BroadPhase();

	bool ComputeAABBTest(RigidbodyComponent * rb1, RigidbodyComponent * rb2);
	bool AABBTest(AABB box1, AABB box2, DirectX::SimpleMath::Vector3 center1, DirectX::SimpleMath::Vector3 center2);
	AABB ComputeAABB(RigidbodyComponent * rb);
	//@Recursive call
	std::vector<AABBNode*> GetFinalNodes(AABBNode * rootNode);
	std::vector<AABBNode*> GetNonFinalNodes(AABBNode * rootNode);//@@DEPRECATED
	std::vector < AABBNode * > GetSemiFinalNodes(AABBNode * rootNode);
	//@Recursive call
	bool TestAgainstAABBTree(RigidbodyComponent * rb, AABBNode * rootNode);
	void ExpandNode(AABBNode * rootNode);
	void UpdateDynamicTree();//@Hierarchical Grid
	//Variables
	AABBNode m_AABBTreeRoot;

};
#endif /*BROADPHASE_H_*/