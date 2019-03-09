#include "pch.h"
#include "..\Headers\BroadPhase.h"
#include "Sphere.h"
#include "OrientedBoundingBox.h"
#include "GameObject.h"

using namespace DirectX;
using namespace SimpleMath;
using namespace std;

BroadPhase::BroadPhase()
	: m_AABBTreeRoot(Vector3(15, 15, 15), Vector3::Zero)
{
	//@Initialize one off grid
}

BroadPhase::~BroadPhase()
{
}

bool BroadPhase::ComputeAABBTest(RigidbodyComponent * rb1, RigidbodyComponent * rb2)
{
	TransformComponent t1 = rb1->m_owner->m_transform;
	TransformComponent t2 = rb2->m_owner->m_transform;
	AABB box1 = ComputeAABB(rb1);
	AABB box2 = ComputeAABB(rb2);

	return AABBTest(box1, box2, t1.m_position, t2.m_position);
}

bool BroadPhase::AABBTest(AABB box1, AABB box2, DirectX::SimpleMath::Vector3 center1, DirectX::SimpleMath::Vector3 center2)
{
	if (abs(center1.x - center2.x) > (box1.m_halfExtents.x + box2.m_halfExtents.x)) {
		return false;
	}
	if (abs(center1.y - center2.y) > (box1.m_halfExtents.y + box2.m_halfExtents.y)) {
		return false;
	}
	if (abs(center1.z - center2.z) > (box1.m_halfExtents.z + box2.m_halfExtents.z)) {
		return false;
	}
	return true;
}

//Get AABBs from different collider shapes
AABB BroadPhase::ComputeAABB(RigidbodyComponent * rb)
{

	//@Process AABB information, using necessary TransformComponent info (Rotation?)
	switch (rb->m_shape->m_type) {
	case ShapeType::SPHERE:
	{
		Sphere* sphere = static_cast<Sphere*>(rb->m_shape);
		sphere->m_AABB = AABB{ Vector3(sphere->m_radius, sphere->m_radius, sphere->m_radius) };
	}
	break;
	case ShapeType::OBB:
	{
		OrientedBoundingBox * obb = static_cast<OrientedBoundingBox*>(rb->m_shape);
#pragma region AABB from OBB
		/* From Christer Ericson's Real-Time collision detection book (OPTIMIZED WITHOUT CALCULATING BC)
		// Transform AABB a by the matrix m and translation t, // find maximum extents, and store result into AABB b.
		void UpdateAABB(AABB a, float m[3][3], float t[3], AABB &b)
		{
			for (int i = 0; i < 3;i++)
			{
				b.c[i] = t[i];
				b.r[i] = 0.0f;
				for (int j = 0; j < 3;j++)
				{
					b.c[i] += m[i][j] * a.c[j];
					b.r[i] += Abs(m[i][j]) * a.r[j];
				}
			}
		}
		*/
		//@People seem to like to do this from matrices, I'm not smart enough to derive another approach for now
		XMFLOAT4X4 m = Matrix::CreateFromQuaternion(rb->m_owner->m_transform.m_rotation);//m[3][3]
		float ar[3] = { obb->m_halfExtents.x, obb->m_halfExtents.y, obb->m_halfExtents.z };
		float * br[3] = { &obb->m_AABB.m_halfExtents.x, &obb->m_AABB.m_halfExtents.y, &obb->m_AABB.m_halfExtents.z }; //b.r

		for (int i = 0; i < 3; i++) {
			*br[i] = 0.0f;
			for (int j = 0; j < 3; j++) {
				*br[i] += abs(m(j, i)) * ar[j];//Flipped matrix components given we dont know DirectX's convention.
			}
		}
#pragma endregion
	}
	break;
	}
	return rb->m_shape->m_AABB;
}

vector<AABBNode*> BroadPhase::GetFinalNodes(AABBNode * rootNode)
{
	vector<AABBNode*> finalNodes;
	if (rootNode->m_children.empty()) {
		//Is empty = is final
		finalNodes.push_back(rootNode);
	}
	else {
		//Is not empty
		for (unsigned int i = 0; i < rootNode->m_children.size(); i++) {
			//Get the vector returned from calling the function on them
			//If that vector is not empty, we add it to ours
			vector<AABBNode*> childrenNodes = GetFinalNodes(&rootNode->m_children[i]);
			//@There's at least one, itself
			for (unsigned int j = 0; j < childrenNodes.size(); j++) {
				finalNodes.push_back(childrenNodes[j]);
			}
		}
	}
	return finalNodes;
}
//@Designed to search from top node
std::vector<AABBNode*> BroadPhase::GetNonFinalNodes(AABBNode * rootNode)
{
	vector<AABBNode*> orderedNonFinalNodes;
	if (rootNode->m_children.empty()) {
		//This node is final.
		//Don't add it to the vector
	}
	else {
		//Node is non final.
		//It can be semi-final or non final.(Children are final nodes or not)
		for (unsigned int i = 0; i < rootNode->m_children.size(); i++) {
			vector<AABBNode*> childNonFinalNodes = GetNonFinalNodes(&rootNode->m_children[i]);
			for (unsigned int j = 0; j < childNonFinalNodes.size(); j++) {
				orderedNonFinalNodes.push_back(childNonFinalNodes[j]);
			}
		}
		orderedNonFinalNodes.push_back(rootNode);
	}
	return orderedNonFinalNodes;
}

std::vector<AABBNode*> BroadPhase::GetSemiFinalNodes(AABBNode * rootNode)
{
	vector<AABBNode*> semiFinalNodes;
	if (rootNode->m_children.empty()) {
		//This node is final.
		//Don't add it to the vector
	}
	else {
		//Node is non final.
		//It can be semi-final or non final.(Children are final nodes or not)
		bool isSemiFinalNode = true;
		for (int i = 0; i < rootNode->m_children.size(); i++) {
			AABBNode * childNode = &rootNode->m_children[i];
			if (!childNode->m_children.empty()) {
				//Node is not semifinal, look for one in its children
				isSemiFinalNode = false;
				vector < AABBNode * > childSemiFinalNodes = GetSemiFinalNodes(childNode);
				for (AABBNode* childSemiFinalNode : childSemiFinalNodes) {
					semiFinalNodes.push_back(childSemiFinalNode);
				}
			}
		}
		if (isSemiFinalNode) {
			//if it is, push itself
			semiFinalNodes.push_back(rootNode);
		}
	}
	return semiFinalNodes;
}
bool BroadPhase::TestAgainstAABBTree(RigidbodyComponent * rb, AABBNode * rootNode)
{
	AABB rbBox = ComputeAABB(rb);
	if (AABBTest(rbBox, rootNode->m_AABB, rb->m_owner->m_transform.m_position, rootNode->m_centre)) {
		rootNode->m_containing.push_back(rb);
		return true;
	}
	return false;
}
//@Add eight children with right size and centre point. Shrinking on the other hand, is just about clearing m_children.
void BroadPhase::ExpandNode(AABBNode * rootNode)
{
	Vector3 centre = rootNode->m_centre;
	Vector3 halfExtents = rootNode->m_AABB.m_halfExtents;
	rootNode->m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(halfExtents.x / 2, halfExtents.y / 2, halfExtents.z / 2)));//Right, top, front
	rootNode->m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(-halfExtents.x / 2, halfExtents.y / 2, halfExtents.z / 2)));//Left, top, front
	rootNode->m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(-halfExtents.x / 2, -halfExtents.y / 2, halfExtents.z / 2)));//Left, bottom, front
	rootNode->m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(halfExtents.x / 2, -halfExtents.y / 2, halfExtents.z / 2)));//Right, bottom, front

	rootNode->m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(halfExtents.x / 2, halfExtents.y / 2, -halfExtents.z / 2)));//Right, top, back
	rootNode->m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(-halfExtents.x / 2, halfExtents.y / 2, -halfExtents.z / 2)));//Left, top, back
	rootNode->m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(-halfExtents.x / 2, -halfExtents.y / 2, -halfExtents.z / 2)));//Left, bottom, back
	rootNode->m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(halfExtents.x / 2, -halfExtents.y / 2, -halfExtents.z / 2)));//Right, bottom, back
}

void BroadPhase::UpdateDynamicTree()
{

	//@Check for shrinking (We loop upwards from the bottom, getting all nonfinal nodes).
	//We delete children from those nonfinal nodes whose children's m_containing sum is less than 8.
	//When deleting children, we add their contained rigidbodies to the parent.
	//Recursion.
	vector<AABBNode*> semiFinalNodes = GetSemiFinalNodes(&m_AABBTreeRoot);
	for (AABBNode * semiFinalNode : semiFinalNodes) {
		//@They are ordered upwards
		//@Count total contained rigidbodies in children
		vector<RigidbodyComponent *> alreadyProcessed;
		for (AABBNode curChild : semiFinalNode->m_children) {
			//Analyze every contained rigidbody in every childnode
			for (RigidbodyComponent * containedRb : curChild.m_containing) {
				bool isAlreadyProcessed = false;
				for (RigidbodyComponent * processedRb : alreadyProcessed) {
					if (containedRb == processedRb) {
						isAlreadyProcessed = true;
					}
				}
				if (!isAlreadyProcessed) {
					alreadyProcessed.push_back(containedRb);
				}
			}
		}
		//@Total amount of rigidbodies in all children nodes is lower than 8
		if (alreadyProcessed.size() < 20) {
			//Destroy children nodes, obtains all their contained rigidbodies (It is now a final node)
			for (RigidbodyComponent * rb : alreadyProcessed) {
				semiFinalNode->m_containing.push_back(rb);
			}
		semiFinalNode->m_children.clear();
		}
	}

	//Get all 'Final' AABBTree nodes (Recursion)
	vector<AABBNode*> finalNodes = GetFinalNodes(&m_AABBTreeRoot);

	//@Check for expanding(We loop through all final nodes. If they have 8 or more rigidbodies, they expand.
	for (int i = 0; i < finalNodes.size(); i++ ) {
		AABBNode * finalNode = finalNodes[i];
		if (finalNode->m_containing.size() >= 20) {
			//We Expand this node (Generate 8 children)
			//All children get added to the end of this list
			//This one gets sacked from the list.
			//We i--; not to skip a loop iteration
			ExpandNode(finalNode);
			//@Updates vector
			for (int j = 0; j < finalNode->m_children.size(); j++) {
				AABBNode * curNode = &finalNode->m_children[j];
				finalNodes.push_back(curNode);
			}
			//@Its not a final node anymore
			finalNode->m_containing.clear();
			finalNodes.erase(finalNodes.begin() + i);
			i--;
		}
	}
}
