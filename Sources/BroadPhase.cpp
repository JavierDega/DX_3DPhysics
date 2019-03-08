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
		finalNodes.push_back(rootNode);
	}
	else {
		for (unsigned int i = 0; i < rootNode->m_children.size(); i++) {
			//Get the vector returned from calling the function on them
			//If that vector is not empty, we add it to ours
			vector<AABBNode*> childrenNodes = GetFinalNodes(&rootNode->m_children[i]);
			if (!childrenNodes.empty()) {//@Redundant check
				for (unsigned int j = 0; j < childrenNodes.size(); j++) {
					finalNodes.push_back(childrenNodes[j]);
				}
			}
		}
	}
	return finalNodes;
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
