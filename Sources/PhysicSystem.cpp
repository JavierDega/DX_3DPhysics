#include "pch.h"
#include "..\Headers\PhysicSystem.h"
#include "ObjectSystem.h"
#include "Sphere.h"
#include "OrientedBoundingBox.h"

using namespace DirectX;
using namespace SimpleMath;
using namespace std;


//Instance
PhysicSystem * PhysicSystem::m_instance = NULL;
PhysicSystem * PhysicSystem::GetInstance()
{
	//Singleton
	if (m_instance == NULL)
	{
		m_instance = new PhysicSystem();
	}
	return m_instance;
}
//Constructor
PhysicSystem::PhysicSystem()
{
	m_gravity = Vector3 ( 0, -9.8f, 0 );//@Relative to mass
	//Viscosity for earth's air @  0'Celsius = 1.33*10^-5 kg/ms^2
	m_airViscosity = 0.133f;
	m_minDt = 1.0f / 60.0f;
	m_accumulator = 0;
	m_visualizeContacts.isEnabled = true;
	m_uniformGrid.isEnabled = false;
	m_hierarchicalGrid.isEnabled = false;
	m_AABBCulling.isEnabled = true;
	m_sphereCulling.isEnabled = false;
	m_stepOnce = false;
	m_stepMode = true;
}
//Destructor
PhysicSystem::~PhysicSystem()
{

}
///Init
void PhysicSystem::Initialize(ID3D11Device1 * device, ID3D11DeviceContext1 * deviceContext)
{
	//@What do here?
	//Process bounding volumes of all rigidbody shapes, so that we can create associated geometric primitives on graphic system
	vector<RigidbodyComponent*> m_rigidbodies = ObjectSystem::GetInstance()->GetRigidbodyComponentList();
	for (RigidbodyComponent* rb : m_rigidbodies) {
		m_broadPhase.ComputeAABB(rb);
		//ComputeSphere(rb);?
	}

}
///Init window
void PhysicSystem::InitWindow(D3D11_VIEWPORT screenViewport)
{
	//@What do here?
}
///Timestep and run physics
void PhysicSystem::Update(float dt)
{
	//@Update settings text
	m_fps = to_wstring(1 / dt);
	m_visualizeContacts.log = m_visualizeContacts.isEnabled ? L"Press F1 hide contact points" : L"Press F1 to view contact points";
	m_uniformGrid.log = m_uniformGrid.isEnabled ? L"Press F2 to disable uniform grid subdivision" : L"Press F2 to enable uniform grid subdivision";
	m_hierarchicalGrid.log = m_hierarchicalGrid.isEnabled ? L"Press F3 to disable octree subdivision" : L"Press F3 to enable octree subdivision";
	m_AABBCulling.log = m_AABBCulling.isEnabled ? L"Press F4 to disable AABB culling" : L"Press F4 to enable AABB culling";

	//@Debug step mode
	if (m_stepMode) {
		if (m_stepOnce) {
			UpdatePhysics(m_minDt);
			m_stepOnce = false;
		}
	
	}
	else {
		//@Timestep
		//https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-core-engine--gamedev-7493
		m_accumulator += dt;

		// @Avoid spiral of death and clamp dt, thus clamping
		// how many times the UpdatePhysics can be called in
		// a single game loop.
		if (m_accumulator > 0.2f) m_accumulator = 0.2f;

		while (m_accumulator > m_minDt) {
			UpdatePhysics(m_minDt);
			m_accumulator -= m_minDt;
		}
	}
	//@To create a lerp between this frame and the next, interact with the graphic system.
	//ApproxTransform.position = transform.position + m_velocity*m_accumulator ?
	float alpha = m_accumulator / m_minDt;

}
///Reset
void PhysicSystem::Reset()
{
	//@What do here?
}
//Physics loop
void PhysicSystem::UpdatePhysics(float dt) {

	vector<RigidbodyComponent*> m_rigidbodies = ObjectSystem::GetInstance()->GetRigidbodyComponentList();
	vector<pair<RigidbodyComponent*, RigidbodyComponent*>> m_pairs;

	//@Clear all previous frame's contact points
	m_narrowPhase.m_solver.m_contactManifolds.clear();

	//@Clear debug colors
	for (RigidbodyComponent* rb : m_rigidbodies) {
		rb->m_shape->m_AABBColor = Colors::Red;
		rb->m_shape->m_sphereColor = Colors::Red;
	}

	//@First loop: Integration
	for (unsigned int i = 0; i < m_rigidbodies.size(); i++) {
		RigidbodyComponent* currentRb = m_rigidbodies[i];
		//@Integration
		if (currentRb->m_isKinematic) {
			currentRb->m_acceleration = Vector3::Zero;
			currentRb->m_velocity = Vector3::Zero;
			currentRb->m_angularVelocity = Vector3::Zero;
			currentRb->m_angularAcceleration = Vector3::Zero;
		}
		else
		{
			//Calculate generalized forces
			currentRb->m_force -= m_airViscosity * currentRb->m_velocity;//@Viscosity
			currentRb->m_force += m_gravity * currentRb->m_mass;//@force relative to mass
			//@Angular drag?
			currentRb->m_torque -= m_airViscosity * currentRb->m_angularVelocity;

			//Apply forces
			currentRb->m_acceleration = currentRb->m_force / currentRb->m_mass;
			currentRb->m_velocity += currentRb->m_acceleration*dt;
			currentRb->m_owner->m_transform.m_position += currentRb->m_velocity*dt;

			//Angular
			currentRb->m_angularAcceleration = currentRb->m_torque / currentRb->m_mass;//For now, we use the mass scalar as our moment of inertia, our inertia tensor
			currentRb->m_angularVelocity += currentRb->m_angularAcceleration*dt;
			float radiansPerSecond = currentRb->m_angularVelocity.Length();
			if (radiansPerSecond > FLT_EPSILON) {
				currentRb->m_owner->m_transform.m_rotation *= Quaternion::CreateFromAxisAngle(currentRb->m_angularVelocity / radiansPerSecond, radiansPerSecond*dt);
			}
		}
		//Forces are computed every frame
		currentRb->m_force = Vector3::Zero;
		currentRb->m_torque = Vector3::Zero;
	}

	//Get all 'Final' AABBTree nodes (Recursion)
	vector<AABBNode*> m_finalTreeNodes = m_broadPhase.GetFinalNodes(&m_broadPhase.m_AABBTreeRoot);

	//@SSScheme
	for (unsigned int i = 0; i < m_finalTreeNodes.size(); i++) {
		//Clear bin for next frame
		m_finalTreeNodes[i]->m_containing.clear();
		for (unsigned int j = 0; j < m_rigidbodies.size(); j++) {
			m_broadPhase.TestAgainstAABBTree(m_rigidbodies[j], m_finalTreeNodes[i]);
		}
	}

	//@Broadphase for each bin
	for (unsigned int i = 0; i < m_finalTreeNodes.size(); i++) {
		//Each node is a separate space, where we do a N^2 collision test.
		for (unsigned int j = 0; j < m_finalTreeNodes[i]->m_containing.size(); j++) {
			//Each rb checks upwards, so we avoid redundant collision checks
			for (unsigned int k = j + 1; k < m_finalTreeNodes[i]->m_containing.size(); k++) {
				if (ComputeBroadPhase(m_finalTreeNodes[i]->m_containing[j], m_finalTreeNodes[i]->m_containing[k])) {
					m_pairs.push_back(make_pair(m_finalTreeNodes[i]->m_containing[j], m_finalTreeNodes[i]->m_containing[k]));
				}
			}
		}
	}

	//@Start nulling out collider pairs
	//@Medium Phase
	//@Narrow Phase
	for (unsigned int i = 0; i < m_pairs.size(); i++) {
		ComputeNarrowPhase(m_pairs[i].first, m_pairs[i].second, dt);
	}

	///@Surviving pairs, with contact points, penetration and normal information have been logged to solver and MUST be colliding
	m_narrowPhase.m_solver.Solve(dt);
}
//@BROADPHASE
bool PhysicSystem::ComputeBroadPhase(RigidbodyComponent * rb1, RigidbodyComponent * rb2) {

	if (rb1->m_isKinematic && rb2->m_isKinematic) return false;//@Two kinematic colliders dont need collision response

	TransformComponent t1 = rb1->m_owner->m_transform;
	TransformComponent t2 = rb2->m_owner->m_transform;
	//@Compute AABB
	if (m_sphereCulling.isEnabled) {
		//@Same process
	
	}
	if (m_AABBCulling.isEnabled) {
		/*Christer Ericson's
		int TestAABBAABB(AABB a, AABB b)
		{
		if (Abs(a.c[0] - b.c[0]) > (a.r[0] + b.r[0])) return 0;
		if (Abs(a.c[1] - b.c[1]) > (a.r[1] + b.r[1])) return 0;
		if (Abs(a.c[2] - b.c[2]) > (a.r[2] + b.r[2])) return 0;
		return 1;
		}*/
		if (m_broadPhase.ComputeAABBTest(rb1, rb2)) {
			rb1->m_shape->m_AABBColor = Colors::Yellow;
			rb2->m_shape->m_AABBColor = Colors::Yellow;
			return true;
		}
		else {
			return false;
		}
	}

	//Default: if BroadPhase is disabled
	return true;
}
//@NARROWPHASE
bool PhysicSystem::ComputeNarrowPhase(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt ) {

	switch (rb1->m_shape->m_type) {
		case ShapeType::SPHERE:
		{
			switch (rb2->m_shape->m_type) {
				case ShapeType::SPHERE:
				{
					 return m_narrowPhase.SphereToSphere(rb1, rb2, dt);
				}
				break;
				case ShapeType::OBB:
				{
					return m_narrowPhase.SphereToOBB(rb1, rb2, dt);
				}
				break;
			}
		}
		break;
		case ShapeType::OBB:
		{
			switch (rb2->m_shape->m_type) {
				case ShapeType::SPHERE:
				{
					return m_narrowPhase.SphereToOBB(rb2, rb1, dt);
				}
				break;
				case ShapeType::OBB:
				{
					return m_narrowPhase.OBBToOBB(rb1, rb2, dt);
				}
				break;
			}
		}
		break;
	}
	return false;
}

void PhysicSystem::EnableUniformGrid()
{
	m_uniformGrid.isEnabled = true;
	//@Add eight children to the rootNode
	m_broadPhase.m_AABBTreeRoot.m_children.clear();//Just in case
	Vector3 centre = m_broadPhase.m_AABBTreeRoot.m_centre;
	Vector3 halfExtents = m_broadPhase.m_AABBTreeRoot.m_AABB.m_halfExtents;
	m_broadPhase.m_AABBTreeRoot.m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(halfExtents.x / 2, halfExtents.y / 2, halfExtents.z / 2)));//Right, top, front
	m_broadPhase.m_AABBTreeRoot.m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(-halfExtents.x / 2, halfExtents.y / 2, halfExtents.z / 2)));//Left, top, front
	m_broadPhase.m_AABBTreeRoot.m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(-halfExtents.x / 2, -halfExtents.y / 2, halfExtents.z / 2)));//Left, bottom, front
	m_broadPhase.m_AABBTreeRoot.m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(halfExtents.x / 2, -halfExtents.y / 2, halfExtents.z / 2)));//Right, bottom, front

	m_broadPhase.m_AABBTreeRoot.m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(halfExtents.x / 2, halfExtents.y / 2, -halfExtents.z / 2)));//Right, top, back
	m_broadPhase.m_AABBTreeRoot.m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(-halfExtents.x / 2, halfExtents.y / 2, -halfExtents.z / 2)));//Left, top, back
	m_broadPhase.m_AABBTreeRoot.m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(-halfExtents.x / 2, -halfExtents.y / 2, -halfExtents.z / 2)));//Left, bottom, back
	m_broadPhase.m_AABBTreeRoot.m_children.push_back(AABBNode(halfExtents / 2, centre + Vector3(halfExtents.x / 2, -halfExtents.y / 2, -halfExtents.z / 2)));//Right, bottom, back
}

void PhysicSystem::DisableUniformGrid()
{
	if (m_hierarchicalGrid.isEnabled) {
		DisableHierarchicalGrid();
	}
	//Then disable itself
	//@Right now, the tree should consist of a root node, and eigh child nodes only.
	//Simply delete the rootNode's children?
	m_broadPhase.m_AABBTreeRoot.m_children.clear();
	m_uniformGrid.isEnabled = false;
}

void PhysicSystem::EnableHierarchicalGrid()
{

}

void PhysicSystem::DisableHierarchicalGrid()
{
}


