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
		ComputeAABB(rb);
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
	m_AABBCulling.log = m_AABBCulling.isEnabled ? L"Press F1 to disable AABB culling" : L"Press F1 to enable AABB culling";
	m_sphereCulling.log = m_sphereCulling.isEnabled ? L"Press F1 to disable sphere culling" : L"Press F1 to enable sphere culling";

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

	//@Clear debug colors
	for (RigidbodyComponent* rb : m_rigidbodies) {
		rb->m_shape->m_AABBColor = Colors::Red;
		rb->m_shape->m_sphereColor = Colors::Red;
	}

	//@First loop: Integration + First culling algorithm
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

	//@SSScheme
	for (unsigned int i = 0; i <m_rigidbodies.size();i++){
		//Each rb checks upwards, so we avoid duplicate colliders.
		//@BroadPhase
		for (unsigned int j = i + 1; j < m_rigidbodies.size(); j++) {
			//@To avoid double checks, we only check upwards
			if (BroadPhase(m_rigidbodies[i], m_rigidbodies[j])) m_pairs.push_back(make_pair(m_rigidbodies[i], m_rigidbodies[j]));
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
bool PhysicSystem::BroadPhase(RigidbodyComponent * rb1, RigidbodyComponent * rb2) {

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
		//Get aabbs depending on shape, and transform state (Rotation)
		AABB box1 = ComputeAABB(rb1);
		AABB box2 = ComputeAABB(rb2);

		float diffX = abs(t1.m_position.x - t2.m_position.x);
		float sum = box1.m_halfExtents.x + box2.m_halfExtents.x;
		if (abs(t1.m_position.x - t2.m_position.x) > (box1.m_halfExtents.x + box2.m_halfExtents.x)) {
			return false;
		}
		if (abs(t1.m_position.y - t2.m_position.y) > (box1.m_halfExtents.y + box2.m_halfExtents.y)) {
			return false;
		}
		if (abs(t1.m_position.z - t2.m_position.z) > (box1.m_halfExtents.z + box2.m_halfExtents.z)) {
			return false;
		}
		
		/*OTHER APPROACH 
		float thisRight = t1.m_position.x + box1.m_halfExtent.x; float otherRight = box2.m_center.x + box2.m_halfExtent.x;
		float thisLeft = box1.m_center.x - box1.m_halfExtent.x; float otherLeft = box2.m_center.x - box2.m_halfExtent.x;
		float thisTop = box1.m_center.y + box1.m_halfExtent.y; float otherTop = box2.m_center.y + box2.m_halfExtent.y;
		float thisBottom = box1.m_center.y - box1.m_halfExtent.y; float otherBottom = box2.m_center.y - box2.m_halfExtent.y;
		float thisFront = box1.m_center.z + box1.m_halfExtent.z; float otherFront = box2.m_center.z + box2.m_halfExtent.z;
		float thisBack = box1.m_center.z - box1.m_halfExtent.z; float otherBack = box2.m_center.z - box2.m_halfExtent.z;

		if (thisRight < otherLeft || thisLeft > otherRight)
			return false;

		if ( thisTop < otherBottom || thisBottom > otherTop )
			return false;

		if ( thisFront < otherBack || thisBack > otherFront)
			return false;*/

			
		//@Otherwise
		rb1->m_shape->m_AABBColor = Colors::Yellow;
		rb2->m_shape->m_AABBColor = Colors::Yellow;
		return true;

		/*if (!(
			thisRight < otherLeft
			|| thisLeft > otherRight
			|| thisTop < otherBottom
			|| thisBottom > otherTop
			|| thisFront < otherBack
			|| thisBack > otherFront
			)
			) {
			//@Set AABB colors
			rb1->m_shape->m_AABBColor = Colors::Yellow;
			rb2->m_shape->m_AABBColor = Colors::Yellow;
			return true;
		}
		else {
			return false;
		}*/
	}

	//Default: if BroadPhase is disabled
	return true;
}
//Get AABBs from different collider shapes
AABB PhysicSystem::ComputeAABB(RigidbodyComponent * rb)
{
	
	//@Process AABB information, using necessary TransformComponent info (Rotation?)
	switch (rb->m_shape->m_type) {
		case ShapeType::SPHERE:
		{
			Sphere* sphere = dynamic_cast<Sphere*>(rb->m_shape);
			sphere->m_AABB = AABB{ Vector3(sphere->m_radius, sphere->m_radius, sphere->m_radius) };
		}
		break;
		case ShapeType::OBB:
		{
			OrientedBoundingBox * obb = dynamic_cast<OrientedBoundingBox*>(rb->m_shape);
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


