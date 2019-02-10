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
	m_frictionCoefficient = .01f;
	m_minDt = 1.0f / 60.0f;
	m_accumulator = 0;
	m_AABBCulling.isEnabled = true;
	m_sphereCulling.isEnabled = false;
	m_stepOnce = false;
	m_stepMode = false;
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
///Utility
///Physics loop
void PhysicSystem::UpdatePhysics(float dt) {
	vector<RigidbodyComponent*> m_rigidbodies = ObjectSystem::GetInstance()->GetRigidbodyComponentList();
	vector<pair<RigidbodyComponent*, RigidbodyComponent*>> m_collidingPairs;

	//@Clear debug colors
	for (RigidbodyComponent* rb : m_rigidbodies){
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
		}
		else
		{
			//Calculate generalized forces
			currentRb->m_force -= m_airViscosity * currentRb->m_velocity;//@Viscosity
			currentRb->m_force += m_gravity*currentRb->m_mass;//@force relative to mass

			//Apply forces
			currentRb->m_acceleration = currentRb->m_force / currentRb->m_mass;
			currentRb->m_velocity += currentRb->m_acceleration*dt;
			currentRb->m_owner->m_transform.m_position += currentRb->m_velocity*dt;
		}
		//Forces are computed every frame
		currentRb->m_force = Vector3(0, 0, 0);

		//@SSScheme

		//@BroadPhase
		for (unsigned int j = i + 1; j < m_rigidbodies.size(); j++) {
			//@To avoid double checks, we only check upwards
			if(BroadPhase(currentRb, m_rigidbodies[j])) m_collidingPairs.push_back(make_pair(currentRb, m_rigidbodies[j]));
		}
	}

	//@Start nulling out collider pairs
	
	//@Medium Phase

	///@Surviving pairs MUST be colliding.
	
	//@Narrow Phase
	for (unsigned int i = 0; i < m_collidingPairs.size(); i++) {
		NarrowPhase(m_collidingPairs[i].first, m_collidingPairs[i].second, dt);
	}
}
//@BROADPHASE
bool PhysicSystem::BroadPhase(RigidbodyComponent * rb1, RigidbodyComponent * rb2) {

	if (rb1->m_isKinematic && rb2->m_isKinematic) return false;//@Two kinematic colliders dont need collision response


	//@Compute AABB
	if (m_sphereCulling.isEnabled) {
		//@Same process
	
	}
	if (m_AABBCulling.isEnabled) {

		//Get aabbs depending on shape, and transform state (Rotation)
		AABB box1 = ComputeAABB(rb1);
		AABB box2 = ComputeAABB(rb2);

		//Define bounds
		float thisRight = box1.m_center.x + box1.m_halfExtent.x; float otherRight = box2.m_center.x + box2.m_halfExtent.x;
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
			return false;
			
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
	//Dynamic cast to find shape
	Sphere* sphere = dynamic_cast<Sphere*>(rb->m_shape);
	OrientedBoundingBox * obb = dynamic_cast<OrientedBoundingBox*>(rb->m_shape);
	
	//@Process AABB information, using necessary TransformComponent info (Rotation?)
	if (sphere) {

		sphere->m_AABB = AABB{ rb->m_owner->m_transform.m_position, Vector3(sphere->m_radius, sphere->m_radius, sphere->m_radius)};
	}
	else if (obb) {
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
		//float t[3] = { rb->m_owner->m_transform.m_position.x, rb->m_owner->m_transform.m_position.y, rb->m_owner->m_transform.m_position.z };//t[3], is also a.c
		float ar[3] = { obb->m_halfExtents.x, obb->m_halfExtents.y, obb->m_halfExtents.z };
		//float * bc[3] = { &obb->m_AABB.m_center.x, &obb->m_AABB.m_center.y, &obb->m_AABB.m_center.z };// b.c
		obb->m_AABB.m_center = rb->m_owner->m_transform.m_position;
		float * br[3] = { &obb->m_AABB.m_halfExtent.x, &obb->m_AABB.m_halfExtent.y, &obb->m_AABB.m_halfExtent.z }; //b.r

		for (int i = 0; i < 3; i++) {
			//*bc[i] = t[i];
			*br[i] = 0.0f;
			for (int j = 0; j < 3; j++) {
				//*bc[i] += m(i,j) * t[j];
				*br[i] += abs(m(i, j)) * ar[j];
			}
		}

		#pragma endregion
	}

	return rb->m_shape->m_AABB;
}
//@NARROWPHASE
bool PhysicSystem::NarrowPhase(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt ) {

	switch (rb1->m_shape->m_type) {
		case ShapeType::SPHERE:
		{
			switch (rb2->m_shape->m_type) {
				case ShapeType::SPHERE:
				{
					 return SphereToSphere(rb1, rb2, dt);
				}
				break;
				case ShapeType::OBB:
				{
					return SphereToOBB(rb1, rb2, dt);
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
					return SphereToOBB(rb2, rb1, dt);
				}
				break;
				case ShapeType::OBB:
				{
					return OBBToOBB(rb1, rb2, dt);
				}
				break;
			}
		}
		break;
	}
	return false;
}

bool PhysicSystem::SphereToSphere(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt)
{
	//@At this point it can be a static_cast
	Sphere * sphere1 = dynamic_cast<Sphere*>(rb1->m_shape);
	Sphere * sphere2 = dynamic_cast<Sphere*>(rb2->m_shape);
	TransformComponent * t1 = &rb1->m_owner->m_transform;
	TransformComponent * t2 = &rb2->m_owner->m_transform;
	//@Cases?
	//Resting contact, moving contact, contact vs kinematic
	//@Impulse based collision response
	//@1:Are they colliding?
	float distSq = Vector3::DistanceSquared(t1->m_position, t2->m_position);
	// Calculate the sum of the radii, then square it
	float sumRadiiSq = sphere1->m_radius + sphere2->m_radius;
	sumRadiiSq *= sumRadiiSq;
	if (distSq <= sumRadiiSq) {
		// A and B are touching
		//@Impulse based collision resolution
		///1:Displacement
#pragma region Displacement
		
		//Calculate overlap
		float dist = sqrtf(distSq);
		float overlap = (dist - sphere1->m_radius - sphere2->m_radius);
		//@Static collision resolution based on speed
		float v1Length = rb1->m_velocity.Length();
		float v2Length = rb2->m_velocity.Length();
		//@What if two objects with no velocity just collided?
		float v1Ratio = v1Length / (v1Length + v2Length);
		float v2Ratio = v2Length / (v1Length + v2Length);

		//@We want to keep values for accurate displacement
		Vector3 pos1Prev = t1->m_position;
		Vector3 pos2Prev = t2->m_position;

		if (!rb1->m_isKinematic)t1->m_position -= v1Ratio * overlap * (pos1Prev - pos2Prev) / dist;
		if (!rb2->m_isKinematic)t2->m_position += v2Ratio * overlap * (pos1Prev - pos2Prev) / dist;
#pragma endregion

		///2:Dynamic resolution
#pragma region Resolution

		//http://www.gamasutra.com/view/feature/131424/pool_hall_lessons_fast_accurate_.php?page=3
		// First, find the normalized vector n from the center of 
		// circle1 to the center of circle2

		// Find the length of the component of each of the movement
		// vectors along n. 
		// a1 = v1 . n
		// a2 = v2 . n
		Vector3 normal = t1->m_position - t2->m_position;
		normal.Normalize();

		float a1 = rb1->m_velocity.Dot(normal);
		float a2 = rb2->m_velocity.Dot(normal);

		// Using the optimized version, 
		// optimizedP =  2(a1 - a2)
		//              -----------
		//                m1 + m2
		float optimizedP = (2.0f * (a1 - a2)) / (rb1->m_mass + rb2->m_mass);

		//@Store previous velocity
		Vector3 prevVel = rb1->m_velocity;
		Vector3 prevVel2 = rb2->m_velocity;

		// Calculate v1', the new movement vector of circle1
		// v1' = v1 - optimizedP * m2 * n
		rb1->m_velocity = rb1->m_velocity - optimizedP * rb2->m_mass * normal;
		rb2->m_velocity = rb2->m_velocity + optimizedP * rb1->m_mass * normal;

#pragma endregion

		Vector3 velDelta = rb1->m_velocity - prevVel;
		Vector3 velDelta2 = rb2->m_velocity - prevVel2;

		//3: Friction::Because of our Impulse based resolution, we need to calculate the normal force 'After the fact'
#pragma region Kinetic friction

		//@Friction:
		//Calculate normal force from impulse
		//f = m*a
		//f = mdv/dt
		Vector3 rb1Force = rb1->m_mass * velDelta / dt;
		//Find component of force along normal
		float normalForce = rb1Force.Dot(normal);//@DX11's Dot is unnornalized so already multiplied by length

		Vector3 rb2Force = rb2->m_mass * velDelta2 / dt;
		float normal2Force = rb2Force.Dot(normal);

		Vector3 vel1Norm = rb1->m_velocity;
		vel1Norm.Normalize();
		Vector3 vel2Norm = rb2->m_velocity;
		vel2Norm.Normalize();

		Vector3 friction1 = -vel1Norm * m_frictionCoefficient * abs(normalForce);//normalForce should be positive here
		rb1->m_force += friction1;

		Vector3 friction2 = -vel2Norm * m_frictionCoefficient * abs(normal2Force);//normal2Force should be negative here
		rb2->m_force += friction2;
#pragma endregion

		return true;
	}
	return false;
}

bool PhysicSystem::SphereToOBB(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt)
{
	Sphere * sphere1 = dynamic_cast<Sphere*>(rb1->m_shape);
	OrientedBoundingBox * obb2 = dynamic_cast<OrientedBoundingBox*>(rb2->m_shape);
	TransformComponent * t1 = &rb1->m_owner->m_transform;
	TransformComponent * t2 = &rb2->m_owner->m_transform;

	/* Christer Ericson's Realtime collision detection
	// Returns true if sphere s intersects OBB b, false otherwise.
	// The point p on the OBB closest to the sphere center is also returned 
	int TestSphereOBB(Sphere s, OBB b, Point &p)
	{
		// Find point p on OBB closest to sphere center 
		ClosestPtPointOBB(s.c, b, p);
		// Sphere and OBB intersect if the (squared) distance from sphere 
		// center to point p is less than the (squared) sphere radius 
		Vector v=p-s.c;
		return Dot(v, v) <= s.r * s.r;
	}
	*/
	Vector3 closestPoint = ClosestPtPointOBB(t1->m_position, obb2, t2->m_position, t2->m_rotation);//Point &p;
	Vector3 v = closestPoint - t1->m_position;//v;

	if (v.LengthSquared() <= sphere1->m_radius * sphere1->m_radius) {
		//@They collide
				//@Impulse based collision resolution
		///1:Displacement
#pragma region Displacement

		//Calculate overlap
		float overlap = ( sphere1->m_radius - v.Length()  );//@Should always be positive value
		//@Static collision resolution based on speed
		float v1Length = rb1->m_velocity.Length();
		float v2Length = rb2->m_velocity.Length();
		//@What if two objects with no velocity just collided?
		float v1Ratio = v1Length / (v1Length + v2Length);
		float v2Ratio = v2Length / (v1Length + v2Length);

		//@We want to keep values for accurate displacement
		Vector3 pos1Prev = t1->m_position;
		Vector3 pos2Prev = t2->m_position;
		Vector3 contactToSphere = t1->m_position - closestPoint;
		contactToSphere.Normalize();
		Vector3 contactToOBB = t2->m_position - closestPoint;
		contactToOBB.Normalize();

		//Displace taking previous velocities into account
		if (!rb1->m_isKinematic)t1->m_position += v1Ratio * overlap * contactToSphere;
		if (!rb2->m_isKinematic)t2->m_position += v2Ratio * overlap * contactToOBB;
#pragma endregion
		
		///2:Dynamic resolution
#pragma region Resolution

		// First, find the normalized vector n from the center of 
		// circle1 to the center of circle2

		// Find the length of the component of each of the movement
		// vectors along n. 
		// a1 = v1 . n
		// a2 = v2 . n
		Vector3 normal = t1->m_position - t2->m_position;
		normal.Normalize();

		float a1 = rb1->m_velocity.Dot(normal);
		float a2 = rb2->m_velocity.Dot(normal);

		// Using the optimized version, 
		// optimizedP =  2(a1 - a2)
		//              -----------
		//                m1 + m2
		float optimizedP = (2.0f * (a1 - a2)) / (rb1->m_mass + rb2->m_mass);

		//@Store previous velocity
		Vector3 prevVel = rb1->m_velocity;
		Vector3 prevVel2 = rb2->m_velocity;

		// Calculate v1', the new movement vector of circle1
		// v1' = v1 - optimizedP * m2 * n
		rb1->m_velocity = rb1->m_velocity - optimizedP * rb2->m_mass * normal;
		rb2->m_velocity = rb2->m_velocity + optimizedP * rb1->m_mass * normal;

#pragma endregion

		Vector3 velDelta = rb1->m_velocity - prevVel;
		Vector3 velDelta2 = rb2->m_velocity - prevVel2;

		//3: Friction::Because of our Impulse based resolution, we need to calculate the normal force 'After the fact'
#pragma region Kinetic friction

		//@Friction:
		//Calculate normal force from impulse
		//f = m*a
		//f = mdv/dt
		Vector3 rb1Force = rb1->m_mass * velDelta / dt;
		//Find component of force along normal
		float normalForce = rb1Force.Dot(normal);//@DX11's Dot is unnornalized so already multiplied by length

		Vector3 rb2Force = rb2->m_mass * velDelta2 / dt;
		float normal2Force = rb2Force.Dot(normal);

		Vector3 vel1Norm = rb1->m_velocity;
		vel1Norm.Normalize();
		Vector3 vel2Norm = rb2->m_velocity;
		vel2Norm.Normalize();

		Vector3 friction1 = -vel1Norm * m_frictionCoefficient * abs(normalForce);//normalForce should be positive here
		rb1->m_force += friction1;

		Vector3 friction2 = -vel2Norm * m_frictionCoefficient * abs(normal2Force);//normal2Force should be negative here
		rb2->m_force += friction2;
#pragma endregion
		
		return true;
	}
	return false;
}

bool PhysicSystem::OBBToOBB(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt)
{
	OrientedBoundingBox * obb1 = dynamic_cast<OrientedBoundingBox*>(rb1->m_shape);
	OrientedBoundingBox * obb2 = dynamic_cast<OrientedBoundingBox*>(rb2->m_shape);
	return false;
}
///@Helpful queries
DirectX::SimpleMath::Vector3 PhysicSystem::ClosestPtPointOBB( Vector3 p, OrientedBoundingBox * b, Vector3 bc, Quaternion bRot)
{
	/*
	// Given point p, return point q on (or in) OBB b, closest to p 
	void ClosestPtPointOBB(Point p, OBB b, Point &q) 
	{ 
		Vector d =p-b.c;
		// Start result at center of box; make steps from there
		q = b.c; 
		// For each OBB axis... 
		for (int i = 0; i < 3;i++) 
		{ 
			// ...project d onto that axis to get the distance 
			// along the axis of d from the box center
			float dist = Dot(d, b.u[i]); 
			// If distance farther than the box extents, clamp to the box
			if (dist > b.e[i]) dist = b.e[i];
			if (dist < -b.e[i]) dist = -b.e[i];
			// Step that distance along the axis to get world coordinate
			q += dist * b.u[i];
		} 
	}
	*/
	Vector3 closestPoint;//Point &q
	float be[3] = { b->m_halfExtents.x, b->m_halfExtents.y, b->m_halfExtents.z };//b.e
	//Get basis vectors from matrix, from quaternion, to define b.u
	Matrix m = Matrix::CreateFromQuaternion(bRot);
	Vector3 bu[3] = { m.Right(), m.Up(), m.Forward() };//b.u
	

	Vector3 d = p - bc;
	closestPoint = bc;
	for (int i = 0; i < 3; i++) {
		float dist = d.Dot(bu[i]);
		
		if (dist > be[i]) dist = be[i];
		if (dist < -be[i]) dist = -be[i];
		closestPoint += dist * bu[i];
	}
	return closestPoint;
}
