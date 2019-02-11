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
	for (RigidbodyComponent* rb : m_rigidbodies){
		rb->m_shape->m_AABBColor = Colors::Red;
		rb->m_shape->m_sphereColor = Colors::Red;
	}
	//@Clear contact manifolds (Naive approach? Might be throwing away useful info?)
	m_solver.m_collidingPairs.clear();

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
			if(BroadPhase(currentRb, m_rigidbodies[j])) m_pairs.push_back(make_pair(currentRb, m_rigidbodies[j]));
		}
	}

	//@Start nulling out collider pairs
	//@Medium Phase
	
	//@Narrow Phase
	for (unsigned int i = 0; i < m_pairs.size(); i++) {
		NarrowPhase(m_pairs[i].first, m_pairs[i].second, dt);
	}
	///@Surviving pairs have been logged to solver and MUST be colliding
	m_solver.Solve(dt);
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
	
	//@Process AABB information, using necessary TransformComponent info (Rotation?)
	switch (rb->m_shape->m_type) {
		case ShapeType::SPHERE:
		{
			Sphere* sphere = dynamic_cast<Sphere*>(rb->m_shape);
			sphere->m_AABB = AABB{ rb->m_owner->m_transform.m_position, Vector3(sphere->m_radius, sphere->m_radius, sphere->m_radius) };
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
		break;
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

void PhysicSystem::ApplyImpulse(DirectX::SimpleMath::Vector3 impulse, DirectX::SimpleMath::Vector3 contactVector)
{
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
		///Create manifold and contact point
		//We provide generic info such as: Necessary contact points, each with penetration depth scalar, a common normal vector, and rigidbodies involved.
		//@CONVENTION: Normal always points to first rigidbody pair.
		//@Overlap is always positive
		float dist = sqrtf(distSq);
		float overlap = (sphere1->m_radius + sphere2->m_radius - dist);
		Vector3 normal = (t1->m_position - t2->m_position) / dist;
		//@Dirk Gregorious: Contact point is middle point of two surfaces
		Vector3 surfacePoint1 = t1->m_position - normal * sphere1->m_radius;
		Vector3 surfacePoint2 = t2->m_position + normal * sphere2->m_radius;

		ContactPoint contactPoint;
		contactPoint.m_penetration = overlap;
		contactPoint.m_position = surfacePoint1 + (surfacePoint2 - surfacePoint1)*0.5f;

		ContactManifold manifold;
		manifold.m_rigidbodies.first = rb1;
		manifold.m_rigidbodies.second = rb2;
		manifold.m_points.push_back(contactPoint);
		manifold.m_normal = normal;
		m_solver.m_collidingPairs.push_back(manifold);
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
	Vector3 v = t1->m_position - closestPoint;//v;
	float distSq = v.LengthSquared();
	if (distSq <= sphere1->m_radius * sphere1->m_radius) {
		// A and B are touching
		///Create manifold and contact point
		//We provide generic info such as: Necessary contact points, each with penetration depth scalar, a common normal vector, and rigidbodies involved.
		//@CONVENTION: Normal always points to first rigidbody pair.
		//@Overlap is always positive
		//Calculate overlap
		float dist = sqrtf(distSq);
		float overlap = ( sphere1->m_radius - dist  );//@Should always be positive value
		Vector3 normal = v/dist;
		ContactPoint contactPoint;
		contactPoint.m_penetration = overlap;
		contactPoint.m_position = closestPoint;

		ContactManifold manifold;
		manifold.m_rigidbodies.first = rb1;
		manifold.m_rigidbodies.second = rb2;
		manifold.m_points.push_back(contactPoint);
		manifold.m_normal = normal;
		m_solver.m_collidingPairs.push_back(manifold);

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
	/*From Christer Ericson's Real-Time Collision Detection
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
