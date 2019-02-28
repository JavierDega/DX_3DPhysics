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

	/* Christer Ericson's Real-Time collision detection
	int TestOBBOBB(OBB &a, OBB &b) {
		float ra, rb; 
		Matrix33 R, AbsR;
		// Compute rotation matrix expressing b in a’s coordinate frame 
		for (int i = 0; i < 3;i++) 
			for (int j = 0; j < 3;j++) 
				R[i][j] = Dot(a.u[i], b.u[j]);

		// Compute translation vector t 
		Vector t = b.c - a.c; 
		// Bring translation into a’s coordinate frame 
		t = Vector(Dot(t, a.u[0]), Dot(t, a.u[2]), Dot(t, a.u[2]));
		// Compute common subexpressions. Add in an epsilon term to 
		// counteract arithmetic errors when two edges are parallel and 
		// their cross product is (near) null (see text for details) 
		for (int i = 0; i < 3;i++) 
			for (int j = 0; j < 3;j++) 
				AbsR[i][j] = Abs(R[i][j]) + EPSILON;
		// Test axes L = A0, L = A1, L = A2 
		for (int i = 0; i < 3;i++) 
		{ 
			ra = a.e[i]; 
			rb = b.e[0] * AbsR[i][0] + b.e[1] * AbsR[i][1] + b.e[2] * AbsR[i][2];
			if (Abs(t[i]) > ra + rb) return 0;
		}
		// Test axes L = B0, L = B1, L = B2 
		for (int i = 0; i < 3;i++) 
		{ 
			ra = a.e[0] * AbsR[0][i] + a.e[1] * AbsR[1][i] + a.e[2] * AbsR[2][i]; 
			rb = b.e[i];
			if (Abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb) return 0;
		}
		// Test axis L = A0 x B0 
		ra = a.e[1] * AbsR[2][0] + a.e[2] * AbsR[1][0];
		rb = b.e[1] * AbsR[0][2] + b.e[2] * AbsR[0][1];
		if (Abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb) return 0;
		// Test axis L = A0 x B1 
		ra = a.e[1] * AbsR[2][1] + a.e[2] * AbsR[1][1];
		rb = b.e[0] * AbsR[0][2] + b.e[2] * AbsR[0][0];
		if (Abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb) return 0;
		// Test axis L = A0 x B2 
		ra = a.e[1] * AbsR[2][2] + a.e[2] * AbsR[1][2];
		rb = b.e[0] * AbsR[0][1] + b.e[1] * AbsR[0][0];
		if (Abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb) return 0;
		// Test axis L = A1 x B0 
		ra = a.e[0] * AbsR[2][0] + a.e[2] * AbsR[0][0];
		rb = b.e[1] * AbsR[1][2] + b.e[2] * AbsR[1][1];
		if (Abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb) return 0;
		// Test axis L = A1 x B1 
		ra = a.e[0] * AbsR[2][1] + a.e[2] * AbsR[0][1];
		rb = b.e[0] * AbsR[1][2] + b.e[2] * AbsR[1][0];
		if (Abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb) return 0;
		// Test axis L = A1 x B2 
		ra = a.e[0] * AbsR[2][2] + a.e[2] * AbsR[0][2];
		rb = b.e[0] * AbsR[1][1] + b.e[1] * AbsR[1][0];
		if (Abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb) return 0;
		// Test axis L = A2 x B0 
		ra = a.e[0] * AbsR[1][0] + a.e[1] * AbsR[0][0];
		rb = b.e[1] * AbsR[2][2] + b.e[2] * AbsR[2][1];
		if (Abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb) return 0;
		// Test axis L = A2 x B1 
		ra = a.e[0] * AbsR[1][1] + a.e[1] * AbsR[0][1];
		rb = b.e[0] * AbsR[2][2] + b.e[2] * AbsR[2][0];
		if (Abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb) return 0;
		// Test axis L = A2 x B2 
		ra = a.e[0] * AbsR[1][2] + a.e[1] * AbsR[0][2];
		rb = b.e[0] * AbsR[2][1] + b.e[1] * AbsR[2][0]; 
		if (Abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb) return 0;
		// Since no separating axis is found, the OBBs must be intersecting 
		return 1;
	}
	*/
	OrientedBoundingBox * a = dynamic_cast<OrientedBoundingBox*>(rb1->m_shape);//a
	OrientedBoundingBox * b = dynamic_cast<OrientedBoundingBox*>(rb2->m_shape);//b

	TransformComponent t1 = rb1->m_owner->m_transform;
	TransformComponent t2 = rb2->m_owner->m_transform;

	float ra, rb;
	Matrix R, AbsR;
	//.u are the local x,y,z axes
	//.e are the halfExtents
	Matrix ma = Matrix::CreateFromQuaternion(rb1->m_owner->m_transform.m_rotation);
	Vector3 au[3] = { ma.Right(), ma.Up(), ma.Forward() };//a.u
	float ae[3] = { a->m_halfExtents.x, a->m_halfExtents.y, a->m_halfExtents.z };

	Matrix mb = Matrix::CreateFromQuaternion(rb2->m_owner->m_transform.m_rotation);
	Vector3 bu[3] = { mb.Right(), mb.Up(), mb.Forward() };//b.u
	float be[3] = { b->m_halfExtents.x, b->m_halfExtents.y, b->m_halfExtents.z };

	// Compute rotation matrix expressing b in a’s coordinate frame 
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R.m[i][j] = au[i].Dot(bu[j]);

	
	// Compute translation vector t 
	Vector3 tv = rb2->m_owner->m_transform.m_position - rb1->m_owner->m_transform.m_position;
	// Bring translation into a’s coordinate frame 
	float t[3] = { tv.Dot(au[0]), tv.Dot(au[1]), tv.Dot(au[2]) };//@POSSIBLE ERRATA(YUP)

	//@@ALTERNATIVE: Manually detecting degenerate case (When two edges are parallel) and skipping all edge cross product axes if so
	//It can be proven that not false positives will come in this case https://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/

	// Compute common subexpressions. Add in an epsilon term to 
	// counteract arithmetic errors when two edges are parallel and 
	// their cross product is (near) null (see text for details) 
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			AbsR.m[i][j] = abs(R.m[i][j]) + FLT_EPSILON;

	//@Penetration values
	float m_penetrationDepth = FLT_MAX;
	//@Convention says normals point away from A
	Vector3 m_axisOfMinimumPenetration;//@Beware of floating point errors when two axis are very similar in penetration, as could be source of jittering
	bool m_isFaceToFaceCollision;
	bool m_isFace1;//Whether reference face is on shape 1 or 2
	Vector3 m_edge1Dir, m_edge2Dir;
	unsigned int m_faceIndex;
	//@Something to face case (Find incident face, reference face,..)
	// Test axes L = A0, L = A1, L = A2 
	for (int i = 0; i < 3;i++) 
	{ 
		ra = ae[i];
		rb = be[0] * AbsR.m[i][0] + be[1] * AbsR.m[i][1] + be[2] * AbsR.m[i][2];
		if (abs(t[i]) > ra + rb) return 0;
		else {
			//Non separating axis, record penetration
			float curPenetration = (ra + rb) - abs(t[i]);
			if (curPenetration < m_penetrationDepth) {
				m_penetrationDepth = curPenetration;
				m_axisOfMinimumPenetration = au[i];
				m_isFaceToFaceCollision = true;
				m_isFace1 = true;
				m_faceIndex = i;
			}
		}
	}
	
	//For solving in local space, then moving to global
	Vector3 bRotatedAxes[3] = { AbsR.Right(), AbsR.Up(), AbsR.Forward() };
	// Test axes L = B0, L = B1, L = B2 
	for (int i = 0; i < 3;i++) 
	{ 
		ra = ae[0] * AbsR.m[0][i] + ae[1] * AbsR.m[1][i] + ae[2] * AbsR.m[2][i];
		rb = be[i];
		if (abs(t[0] * R.m[0][i] + t[1] * R.m[1][i] + t[2] * R.m[2][i]) > ra + rb) return 0;
		else {
			//Non separating axis, record penetration
			float curPenetration = (ra + rb) - abs(t[0] * R.m[0][i] + t[1] * R.m[1][i] + t[2] * R.m[2][i]);
			if (curPenetration < m_penetrationDepth) {
				m_penetrationDepth = curPenetration;
				m_axisOfMinimumPenetration = bRotatedAxes[i];
				m_isFaceToFaceCollision = true;
				m_isFace1 = false;
				m_faceIndex = i;
			}
		}
	}

	// Test axis L = A0 x B0 
	ra = ae[1] * AbsR.m[2][0] + ae[2] * AbsR.m[1][0];
	rb = be[1] * AbsR.m[0][2] + be[2] * AbsR.m[0][1];
	if (abs(t[2] * R.m[1][0] - t[1] * R.m[2][0]) > ra + rb) return 0;
	else {
		//Non separating axis, record penetration
		float curPenetration = (ra + rb) - abs(t[2] * R.m[1][0] - t[1] * R.m[2][0]);
		if (curPenetration < m_penetrationDepth) {
			m_penetrationDepth = curPenetration;
			m_axisOfMinimumPenetration = au[0].Cross(AbsR.Right());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = au[0];
			m_edge2Dir = AbsR.Right();
		}
	}


	// Test axis L = A0 x B1 
	ra = ae[1] * AbsR.m[2][1] + ae[2] * AbsR.m[1][1];
	rb = be[0] * AbsR.m[0][2] + be[2] * AbsR.m[0][0];
	if (abs(t[2] * R.m[1][1] - t[1] * R.m[2][1]) > ra + rb) return 0;
	else {
		//Non separating axis, record penetration
		float curPenetration = (ra + rb) - abs(t[2] * R.m[1][1] - t[1] * R.m[2][1]);
		if (curPenetration < m_penetrationDepth) {
			m_penetrationDepth = curPenetration;
			m_axisOfMinimumPenetration = au[0].Cross(AbsR.Up());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = au[0];
			m_edge2Dir = AbsR.Up();
		}
	}

	// Test axis L = A0 x B2 
	ra = ae[1] * AbsR.m[2][2] + ae[2] * AbsR.m[1][2];
	rb = be[0] * AbsR.m[0][1] + be[1] * AbsR.m[0][0];
	if (abs(t[2] * R.m[1][2] - t[1] * R.m[2][2]) > ra + rb) return 0;
	else {
		//Non separating axis, record penetration
		float curPenetration = (ra + rb) - abs(t[2] * R.m[1][2] - t[1] * R.m[2][2]);
		if (curPenetration < m_penetrationDepth) {
			m_penetrationDepth = curPenetration;
			m_axisOfMinimumPenetration = au[0].Cross(AbsR.Forward());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = au[0];
			m_edge2Dir = AbsR.Forward();
		}
	}

	// Test axis L = A1 x B0 
	ra = ae[0] * AbsR.m[2][0] + ae[2] * AbsR.m[0][0];
	rb = be[1] * AbsR.m[1][2] + be[2] * AbsR.m[1][1];
	if (abs(t[0] * R.m[2][0] - t[2] * R.m[0][0]) > ra + rb) return 0;
	else {
		//Non separating axis, record penetration
		float curPenetration = (ra + rb) - abs(t[0] * R.m[2][0] - t[2] * R.m[0][0]);
		if (curPenetration < m_penetrationDepth) {
			m_penetrationDepth = curPenetration;
			m_axisOfMinimumPenetration = au[1].Cross(AbsR.Right());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = au[1];
			m_edge2Dir = AbsR.Right();
		}
	}

	// Test axis L = A1 x B1 
	ra = ae[0] * AbsR.m[2][1] + ae[2] * AbsR.m[0][1];
	rb = be[0] * AbsR.m[1][2] + be[2] * AbsR.m[1][0];
	if (abs(t[0] * R.m[2][1] - t[2] * R.m[0][1]) > ra + rb) return 0;
	else {
		//Non separating axis, record penetration
		float curPenetration = (ra + rb) - abs(t[0] * R.m[2][1] - t[2] * R.m[0][1]);
		if (curPenetration < m_penetrationDepth) {
			m_penetrationDepth = curPenetration;
			m_axisOfMinimumPenetration = au[1].Cross(AbsR.Up());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = au[1];
			m_edge2Dir = AbsR.Up();
		}
	}

	// Test axis L = A1 x B2 
	ra = ae[0] * AbsR.m[2][2] + ae[2] * AbsR.m[0][2];
	rb = be[0] * AbsR.m[1][1] + be[1] * AbsR.m[1][0];
	if (abs(t[0] * R.m[2][2] - t[2] * R.m[0][2]) > ra + rb) return 0;
	else {
		//Non separating axis, record penetration
		float curPenetration = (ra + rb) - abs(t[0] * R.m[2][2] - t[2] * R.m[0][2]);
		if (curPenetration < m_penetrationDepth) {
			m_penetrationDepth = curPenetration;
			m_axisOfMinimumPenetration = au[1].Cross(AbsR.Forward());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = au[1];
			m_edge2Dir = AbsR.Forward();
		}
	}

	// Test axis L = A2 x B0 
	ra = ae[0] * AbsR.m[1][0] + ae[1] * AbsR.m[0][0];
	rb = be[1] * AbsR.m[2][2] + be[2] * AbsR.m[2][1];
	if (abs(t[1] * R.m[0][0] - t[0] * R.m[1][0]) > ra + rb) return 0;
	else {
		//Non separating axis, record penetration
		float curPenetration = (ra + rb) - abs(t[1] * R.m[0][0] - t[0] * R.m[1][0]);
		if (curPenetration < m_penetrationDepth) {
			m_penetrationDepth = curPenetration;
			m_axisOfMinimumPenetration = au[2].Cross(AbsR.Right());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = au[2];
			m_edge2Dir = AbsR.Right();
		}
	}

	// Test axis L = A2 x B1 
	ra = ae[0] * AbsR.m[1][1] + ae[1] * AbsR.m[0][1];
	rb = be[0] * AbsR.m[2][2] + be[2] * AbsR.m[2][0];
	if (abs(t[1] * R.m[0][1] - t[0] * R.m[1][1]) > ra + rb) return 0;
	else {
		//Non separating axis, record penetration
		float curPenetration = (ra + rb) - abs(t[1] * R.m[0][1] - t[0] * R.m[1][1]);
		if (curPenetration < m_penetrationDepth) {
			m_penetrationDepth = curPenetration;
			m_axisOfMinimumPenetration = au[2].Cross(AbsR.Up());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = au[2];
			m_edge2Dir = AbsR.Up();
		}
	}

	// Test axis L = A2 x B2 
	ra = ae[0] * AbsR.m[1][2] + ae[1] * AbsR.m[0][2];
	rb = be[0] * AbsR.m[2][1] + be[1] * AbsR.m[2][0];
	if (abs(t[1] * R.m[0][2] - t[0] * R.m[1][2]) > ra + rb) return 0;
	else {
		//Non separating axis, record penetration
		float curPenetration = (ra + rb) - abs(t[1] * R.m[0][2] - t[0] * R.m[1][2]);
		if (curPenetration < m_penetrationDepth) {
			m_penetrationDepth = curPenetration;
			m_axisOfMinimumPenetration = au[2].Cross(AbsR.Forward());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = au[2];
			m_edge2Dir = AbsR.Forward();
		}
	}

	/*DERIVE MANIFOLD: OPEN DYNAMICS ENGINE:
	// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `return_code' returns a number indicating the type of contact that was
// detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.


int dBoxBox (const dVector3 p1, const dMatrix3 R1,
             const dVector3 side1, const dVector3 p2,
             const dMatrix3 R2, const dVector3 side2,
             dVector3 normal, dReal *depth, int *return_code,
             int flags, dContactGeom *contact, int skip)
{
    const dReal fudge_factor = REAL(1.05);
    dVector3 p,pp,normalC={0,0,0};
    const dReal *normalR = 0;
    dReal A[3],B[3],R11,R12,R13,R21,R22,R23,R31,R32,R33,
        Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33,s,s2,l,expr1_val;
    int i,j,invert_normal,code;

    // get vector from centers of box 1 to box 2, relative to box 1
    p[0] = p2[0] - p1[0];
    p[1] = p2[1] - p1[1];
    p[2] = p2[2] - p1[2];
    dMultiply1_331 (pp,R1,p);		// get pp = p relative to body 1

    // get side lengths / 2
    A[0] = side1[0]*REAL(0.5);
    A[1] = side1[1]*REAL(0.5);
    A[2] = side1[2]*REAL(0.5);
    B[0] = side2[0]*REAL(0.5);
    B[1] = side2[1]*REAL(0.5);
    B[2] = side2[2]*REAL(0.5);

    // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
    R11 = dCalcVectorDot3_44(R1+0,R2+0); R12 = dCalcVectorDot3_44(R1+0,R2+1); R13 = dCalcVectorDot3_44(R1+0,R2+2);
    R21 = dCalcVectorDot3_44(R1+1,R2+0); R22 = dCalcVectorDot3_44(R1+1,R2+1); R23 = dCalcVectorDot3_44(R1+1,R2+2);
    R31 = dCalcVectorDot3_44(R1+2,R2+0); R32 = dCalcVectorDot3_44(R1+2,R2+1); R33 = dCalcVectorDot3_44(R1+2,R2+2);

    Q11 = dFabs(R11); Q12 = dFabs(R12); Q13 = dFabs(R13);
    Q21 = dFabs(R21); Q22 = dFabs(R22); Q23 = dFabs(R23);
    Q31 = dFabs(R31); Q32 = dFabs(R32); Q33 = dFabs(R33);

    // for all 15 possible separating axes:
    //   * see if the axis separates the boxes. if so, return 0.
    //   * find the depth of the penetration along the separating axis (s2)
    //   * if this is the largest depth so far, record it.
    // the normal vector will be set to the separating axis with the smallest
    // depth. note: normalR is set to point to a column of R1 or R2 if that is
    // the smallest depth normal so far. otherwise normalR is 0 and normalC is
    // set to a vector relative to body 1. invert_normal is 1 if the sign of
    // the normal should be flipped.

    do {
#define TST(expr1,expr2,norm,cc) \
	expr1_val = (expr1); /* Avoid duplicate evaluation of expr1  \
		s2 = dFabs(expr1_val) - (expr2); \
		if (s2 > 0) return 0; \
			if (s2 > s) {
				\
					s = s2; \
					normalR = norm; \
					invert_normal = ((expr1_val) < 0); \
					code = (cc); \
					if (flags & CONTACTS_UNIMPORTANT) break; \
			}

	s = -dInfinity;
	invert_normal = 0;
	code = 0;

	// separating axis = u1,u2,u3
	TST(pp[0], (A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13), R1 + 0, 1);
	TST(pp[1], (A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23), R1 + 1, 2);
	TST(pp[2], (A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33), R1 + 2, 3);

	// separating axis = v1,v2,v3
	TST(dCalcVectorDot3_41(R2 + 0, p), (A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]), R2 + 0, 4);
	TST(dCalcVectorDot3_41(R2 + 1, p), (A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1]), R2 + 1, 5);
	TST(dCalcVectorDot3_41(R2 + 2, p), (A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2]), R2 + 2, 6);

	// note: cross product axes need to be scaled when s is computed.
	// normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1,expr2,n1,n2,n3,cc) \
    expr1_val = (expr1); /* Avoid duplicate evaluation of expr1  \
    s2 = dFabs(expr1_val) - (expr2); \
    if (s2 > 0) return 0; \
    l = dSqrt ((n1)*(n1) + (n2)*(n2) + (n3)*(n3)); \
    if (l > 0) { \
    s2 /= l; \
    if (s2*fudge_factor > s) { \
    s = s2; \
    normalR = 0; \
    normalC[0] = (n1)/l; normalC[1] = (n2)/l; normalC[2] = (n3)/l; \
    invert_normal = ((expr1_val) < 0); \
    code = (cc); \
    if (flags & CONTACTS_UNIMPORTANT) break; \
    } \
    }

		// We only need to check 3 edges per box 
		// since parallel edges are equivalent.

		// separating axis = u1 x (v1,v2,v3)
	TST(pp[2] * R21 - pp[1] * R31, (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12), 0, -R31, R21, 7);
	TST(pp[2] * R22 - pp[1] * R32, (A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11), 0, -R32, R22, 8);
	TST(pp[2] * R23 - pp[1] * R33, (A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11), 0, -R33, R23, 9);

	// separating axis = u2 x (v1,v2,v3)
	TST(pp[0] * R31 - pp[2] * R11, (A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22), R31, 0, -R11, 10);
	TST(pp[0] * R32 - pp[2] * R12, (A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21), R32, 0, -R12, 11);
	TST(pp[0] * R33 - pp[2] * R13, (A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21), R33, 0, -R13, 12);

	// separating axis = u3 x (v1,v2,v3)
	TST(pp[1] * R11 - pp[0] * R21, (A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32), -R21, R11, 0, 13);
	TST(pp[1] * R12 - pp[0] * R22, (A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31), -R22, R12, 0, 14);
	TST(pp[1] * R13 - pp[0] * R23, (A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31), -R23, R13, 0, 15);
#undef TST
	} while (0);

	if (!code) return 0;

	// if we get to this point, the boxes interpenetrate. compute the normal
	// in global coordinates.
	if (normalR) {
		normal[0] = normalR[0];
		normal[1] = normalR[4];
		normal[2] = normalR[8];
	}
	else {
		dMultiply0_331(normal, R1, normalC);
	}
	if (invert_normal) {
		normal[0] = -normal[0];
		normal[1] = -normal[1];
		normal[2] = -normal[2];
	}
	*depth = -s;

	// compute contact point(s)

	if (code > 6) {
		// An edge from box 1 touches an edge from box 2.
		// find a point pa on the intersecting edge of box 1
		dVector3 pa;
		dReal sign;
		// Copy p1 into pa
		for (i = 0; i < 3; i++) pa[i] = p1[i]; // why no memcpy?
		// Get world position of p2 into pa
		for (j = 0; j < 3; j++) {
			sign = (dCalcVectorDot3_14(normal, R1 + j) > 0) ? REAL(1.0) : REAL(-1.0);
			for (i = 0; i < 3; i++) pa[i] += sign * A[j] * R1[i * 4 + j];
		}

		// find a point pb on the intersecting edge of box 2
		dVector3 pb;
		// Copy p2 into pb
		for (i = 0; i < 3; i++) pb[i] = p2[i]; // why no memcpy?
		// Get world position of p2 into pb
		for (j = 0; j < 3; j++) {
			sign = (dCalcVectorDot3_14(normal, R2 + j) > 0) ? REAL(-1.0) : REAL(1.0);
			for (i = 0; i < 3; i++) pb[i] += sign * B[j] * R2[i * 4 + j];
		}

		dReal alpha, beta;
		dVector3 ua, ub;
		// Get direction of first edge
		for (i = 0; i < 3; i++) ua[i] = R1[((code)-7) / 3 + i * 4];
		// Get direction of second edge
		for (i = 0; i < 3; i++) ub[i] = R2[((code)-7) % 3 + i * 4];
		// Get closest points between edges (one at each)
		dLineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
		for (i = 0; i < 3; i++) pa[i] += ua[i] * alpha;
		for (i = 0; i < 3; i++) pb[i] += ub[i] * beta;
		// Set the contact point as halfway between the 2 closest points
		for (i = 0; i < 3; i++) contact[0].pos[i] = REAL(0.5)*(pa[i] + pb[i]);
		contact[0].depth = *depth;
		*return_code = code;
		return 1;
	}

	// okay, we have a face-something intersection (because the separating
	// axis is perpendicular to a face). define face 'a' to be the reference
	// face (i.e. the normal vector is perpendicular to this) and face 'b' to be
	// the incident face (the closest face of the other box).
	// Note: Unmodified parameter values are being used here
	const dReal *Ra, *Rb, *pa, *pb, *Sa, *Sb;
	if (code <= 3) { // One of the faces of box 1 is the reference face
		Ra = R1; // Rotation of 'a'
		Rb = R2; // Rotation of 'b'
		pa = p1; // Center (location) of 'a'
		pb = p2; // Center (location) of 'b'
		Sa = A;  // Side Lenght of 'a'
		Sb = B;  // Side Lenght of 'b'
	}
	else { // One of the faces of box 2 is the reference face
		Ra = R2; // Rotation of 'a'
		Rb = R1; // Rotation of 'b'
		pa = p2; // Center (location) of 'a'
		pb = p1; // Center (location) of 'b'
		Sa = B;  // Side Lenght of 'a'
		Sb = A;  // Side Lenght of 'b'
	}

	// nr = normal vector of reference face dotted with axes of incident box.
	// anr = absolute values of nr.
	/*
	The normal is flipped if necessary so it always points outward from box 'a',
	box 'b' is thus always the incident box
	
	dVector3 normal2, nr, anr;
	if (code <= 3) {
		normal2[0] = normal[0];
		normal2[1] = normal[1];
		normal2[2] = normal[2];
	}
	else {
		normal2[0] = -normal[0];
		normal2[1] = -normal[1];
		normal2[2] = -normal[2];
	}
	// Rotate normal2 in incident box opposite direction
	dMultiply1_331(nr, Rb, normal2);
	anr[0] = dFabs(nr[0]);
	anr[1] = dFabs(nr[1]);
	anr[2] = dFabs(nr[2]);

	// find the largest compontent of anr: this corresponds to the normal
	// for the incident face. the other axis numbers of the incident face
	// are stored in a1,a2.
	int lanr, a1, a2;
	if (anr[1] > anr[0]) {
		if (anr[1] > anr[2]) {
			a1 = 0;
			lanr = 1;
			a2 = 2;
		}
		else {
			a1 = 0;
			a2 = 1;
			lanr = 2;
		}
	}
	else {
		if (anr[0] > anr[2]) {
			lanr = 0;
			a1 = 1;
			a2 = 2;
		}
		else {
			a1 = 0;
			a2 = 1;
			lanr = 2;
		}
	}

	// compute center point of incident face, in reference-face coordinates
	dVector3 center;
	if (nr[lanr] < 0) {
		for (i = 0; i < 3; i++) center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i * 4 + lanr];
	}
	else {
		for (i = 0; i < 3; i++) center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i * 4 + lanr];
	}

	// find the normal and non-normal axis numbers of the reference box
	int codeN, code1, code2;
	if (code <= 3) codeN = code - 1; else codeN = code - 4;
	if (codeN == 0) {
		code1 = 1;
		code2 = 2;
	}
	else if (codeN == 1) {
		code1 = 0;
		code2 = 2;
	}
	else {
		code1 = 0;
		code2 = 1;
	}

	// find the four corners of the incident face, in reference-face coordinates
	dReal quad[8];	// 2D coordinate of incident face (x,y pairs)
	dReal c1, c2, m11, m12, m21, m22;
	c1 = dCalcVectorDot3_14(center, Ra + code1);
	c2 = dCalcVectorDot3_14(center, Ra + code2);
	// optimize this? - we have already computed this data above, but it is not
	// stored in an easy-to-index format. for now it's quicker just to recompute
	// the four dot products.
	m11 = dCalcVectorDot3_44(Ra + code1, Rb + a1);
	m12 = dCalcVectorDot3_44(Ra + code1, Rb + a2);
	m21 = dCalcVectorDot3_44(Ra + code2, Rb + a1);
	m22 = dCalcVectorDot3_44(Ra + code2, Rb + a2);
	{
		dReal k1 = m11 * Sb[a1];
		dReal k2 = m21 * Sb[a1];
		dReal k3 = m12 * Sb[a2];
		dReal k4 = m22 * Sb[a2];
		quad[0] = c1 - k1 - k3;
		quad[1] = c2 - k2 - k4;
		quad[2] = c1 - k1 + k3;
		quad[3] = c2 - k2 + k4;
		quad[4] = c1 + k1 + k3;
		quad[5] = c2 + k2 + k4;
		quad[6] = c1 + k1 - k3;
		quad[7] = c2 + k2 - k4;
	}

	// find the size of the reference face
	dReal rect[2];
	rect[0] = Sa[code1];
	rect[1] = Sa[code2];

	// intersect the incident and reference faces
	dReal ret[16];
	int n = intersectRectQuad(rect, quad, ret);
	if (n < 1) return 0;		// this should never happen

	// convert the intersection points into reference-face coordinates,
	// and compute the contact position and depth for each point. only keep
	// those points that have a positive (penetrating) depth. delete points in
	// the 'ret' array as necessary so that 'point' and 'ret' correspond.
	dReal point[3 * 8];		// penetrating contact points
	dReal dep[8];			// depths for those points
	dReal det1 = dRecip(m11*m22 - m12 * m21);
	m11 *= det1;
	m12 *= det1;
	m21 *= det1;
	m22 *= det1;
	int cnum = 0;			// number of penetrating contact points found
	for (j = 0; j < n; j++) {
		dReal k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
		dReal k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
		for (i = 0; i < 3; i++) point[cnum * 3 + i] =
			center[i] + k1 * Rb[i * 4 + a1] + k2 * Rb[i * 4 + a2];
		dep[cnum] = Sa[codeN] - dCalcVectorDot3(normal2, point + cnum * 3);
		if (dep[cnum] >= 0) {
			ret[cnum * 2] = ret[j * 2];
			ret[cnum * 2 + 1] = ret[j * 2 + 1];
			cnum++;
			if ((cnum | CONTACTS_UNIMPORTANT) == (flags & (NUMC_MASK | CONTACTS_UNIMPORTANT))) {
				break;
			}
		}
	}
	if (cnum < 1) {
		return 0;	// this should not happen, yet does at times (demo_plane2d single precision).
	}

	// we can't generate more contacts than we actually have
	int maxc = flags & NUMC_MASK;
	if (maxc > cnum) maxc = cnum;
	if (maxc < 1) maxc = 1;	// Even though max count must not be zero this check is kept for backward compatibility as this is a public function

	if (cnum <= maxc) {
		// we have less contacts than we need, so we use them all
		for (j = 0; j < cnum; j++) {
			dContactGeom *con = CONTACT(contact, skip*j);
			for (i = 0; i < 3; i++) con->pos[i] = point[j * 3 + i] + pa[i];
			con->depth = dep[j];
		}
	}
	else {
		dIASSERT(!(flags & CONTACTS_UNIMPORTANT)); // cnum should be generated not greater than maxc so that "then" clause is executed
		// we have more contacts than are wanted, some of them must be culled.
		// find the deepest point, it is always the first contact.
		int i1 = 0;
		dReal maxdepth = dep[0];
		for (i = 1; i < cnum; i++) {
			if (dep[i] > maxdepth) {
				maxdepth = dep[i];
				i1 = i;
			}
		}

		int iret[8];
		cullPoints(cnum, ret, maxc, i1, iret);

		for (j = 0; j < maxc; j++) {
			dContactGeom *con = CONTACT(contact, skip*j);
			for (i = 0; i < 3; i++) con->pos[i] = point[iret[j] * 3 + i] + pa[i];
			con->depth = dep[iret[j]];
		}
		cnum = maxc;
	}

	*return_code = code;
	return cnum;
}*/
	// Since no separating axis is found, the OBBs must be intersecting
	//@Manifold generation
	//@Two approaches for querying contact data: Face to face contact and edge to edge
	//@We move all necessary data to global space, and we solve for our contacts THERE
	//@WE SOLVE IN LOCAL SPACE
	//https://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/
	if (m_isFaceToFaceCollision) {
		//Identify axis of minimum penetration using the SAT (This defines the reference face)
		//Find the most anti parallel face on other shape(this defines the incident face)
		//Clipping incident face to reference face (Sutherman-Hodgmann clipping)
		//Incident face is the one which most faces the axisOfminimumPenetration (dot product is smaller)
		m_axisOfMinimumPenetration.Normalize();
		Plane referencePlane;//Plane in local space
		if (m_isFace1) {
			//m_axisOfMinimumPenetration comes from rb1
			Vector3 oneToTwo = t2.m_position - t1.m_position;
			if (oneToTwo.Dot(m_axisOfMinimumPenetration) < 0) m_axisOfMinimumPenetration *= -1;
			referencePlane = Plane(m_axisOfMinimumPenetration, ae[m_faceIndex]);
			//We need to find the incident face, that is, the one that most faces (Most negative dot product) the axisOfMinimumPenetration, in rb2
			Vector3 possibleFaces[6] = { AbsR.Right(), AbsR.Up(), AbsR.Forward() };
			float minDot = FLT_MAX;
			Vector3 incidentFaceNormal;
			unsigned int incidentFaceIndex;
			for (int i = 0; i < 6; i++) {
				float currentDot = m_axisOfMinimumPenetration.Dot(possibleFaces[i]);
				if (currentDot < minDot) {
					minDot = currentDot;
					incidentFaceNormal = possibleFaces[i];
					incidentFaceIndex = i;
				}
			}

			Vector3 centreOfFace;

			for (int i = 0; i < 3; i++) {
				
			}
			//@We need more data: The points that define the face whose outward normal = incidentFaceNormal
			//@The orthonormal planes to the reference plane, to define our area for Sutherland-Clipping

			//@GENERATE THE FACE POINTS
		}
		else {
			//m_axisOfMinimumPenetration comes from rb2
			Vector3 twoToOne = t1.m_position - t2.m_position;
			if (twoToOne.Dot(m_axisOfMinimumPenetration) < 0) m_axisOfMinimumPenetration *= -1;
			referencePlane = Plane(m_axisOfMinimumPenetration, be[m_faceIndex]);
			//We need to find the incident face, that is, the one that most faces (Most negative dot product) the axisOfMinimumPenetration, in rb1
			Vector3 possibleFaces[6] = { ma.Right(), ma.Left(), ma.Up(), ma.Down(), ma.Forward(), ma.Backward() };
			float minDot = FLT_MAX;
			Vector3 incidentFaceNormal;
			for (int i = 0; i < 6; i++) {
				float currentDot = m_axisOfMinimumPenetration.Dot(possibleFaces[i]);
				if (currentDot < minDot) {
					minDot = currentDot;
					incidentFaceNormal = possibleFaces[i];
				}
			}
		}

		//Now m_axisOfMinimumPenetration points the right place
		//We can get the reference face, and the incident face.
		//We will also need the clipping planes, that are normal to the reference face
		
	}
	else {
		//m_axisOfMinimumPenetration = Vector3::Transform(m_axisOfMinimumPenetration, AbsR.Invert());SOLVE IN LOCAL SPACE
		m_axisOfMinimumPenetration.Normalize();
		//@Support points / Supporting edges
		// We have the axes, but not the edges: each axis has 4 edges parallel
		// to it, we need to find which of the 4 for each object. We do
		// that by finding the point in the centre of the edge. We know
		// its component in the direction of the box's collision axis is zero
		// (its a mid-point) and we determine which of the extremes in each
		// of the other axes is closest.

		m_edge2Dir = Vector3::Transform(m_edge2Dir, AbsR.Invert());
		//Rotate axis through inverse R.
		//@Return contact point?
		QueryOBBEdgeContact(rb1, rb2, m_edge1Dir, m_edge2Dir, m_axisOfMinimumPenetration, m_penetrationDepth);
	}
	return true;
}
Vector3 PhysicSystem::QueryOBBEdgeContact(RigidbodyComponent * rb1, RigidbodyComponent * rb2, Vector3 edge1Dir, Vector3 edge2Dir,
	Vector3 axisOfMinimumPenetration, float penetrationDepth)
{
	//@Get contact point
	OrientedBoundingBox * a = dynamic_cast<OrientedBoundingBox*>(rb1->m_shape);//a
	OrientedBoundingBox * b = dynamic_cast<OrientedBoundingBox*>(rb2->m_shape);//b


	return Vector3();
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

float PhysicSystem::ClosestPtSegmentSegment(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2, Vector3 &p1, Vector3 &p2, float &f1, float &f2)
{
	//@From Christer Ericson's Real-Time Collision Detection
	/*
	// Clamp n to lie within the range [min, max] 
	float Clamp(float n, float min, float max) 
	{
		if (n < min) return min;
		if (n > max) return max;
		return n;
	}
	// Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and 
	// S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared 
	// distance between between S1(s) and S2(t) 
	float ClosestPtSegmentSegment(Point p1, Point q1, Point p2, Point q2, float &s, float &t, Point &c1, Point &c2) 
	{
		Vector d1 = q1 - p1; 
		// Direction vector of segment S1 
		Vector d2 = q2 - p2;
		// Direction vector of segment S2 
		Vector r = p1 - p2;
		float a = Dot(d1, d1);
		// Squared length of segment S1, always nonnegative 
		float e = Dot(d2, d2);
		// Squared length of segment S2, always nonnegative 
		float f = Dot(d2, r);
		// Check if either or both segments degenerate into points 
		if (a <= EPSILON && e <= EPSILON) 
		{ 
			// Both segments degenerate into points 
			s=t=0.0f; 
			c1 = p1;
			c2 = p2;
			return Dot(c1 - c2, c1 - c2);
		} 
		if (a <= EPSILON) 
		{
			// First segment degenerates into a point 
			s = 0.0f;
			t = f / e; // s = 0 => t =(b*s + f) / e = f / e 
			t = Clamp(t, 0.0f, 1.0f);
		} else 
		{
		float c = Dot(d1, r);
		if (e <= EPSILON) 
		{
			// Second segment degenerates into a point 
			t = 0.0f; s = Clamp(-c / a, 0.0f, 1.0f); // t = 0 => s =(b*t - c) / a = -c / a 
		} else 
		{
			// The general nondegenerate case starts here 
			float b = Dot(d1, d2);
			float denom = a*e-b*b; // Always nonnegative
			// If segments not parallel, compute closest point on L1 to L2 and 
			// clamp to segment S1. Else pick arbitrary s (here 0) 
			if (denom != 0.0f) 
			{
				s = Clamp((b*f - c*e) / denom, 0.0f, 1.0f);
			} 
			else s = 0.0f; 
			// Compute point on L2 closest to S1(s) using 
			// t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e 
			t = (b*s + f) / e;
			// If t in [0,1] done. Else clamp t, recompute s for the new value 
			// of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a 
			// and clamp s to [0, 1] 
			if (t < 0.0f) 
			{ 
				t = 0.0f;
				s = Clamp(-c / a, 0.0f, 1.0f);
			} else if (t > 1.0f) 
			{ 
				t = 1.0f;
				s = Clamp((b - c) / a, 0.0f, 1.0f);
			}
		}
	}
	c1 = p1 + d1 * s;
	c2 = p2 + d2 * t;
	return Dot(c1 - c2, c1 - c2);
	}
	*/

	//@FOR NOW WE WON'T HANDLE DEGENERATE CASES
	//Direction vector of segment a;
	Vector3 d1 = a2 - a1;
	//Dv of segment b
	Vector3 d2 = b2 - b1;
	
	Vector3 r = a2 - b2;

	float aSq = d1.LengthSquared();
	float bSq = d2.LengthSquared();

	float f = d2.Dot(r);
	float c = d1.Dot(r);
	// The general nondegenerate case starts here 
	float b = d1.Dot(d2);
	float denom = aSq * bSq - b * b; // Always nonnegative
	// If segments not parallel, compute closest point on L1 to L2 and 
	// clamp to segment S1. Else pick arbitrary s (here 0) 
	if (denom != 0.0f)
	{	
		//@Quick way to clamp: std::max(lower, std::min(n, upper))
			f1 = max(0.0f, min(b*f - c * bSq, 1.0f));
	}
	else f1 = 0.0f;
	// Compute point on L2 closest to S1(s) using 
	// t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e 
	f2 = (b*f1 + f) / bSq;
	// If t in [0,1] done. Else clamp t, recompute s for the new value 
	// of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a 
	// and clamp s to [0, 1] 
	if (f2 < 0.0f)
	{
		f2 = 0.0f;
		f1 = max(0.0f, min(-c / aSq, 1.0f));// Clamp(-c / aSq, 0.0f, 1.0f);
	}
	else if (f2 > 1.0f)
	{
		f2 = 1.0f;
		f1 = max(0.0f, min(b - c / aSq, 1.0f));// Clamp((b - c) / a, 0.0f, 1.0f);
	}

	p1 = a1 + d1 * f1;
	p2 = b1 + d2 * f2;
	Vector3 p2ToP1 = p1 - p2;

	return p2ToP1.LengthSquared();
}

DirectX::SimpleMath::Vector3 PhysicSystem::ClosestPtPointTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
{
	//@From Christer Ericson's Real-Time collision detection book

	//Check if P in vertex region outside A
	Vector3 ab = b - a;
	Vector3 ac = c - a;
	Vector3 ap = p - a;

	float d1 = ab.Dot(ap);
	float d2 = ac.Dot(ap);
	if (d1 <= 0.0f && d2 <= 0.0f)return a;//barycentric coordinates(1,0,0)

	//Check if P in vertex region outside B
	Vector3 bp = p - b;
	
	float d3 = ab.Dot(bp);
	float d4 = ac.Dot(bp);
	if (d3 >= 0.0f && d4 <= d3) return b;//barycentric coordinates(0,1,0)

	//Check if P in edge region of AB, if so return projection of P onto AB
	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		float v = d1 / (d1 - d3);
		return a + v * ab;//barycentric coordinates(1-v,v,0)
	}
	
	//Check if P in vertex region outside C
	Vector3 cp = p - c;
	float d5 = ab.Dot(cp);
	float d6 = ac.Dot(cp);
	if (d6 >= 0.0f && d5 <= d6)return c; //barycentric coordinates (0,0,1)
}

Vector3 PhysicSystem::ClosestPtPointTetrahedron(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
{
	// Start out assuming point inside all halfspaces, so closest to itself 
	Vector3 closestPt = p; float bestSqDist = FLT_MAX; // If point outside face abc then compute closest point on abc 
	Vector3 q = ClosestPtPointTriangle(p, a, b, c);
	float sqDist = (q-p).LengthSquared(); 
	// Update best closest point if (squared) distance is less than current best
	if (sqDist < bestSqDist) bestSqDist = sqDist, closestPt = q; 
	
	// Repeat test for face acd 
	Vector3 q2 = ClosestPtPointTriangle(p, a, c, d); 
	float sqDist2 = (q2-p).LengthSquared();
	if (sqDist2 < bestSqDist) bestSqDist = sqDist2, closestPt = q2;

	// Repeat test for face adb 
	Vector3 q3 = ClosestPtPointTriangle(p, a, d, b); 
	float sqDist3 = (q3-p).LengthSquared();
	if (sqDist3 < bestSqDist) bestSqDist = sqDist3, closestPt = q3;
	// Repeat test for face bdc 
	Vector3 q4 = ClosestPtPointTriangle(p, b, d, c);
	float sqDist4 = (q4 - p).LengthSquared();
	if (sqDist4 < bestSqDist) bestSqDist = sqDist4, closestPt = q4;
	
	return closestPt;
}
