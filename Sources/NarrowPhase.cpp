#include "pch.h"
#include "..\Headers\NarrowPhase.h"
#include "GameObject.h"
#include "Sphere.h"

using namespace std;
using namespace DirectX;
using namespace SimpleMath;

NarrowPhase::NarrowPhase()
{
}


NarrowPhase::~NarrowPhase()
{
}

void NarrowPhase::ApplyImpulse(DirectX::SimpleMath::Vector3 impulse, DirectX::SimpleMath::Vector3 contactVector)
{
}

bool NarrowPhase::SphereToSphere(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt)
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

		ContactManifold manifold;
		Vector3 contactPoint = surfacePoint1 + (surfacePoint2 - surfacePoint1)*0.5f;
		manifold.m_maxPenetration = overlap;
		manifold.m_rigidbodies.first = rb1;
		manifold.m_rigidbodies.second = rb2;
		manifold.m_points.push_back(contactPoint);
		manifold.m_normal = normal;
		m_solver.m_collidingPairs.push_back(manifold);
		return true;
	}
	return false;
}

bool NarrowPhase::SphereToOBB(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt)
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
		float overlap = (sphere1->m_radius - dist);//@Should always be positive value
		Vector3 normal = v / dist;

		ContactManifold manifold;
		manifold.m_maxPenetration = overlap;
		manifold.m_rigidbodies.first = rb1;
		manifold.m_rigidbodies.second = rb2;
		manifold.m_points.push_back(closestPoint);
		manifold.m_normal = normal;
		m_solver.m_collidingPairs.push_back(manifold);

		return true;
	}
	return false;
}

bool NarrowPhase::OBBToOBB(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt)
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
	Vector3 localBasis[3] = { Matrix::Identity.Right(), Matrix::Identity.Up(), Matrix::Identity.Forward() };
	float ae[3] = { a->m_halfExtents.x, a->m_halfExtents.y, a->m_halfExtents.z };

	Matrix mb = Matrix::CreateFromQuaternion(rb2->m_owner->m_transform.m_rotation);
	Vector3 bu[3] = { mb.Right(), mb.Up(), mb.Forward() };//b.u
	float be[3] = { b->m_halfExtents.x, b->m_halfExtents.y, b->m_halfExtents.z };

	// Compute rotation matrix expressing b in a’s coordinate frame 
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R.m[i][j] = au[i].Dot(bu[j]);

	//R = ma * mb;

	// Compute translation vector t 
	Vector3 tv = rb2->m_owner->m_transform.m_position - rb1->m_owner->m_transform.m_position;
	// Bring translation into a’s coordinate frame 
	float t[3] = { tv.Dot(au[0]), tv.Dot(au[1]), tv.Dot(au[2]) };//@POSSIBLE ERRATA(YUP)
	Vector3 translationVector = Vector3(t[0], t[1], t[2]);//Translation vector from A's coordinate frame

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
	unsigned int m_edge1Index, m_edge2Index;//@Starts to be too much data?
	unsigned int m_faceIndex;

	//@Something to face case (Find incident face, reference face,..)
	// Test axes L = A0, L = A1, L = A2 
	for (int i = 0; i < 3; i++)
	{
		ra = ae[i];
		rb = be[0] * AbsR.m[i][0] + be[1] * AbsR.m[i][1] + be[2] * AbsR.m[i][2];
		if (abs(t[i]) > ra + rb) return 0;
		else {
			//Non separating axis, record penetration
			float curPenetration = (ra + rb) - abs(t[i]);
			if (curPenetration < m_penetrationDepth) {
				m_penetrationDepth = curPenetration;
				m_axisOfMinimumPenetration = localBasis[i];
				m_isFaceToFaceCollision = true;
				m_isFace1 = true;
				m_faceIndex = i;
			}
		}
	}

	//For solving in local space, then moving to global
	Vector3 bRotatedAxes[3] = { AbsR.Right(), AbsR.Up(), AbsR.Forward() };
	// Test axes L = B0, L = B1, L = B2 
	for (int i = 0; i < 3; i++)
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
			m_axisOfMinimumPenetration = localBasis[0].Cross(AbsR.Right());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = localBasis[0];
			m_edge1Index = 0;
			m_edge2Dir = AbsR.Right();
			m_edge2Index = 0;
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
			m_axisOfMinimumPenetration = localBasis[0].Cross(AbsR.Up());//@NOT IN LOCAL SPACE
			//IN LOCAL SPACE, THE EDGEA WOULD BE IDENTITY.RIGHT(). AbsR however should be right.
			m_isFaceToFaceCollision = false;
			m_edge1Dir = localBasis[0];
			m_edge1Index = 0;
			m_edge2Dir = AbsR.Up();
			m_edge2Index = 1;
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
			m_axisOfMinimumPenetration = localBasis[0].Cross(AbsR.Forward());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = localBasis[0];
			m_edge1Index = 0;
			m_edge2Dir = AbsR.Forward();
			m_edge2Index = 2;
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
			m_axisOfMinimumPenetration = localBasis[1].Cross(AbsR.Right());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = localBasis[1];
			m_edge1Index = 1;
			m_edge2Dir = AbsR.Right();
			m_edge2Index = 0;
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
			m_axisOfMinimumPenetration = localBasis[1].Cross(AbsR.Up());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = localBasis[1];
			m_edge1Index = 1;
			m_edge2Dir = AbsR.Up();
			m_edge2Index = 1;
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
			m_axisOfMinimumPenetration = localBasis[1].Cross(AbsR.Forward());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = localBasis[1];
			m_edge1Index = 1;
			m_edge2Dir = AbsR.Forward();
			m_edge2Index = 2;
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
			m_axisOfMinimumPenetration = localBasis[2].Cross(AbsR.Right());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = localBasis[2];
			m_edge1Index = 2;
			m_edge2Dir = AbsR.Right();
			m_edge2Index = 0;
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
			m_axisOfMinimumPenetration = localBasis[2].Cross(AbsR.Up());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = localBasis[2];
			m_edge1Index = 2;
			m_edge2Dir = AbsR.Up();
			m_edge2Index = 1;
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
			m_axisOfMinimumPenetration = localBasis[2].Cross(AbsR.Forward());
			m_isFaceToFaceCollision = false;
			m_edge1Dir = localBasis[2];
			m_edge1Index = 2;
			m_edge2Dir = AbsR.Forward();
			m_edge2Index = 2;
		}
	}

// Since no separating axis is found, the OBBs must be intersecting
//@Manifold generation
//@Two approaches for querying contact data: Face to face contact and edge to edge
//@We move all necessary data to global space, and we solve for our contacts THERE
//@WE SOLVE IN LOCAL SPACE
//https://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/


	ContactManifold manifold;
	manifold.m_maxPenetration = m_penetrationDepth;
	manifold.m_rigidbodies.first = rb1;
	manifold.m_rigidbodies.second = rb2;
	
	Matrix axisMatrix = Matrix::CreateTranslation(m_axisOfMinimumPenetration);
	axisMatrix *= ma.Invert();
	Vector3 globalAxis = axisMatrix.Translation();
	//Flip normal if we need to
	manifold.m_normal = (globalAxis.Dot(tv) > 0) ? (-globalAxis) : (globalAxis);//@We follow convention to point towards A

	if (m_isFaceToFaceCollision) {
		//Identify axis of minimum penetration using the SAT (This defines the reference face)
		//Find the most anti parallel face on other shape(this defines the incident face)
		//Clipping incident face to reference face (Sutherman-Hodgmann clipping)
		//Incident face is the one which most faces the axisOfminimumPenetration (dot product is smaller)
		Plane referencePlane;//Plane in local space
		Vector3 incidentFaceNormal;
		float direction;
		unsigned int incidentFaceIndex;
		Vector3 centreOfFace;
		Vector3 facePoints[4];
		//@The orthonormal planes to the reference plane, to define our area for Sutherland-Clipping (Their normals point inwards)
		Plane supportPlanes[4];

		if (m_isFace1) {
#pragma region Reference / Incident face
			//We want the axis to point outwards from OBB1
			m_axisOfMinimumPenetration = (m_axisOfMinimumPenetration.Dot(translationVector) >= 0) ? (m_axisOfMinimumPenetration) : (-m_axisOfMinimumPenetration);
			//@We flip the referencePlane for clipping
			referencePlane = Plane(-m_axisOfMinimumPenetration, -ae[m_faceIndex]);//We can also use au[m_faceIndex] instead of m_axisOfMinimumPenetration

			//We need to find the incident face, that is, the one that most faces (Most negative dot product) the axisOfMinimumPenetration, in rb2
			float minDot = FLT_MAX;
			for (int i = 0; i < 3; i++) {
				//@Each basis corresponds to two, opposite face normals
				//Face1
				float currentDot = m_axisOfMinimumPenetration.Dot(bRotatedAxes[i]);
				if (currentDot < minDot) {
					minDot = currentDot;
					incidentFaceNormal = bRotatedAxes[i];
					incidentFaceIndex = i;
					direction = 1;
				}
				//Face2
				currentDot = m_axisOfMinimumPenetration.Dot(-bRotatedAxes[i]);
				if (currentDot < minDot) {
					minDot = currentDot;
					incidentFaceNormal = -bRotatedAxes[i];
					incidentFaceIndex = i;
					direction = -1;
				}
			}
			//tv is our translation vector for object2, i.e, the centerpoint of OBB2
			centreOfFace = translationVector + direction * bRotatedAxes[incidentFaceIndex] * be[incidentFaceIndex];
			//@We need more data: The points that define the face whose outward normal = incidentFaceNormal
			switch (incidentFaceIndex) {
			case 0:
			{
				//X
				/*DebugVector3 Up = AbsR.Up();
				Vector3 Down = AbsR.Down();
				Vector3 Right = AbsR.Right();
				Vector3 Left = AbsR.Left();
				Vector3 Forward = AbsR.Forward();
				Vector3 Backward = AbsR.Backward();*/
				//We want to get to the top back vector, meaning we'll use be.y and be.z as magnitude multipliers
				//The direction comes from AbsR, the rotated matrix
				facePoints[0] = centreOfFace + AbsR.Up()*b->m_halfExtents.y + AbsR.Backward()*b->m_halfExtents.z;

				//TOP front: We derive it from previous point, top back
				facePoints[1] = facePoints[0] + AbsR.Forward()*b->m_halfExtents.z * 2;

				//Bottom front: from previous
				facePoints[2] = facePoints[1] + AbsR.Down()*b->m_halfExtents.y * 2;

				//Bottom back: from previous
				facePoints[3] = facePoints[2] + AbsR.Backward()*b->m_halfExtents.z * 2;


			}
			break;
			case 1:
				//Y
				//Get left front
				facePoints[0] = centreOfFace + AbsR.Forward()*b->m_halfExtents.z + AbsR.Left()*b->m_halfExtents.x;

				//Left back
				facePoints[1] = facePoints[0] + AbsR.Backward()*b->m_halfExtents.z * 2;

				//Right back
				facePoints[2] = facePoints[1] + AbsR.Right()*b->m_halfExtents.x * 2;

				//Right front
				facePoints[3] = facePoints[2] + AbsR.Forward()*b->m_halfExtents.z * 2;
				break;
			case 2:
				//Z
				//Get top left
				facePoints[0] = centreOfFace + AbsR.Up()*b->m_halfExtents.y + AbsR.Left()*b->m_halfExtents.x;

				//Top right
				facePoints[1] = facePoints[0] + AbsR.Right()*b->m_halfExtents.x * 2;

				//Bottom right
				facePoints[2] = facePoints[1] + AbsR.Down()*b->m_halfExtents.y * 2;

				//Bottom left
				facePoints[3] = facePoints[2] + AbsR.Left()*b->m_halfExtents.x * 2;
				break;
			}
#pragma endregion

#pragma region Reference planes
			switch (m_faceIndex) {
			case 0:
				//orthonormal planes to x plane
				supportPlanes[0] = Plane(-localBasis[1], -ae[1]);//Plane pointing down, at the top of the OBB
				supportPlanes[1] = Plane(localBasis[1], -ae[1]);//Plane pointing up, at the bottom of the OBB
				supportPlanes[2] = Plane(localBasis[2], -ae[2]);//Plane pointing forward, at the back of the OBB
				supportPlanes[3] = Plane(-localBasis[2], -ae[2]);//Plane pointing backward, at the front of the OBB
				break;
			case 1:
				//orthonormal planes to y plane
				supportPlanes[0] = Plane(-localBasis[0], -ae[0]);//Plane pointing left, at the right of the OBB
				supportPlanes[1] = Plane(localBasis[0], -ae[0]);//Plane pointing right, at the left of the OBB
				supportPlanes[2] = Plane(localBasis[2], -ae[2]);//Plane pointing forward, at the back of the OBB
				supportPlanes[3] = Plane(-localBasis[2], -ae[2]);//Plane pointing backwards, at the front of the OBB
				break;
			case 2:
				//orthonormal planes to z plane
				supportPlanes[0] = Plane(-localBasis[0], -ae[0]);//Plane pointing left, at the right of the OBB
				supportPlanes[1] = Plane(localBasis[0], -ae[0]);//Plane pointing right, at the left of the OBB
				supportPlanes[2] = Plane(-localBasis[1], -ae[1]);//Plane pointing down, at the top of the OBB
				supportPlanes[3] = Plane(localBasis[1], -ae[1]);//Plane pointing upwards, at the bottom of the OBB
				break;
			}

#pragma endregion
		}
		else {
			//m_axisOfMinimumPenetration comes from rb2
			//We want the axis to point outwards from OBB2
			m_axisOfMinimumPenetration = (m_axisOfMinimumPenetration.Dot(translationVector) > 0) ? (-m_axisOfMinimumPenetration) : (m_axisOfMinimumPenetration);
			//Right now we want to find the right normal direction to get the reference plane
			referencePlane = Plane(translationVector + m_axisOfMinimumPenetration * be[m_faceIndex], -m_axisOfMinimumPenetration);//We flip the normal for later clipping algorithm

			//We need to find the incident face, that is, the one that most faces (Most negative dot product) the axisOfMinimumPenetration, in rb1
			float minDot = FLT_MAX;
			for (int i = 0; i < 3; i++) {
				//@Each basis corresponds to two, opposite face normals
				//Face1
				float currentDot = m_axisOfMinimumPenetration.Dot(localBasis[i]);
				if (currentDot < minDot) {
					minDot = currentDot;
					incidentFaceNormal = localBasis[i];
					incidentFaceIndex = i;
					direction = 1;
				}
				//Face2
				currentDot = m_axisOfMinimumPenetration.Dot(-localBasis[i]);
				if (currentDot < minDot) {
					minDot = currentDot;
					incidentFaceNormal = -localBasis[i];
					incidentFaceIndex = i;
					direction = -1;
				}
			}
			//tv is our translation vector for object2, i.e, the centerpoint of OBB2
			//Vector3 translationVector = Vector3(t[0], t[1], t[2]);//Translation vector from A's coordinate frame
			centreOfFace = direction * localBasis[incidentFaceIndex] * ae[incidentFaceIndex];//Centre is at origin now, because its on object A
			//@We need more data: The points that define the face whose outward normal = incidentFaceNormal
			switch (incidentFaceIndex) {
			case 0:
			{
				//X
				/*DebugVector3 Up = AbsMatrix::Identity.Up();
				Vector3 Down = AbsMatrix::Identity.Down();
				Vector3 Right = AbsMatrix::Identity.Right();
				Vector3 Left = AbsMatrix::Identity.Left();
				Vector3 Forward = AbsMatrix::Identity.Forward();
				Vector3 Backward = AbsMatrix::Identity.Backward();*/
				//We want to get to the top back vector, meaning we'll use be.y and be.z as magnitude multipliers
				//The direction comes from AbsR, the rotated matrix
				facePoints[0] = centreOfFace + Matrix::Identity.Up()*a->m_halfExtents.y + Matrix::Identity.Backward()*a->m_halfExtents.z;

				//TOP front: We derive it from previous point, top back
				facePoints[1] = facePoints[0] + Matrix::Identity.Forward()*a->m_halfExtents.z * 2;

				//Bottom front: from previous
				facePoints[2] = facePoints[1] + Matrix::Identity.Down()*a->m_halfExtents.y * 2;

				//Bottom back: from previous
				facePoints[3] = facePoints[2] + Matrix::Identity.Backward()*a->m_halfExtents.z * 2;


			}
			break;
			case 1:
			{
				//Y
				//Get left front
				facePoints[0] = centreOfFace + Matrix::Identity.Forward()*a->m_halfExtents.z + Matrix::Identity.Left()*a->m_halfExtents.x;

				//Left back
				facePoints[1] = facePoints[0] + Matrix::Identity.Backward()*a->m_halfExtents.z * 2;

				//Right back
				facePoints[2] = facePoints[1] + Matrix::Identity.Right()*a->m_halfExtents.x * 2;

				//Right front
				facePoints[3] = facePoints[2] + Matrix::Identity.Forward()*a->m_halfExtents.z * 2;
			}
			break;
			case 2:
			{
				//Z
				//Get top left
				facePoints[0] = centreOfFace + Matrix::Identity.Up()*a->m_halfExtents.y + Matrix::Identity.Left()*a->m_halfExtents.x;

				//Top right
				facePoints[1] = facePoints[0] + Matrix::Identity.Right()*a->m_halfExtents.x * 2;

				//Bottom right
				facePoints[2] = facePoints[1] + Matrix::Identity.Down()*a->m_halfExtents.y * 2;

				//Bottom left
				facePoints[3] = facePoints[2] + Matrix::Identity.Left()*a->m_halfExtents.x * 2;
			}
			break;
			}

			switch (m_faceIndex) {
			case 0:
				//orthonormal planes to x plane
				supportPlanes[0] = Plane(translationVector + be[1] * AbsR.Up(), AbsR.Down());//Plane pointing down, at the top of the OBB
				supportPlanes[1] = Plane(translationVector + be[1] * AbsR.Down(), AbsR.Up());//Plane pointing up, at the bottom of the OBB
				supportPlanes[2] = Plane(translationVector + be[2] * AbsR.Backward(), AbsR.Forward());//Plane pointing forward, at the back of the OBB
				supportPlanes[3] = Plane(translationVector + be[2] * AbsR.Forward(), AbsR.Backward());//Plane pointing backward, at the front of the OBB
				break;
			case 1:
				//orthonormal planes to y plane
				supportPlanes[0] = Plane(translationVector + be[0] * AbsR.Right(), AbsR.Left());//Plane pointing left, at the right of the OBB
				supportPlanes[1] = Plane(translationVector + be[0] * AbsR.Left(), AbsR.Right());//Plane pointing right, at the left of the OBB
				supportPlanes[2] = Plane(translationVector + be[2] * AbsR.Backward(), AbsR.Forward());//Plane pointing forward, at the back of the OBB
				supportPlanes[3] = Plane(translationVector + be[2] * AbsR.Forward(), AbsR.Backward());//Plane pointing backwards, at the front of the OBB
				break;
			case 2:
				//orthonormal planes to z plane
				supportPlanes[0] = Plane(translationVector + be[0] * AbsR.Right(), AbsR.Left());//Plane pointing left, at the right of the OBB
				supportPlanes[1] = Plane(translationVector + be[0] * AbsR.Left(), AbsR.Right());//Plane pointing right, at the left of the OBB
				supportPlanes[2] = Plane(translationVector + be[1] * AbsR.Up(), AbsR.Down());//Plane pointing down, at the top of the OBB
				supportPlanes[3] = Plane(translationVector + be[1] * AbsR.Down(), AbsR.Up());//Plane pointing upwards, at the bottom of the OBB
				break;
			}

		}
		//We have all the data we need now
		vector<Vector3> contactPoints = OBBClip(facePoints, supportPlanes, referencePlane);
		vector<Vector3> finalContactPoints;

		if (contactPoints.size() == 0) return false;//@Just in case

		//Filter out all contactPoints on the reference plane's surface.
		for (unsigned int i = 0; i < contactPoints.size(); i++) {
			if (DistPointPlane(contactPoints[i], referencePlane) > 0) {
				finalContactPoints.push_back(ClosestPtPointPlane(contactPoints[i], referencePlane));
			}
		}
		//Bring final points to global space
		for (int i = 0; i < finalContactPoints.size(); i++) {
			Matrix pointMatrix = Matrix::CreateTranslation(finalContactPoints[i]);
			pointMatrix *= ma.Invert();
			Vector3 point = pointMatrix.Translation();
			point += t1.m_position;
			manifold.m_points.push_back(point);
		}
	}
	else {
		//SOLVE IN LOCAL SPACE
		//@Support points / Supporting edges
		// We have the axes, but not the edges: each axis has 4 edges parallel
		// to it, we need to find which of the 4 for each object. We do
		// that by finding the point in the centre of the edge. We know
		// its component in the direction of the box's collision axis is zero
		// (its a mid-point) and we determine which of the extremes in each
		// of the other axes is closest.

		//@Convention of axis pointing from A outwards 
		//m_axisOfMinimumPenetration comes from rb1
		Vector3 oneToTwo = Vector3(t[0],t[1],t[2]);
		//We want the axis to point outwards from OBB2
		m_axisOfMinimumPenetration = (m_axisOfMinimumPenetration.Dot(oneToTwo) >= 0) ? (m_axisOfMinimumPenetration) : (-m_axisOfMinimumPenetration);
		Vector3 edgeCentres[4];
		//We have data of edge directions normalizes, axis of penetration, edge indices.
		//@Working from assumptions.
		//@Find all edge centre points and compute the one farthest in m_axisOfMinimumPenetration's direction? (Biggest dot product?)

		switch (m_edge1Index) {
		
		case 0:
		{
			//X Component is zero for all centres
			//Top front
			edgeCentres[0] = Vector3(0, a->m_halfExtents.y, a->m_halfExtents.z);
			//Top back
			edgeCentres[1] = Vector3(0, a->m_halfExtents.y, -a->m_halfExtents.z);
			//Bottom front
			edgeCentres[2] = Vector3(0, -a->m_halfExtents.y, a->m_halfExtents.z);
			//Bottom back
			edgeCentres[3] = Vector3(0, -a->m_halfExtents.y, -a->m_halfExtents.z);
		}
		break;
		case 1:
			//Y Component is zero for all centres
			//Front right
			edgeCentres[0] = Vector3(a->m_halfExtents.x, 0, a->m_halfExtents.z);
			//Back right
			edgeCentres[1] = Vector3(a->m_halfExtents.x, 0, -a->m_halfExtents.z);
			//Front left
			edgeCentres[2] = Vector3(-a->m_halfExtents.x, 0, a->m_halfExtents.z);
			//Back left
			edgeCentres[3] = Vector3(-a->m_halfExtents.x, 0, -a->m_halfExtents.z);
		{
		}
		break;
		case 2:
		{
			//@Z component is zero for all centres
			//Right top
			edgeCentres[0] = Vector3(a->m_halfExtents.x, a->m_halfExtents.y, 0);
			//Right bottom
			edgeCentres[1] = Vector3(a->m_halfExtents.x, -a->m_halfExtents.y, 0);
			//Left top
			edgeCentres[2] = Vector3(-a->m_halfExtents.x, a->m_halfExtents.y, 0);
			//Left bottom
			edgeCentres[3] = Vector3(-a->m_halfExtents.x, -a->m_halfExtents.y, 0);
		}
		break;
		}

		Vector3 finalEdgeCentre;
		Vector3 e1p1;
		Vector3 e1p2;
		float maxDp = FLT_MIN;
		for (int i = 0; i < 4; i++) {
			//Find maximum dp.
			float currentDp = edgeCentres[i].Dot(oneToTwo);
			if (currentDp > maxDp) {
				maxDp = currentDp;
				finalEdgeCentre = edgeCentres[i];
			}
		}

		switch (m_edge1Index) {
		case 0:
		{
			e1p1 = Vector3(finalEdgeCentre.x + a->m_halfExtents.x, finalEdgeCentre.y, finalEdgeCentre.z);
			e1p2 = Vector3(finalEdgeCentre.x - a->m_halfExtents.x, finalEdgeCentre.y, finalEdgeCentre.z);
		}
		break;
		case 1:
		{
			e1p1 = Vector3(finalEdgeCentre.x, finalEdgeCentre.y + a->m_halfExtents.y, finalEdgeCentre.z);
			e1p2 = Vector3(finalEdgeCentre.x, finalEdgeCentre.y - a->m_halfExtents.y, finalEdgeCentre.z);
		}
		break;
		case 2:
		{
			e1p1 = Vector3(finalEdgeCentre.x, finalEdgeCentre.y, finalEdgeCentre.z + a->m_halfExtents.z);
			e1p2 = Vector3(finalEdgeCentre.x, finalEdgeCentre.y, finalEdgeCentre.z - a->m_halfExtents.z);
		}
		break;
		}

		Vector3 edge2Centres[4];
		//We have data of edge directions normalizes, axis of penetration, edge indices.
		//@Working from assumptions.
		//@Find all edge centre points and compute the one farthest in m_axisOfMinimumPenetration's direction? (Biggest dot product?)

		switch (m_edge2Index) {

		case 0:
		{
			//Top front
			edge2Centres[0] = oneToTwo + AbsR.Forward()*b->m_halfExtents.z + AbsR.Up()*b->m_halfExtents.y;
			//Top back
			edge2Centres[1] = edge2Centres[0] + AbsR.Backward()*b->m_halfExtents.z*2;
			//Bottom back
			edge2Centres[2] = edge2Centres[1] + AbsR.Down()*b->m_halfExtents.y*2;
			//Bottom front
			edge2Centres[3] = edge2Centres[2] * AbsR.Forward()*b->m_halfExtents.z*2;
		}
		break;
		case 1:
		{
			//Front right
			edge2Centres[0] = oneToTwo + AbsR.Forward()*b->m_halfExtents.z + AbsR.Right()*b->m_halfExtents.x;
			//Back right
			edge2Centres[1] = edge2Centres[0] + AbsR.Backward()*b->m_halfExtents.z*2;
			//Back left
			edge2Centres[2] = edge2Centres[1] + AbsR.Left()*b->m_halfExtents.x*2;
			//Front left
			edge2Centres[3] = edge2Centres[2] + AbsR.Forward()*b->m_halfExtents.z*2;
		}
		break;
		case 2:
		{
			//Right top
			edge2Centres[0] = oneToTwo + AbsR.Right()*b->m_halfExtents.x + AbsR.Up()*b->m_halfExtents.y;
			//Right bottom
			edge2Centres[1] = edge2Centres[0] + AbsR.Down()*b->m_halfExtents.y * 2;
			//Left bottom
			edge2Centres[2] = edge2Centres[1] + AbsR.Left()*b->m_halfExtents.x * 2;
			//Left top
			edge2Centres[3] = edge2Centres[2] + AbsR.Up()*b->m_halfExtents.y*2;
		}
		break;
		}

		//Find edge centre in OBB2
		Vector3 finalEdge2Centre;
		Vector3 e2p1;
		Vector3 e2p2;

		float minDp = FLT_MAX;
		for (int i = 0; i < 4; i++) {
			//Find minimum dp.
			float currentDp = edge2Centres[i].Dot(oneToTwo);
			if (currentDp < minDp) {
				minDp = currentDp;
				finalEdge2Centre = edge2Centres[i];
			}
		}

		switch (m_edge2Index) {
		case 0:
		{
			e2p1 = finalEdge2Centre + AbsR.Right()*b->m_halfExtents.x;
			e2p2 = finalEdge2Centre + AbsR.Left()*b->m_halfExtents.x;
		}
		break;
		case 1:
		{
			e2p1 = finalEdge2Centre + AbsR.Up()*b->m_halfExtents.y;
			e2p2 = finalEdge2Centre + AbsR.Down()*b->m_halfExtents.y;
		}
		break;
		case 2:
		{
			e2p1 = finalEdge2Centre + AbsR.Forward()*b->m_halfExtents.z;
			e2p2 = finalEdge2Centre + AbsR.Backward()*b->m_halfExtents.z;
		}
		break;
		}

		//Now we have all data we need to get ClosestPtBetween segments
		Vector3 p1, p2;
		float f1, f2;
		ClosestPtSegmentSegment(e1p1, e1p2, e2p1, e2p2, p1, p2, f1, f2);
		Vector3 avgPoint = p1 + (p2 - p1)*0.5f;

		//@Bring the contact point into Global space
		Matrix pointMatrix = Matrix::CreateTranslation(avgPoint);
		pointMatrix *= ma.Invert();
		avgPoint = pointMatrix.Translation();
		avgPoint += t1.m_position;

		manifold.m_points.push_back(avgPoint);
	}
	m_solver.m_collidingPairs.push_back(manifold);
	return true;
}

DirectX::SimpleMath::Vector3 NarrowPhase::FindIntersectionWithPlaneFromDistances(DirectX::SimpleMath::Vector3 start, DirectX::SimpleMath::Vector3 end, float d1, float d2)
{
	//@Using linear interpolation
	float absD = abs(d1);
	float absD2 = abs(d2);
	return Vector3::Lerp(start, end, absD / (absD + absD2));
}
//@Helpful queries
DirectX::SimpleMath::Vector3 NarrowPhase::ClosestPtPointPlane(DirectX::SimpleMath::Vector3 point, DirectX::SimpleMath::Plane plane)
{
	float t = plane.Normal().Dot(point) - plane.D();//@Assuming plane equation is normalized
	return point - t * plane.Normal();
}

float NarrowPhase::DistPointPlane(DirectX::SimpleMath::Vector3 q, Plane p)
{
	/*Christer Ericson's
	float DistPointPlane(Point q, Plane p)
	{
	// return Dot(q, p.n) - p.d;
	if plane equation normalized (||p.n||==1)
	return (Dot(p.n, q) - p.d) / Dot(p.n, p.n);
	}*/
	return q.Dot(p.Normal()) - p.D();

}

DirectX::SimpleMath::Vector3 NarrowPhase::ClosestPtPointOBB(Vector3 p, OrientedBoundingBox * b, Vector3 bc, Quaternion bRot)
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

float NarrowPhase::ClosestPtSegmentSegment(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2, Vector3 &p1, Vector3 &p2, float &f1, float &f2)
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

DirectX::SimpleMath::Vector3 NarrowPhase::ClosestPtPointTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
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

Vector3 NarrowPhase::ClosestPtPointTetrahedron(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
{
	// Start out assuming point inside all halfspaces, so closest to itself 
	Vector3 closestPt = p; float bestSqDist = FLT_MAX; // If point outside face abc then compute closest point on abc 
	Vector3 q = ClosestPtPointTriangle(p, a, b, c);
	float sqDist = (q - p).LengthSquared();
	// Update best closest point if (squared) distance is less than current best
	if (sqDist < bestSqDist) bestSqDist = sqDist, closestPt = q;

	// Repeat test for face acd 
	Vector3 q2 = ClosestPtPointTriangle(p, a, c, d);
	float sqDist2 = (q2 - p).LengthSquared();
	if (sqDist2 < bestSqDist) bestSqDist = sqDist2, closestPt = q2;

	// Repeat test for face adb 
	Vector3 q3 = ClosestPtPointTriangle(p, a, d, b);
	float sqDist3 = (q3 - p).LengthSquared();
	if (sqDist3 < bestSqDist) bestSqDist = sqDist3, closestPt = q3;
	// Repeat test for face bdc 
	Vector3 q4 = ClosestPtPointTriangle(p, b, d, c);
	float sqDist4 = (q4 - p).LengthSquared();
	if (sqDist4 < bestSqDist) bestSqDist = sqDist4, closestPt = q4;

	return closestPt;
}

vector<DirectX::SimpleMath::Vector3> NarrowPhase::OBBClip(DirectX::SimpleMath::Vector3 faceVertices[4], DirectX::SimpleMath::Plane supportPlanes[4], DirectX::SimpleMath::Plane referencePlane)
{
https://gamedevelopment.tutsplus.com/tutorials/understanding-sutherland-hodgman-clipping-for-physics-engines--gamedev-11917
/*Polygon SutherlandHodgman(const Polygon startingPolygon, Plane[] clippingPlanes)
{
	Polygon output = startingPolygon
		for each Plane clippingPlane in clippingPlanes
			input = output
			output.Clear()
			Vec2 startingPoint = input.Last()
			for each Vec2 endPoint in input
				if startingPoint and endPoint in front of clippingPlane
					out.push(endPoint)
				else if startingPoint in front and endPoint behind clippingPlane
					out.push(Intersection(clippingPlane, startingPoint, endPoint))
				else if startingPoint and endPoint behind clippingPlane
					out.push(Intersection(clippingPlane, startingPoint, endPoint))
					out.push(endPoint)
					endPoint = startingPoint
					return output
}*/
/*// InFront = plane.Distance( point ) > 0.0f
// Behind  = plane.Distance( point ) < 0.0f

Vec2 p1, p2;
ClipPlane plane;

case p1 InFront and p2 InFront
  push p2
case p1 InFront and p2 Behind
  push intersection
case p1 Behind and p2 InFront
  push intersection
  push p2
*/

	vector<DirectX::SimpleMath::Vector3> contactPoints;

	for (int i = 0; i < 4; i++) {
		//For each vertex
		Vector3 startingPoint = faceVertices[i];
		Vector3 endPoint = (i == 3) ? (faceVertices[0]) : (faceVertices[i + 1]);//@So we run the last line loop properly
		bool inOrOut1 = true;
		bool inOrOut2 = true;

		Vector3 intersectionPoint = Vector3(FLT_MAX, FLT_MAX, FLT_MAX);
		//@Clip against support planes
		for (int j = 0; j < 4; j++) {
			//For each plane
			float dist1 = DistPointPlane(startingPoint, supportPlanes[j]);
			bool localInOut1 = true;//@In/out of CURRENT plane
			bool localInOut2 = true;

			if (dist1 > 0) {
				//Do nothing, point is inside this support plane, but not necessarily any of the others
			}
			else {
				localInOut1 = false;
				inOrOut1 = false;
			}

			float dist2 = DistPointPlane(endPoint, supportPlanes[j]);
			if (dist2 > 0) {
				//Idem
			}
			else {
				localInOut2 = false;
				inOrOut2 = false;
			}

			if (localInOut1 != localInOut2) {
				//There is an intersection with this plane, we can find it with lerp
				//By using the ratio of the distances as the alpha for the point in the line
				//@If the intersectionPoint to this plane is smaller, then this is the one we keep, since multiple intersections aren't considered
				Vector3 curIntersection = FindIntersectionWithPlaneFromDistances(startingPoint, endPoint, dist1, dist2);
				if (curIntersection.Length() < intersectionPoint.Length()) {
					intersectionPoint = curIntersection;
				}
			}

		}

		//Clip against referencePlane
		bool localInOut1 = true;
		bool localInOut2 = true;
		float dist1 = DistPointPlane(startingPoint,referencePlane);
		if (dist1 > 0) {
			//Idem, its on the right side of the plane
		}
		else {
			inOrOut1 = false;
			localInOut1 = false;
		}

		float dist2 = DistPointPlane(endPoint, referencePlane);
		if (dist2 > 0) {
			//Idem
		}
		else {
			inOrOut2 = false;
			localInOut2 = false;
		}

		if (localInOut1 != localInOut2) {
			//There is an intersection with the plane, we can find it with lerp
			//By using the ratio of the distances as the alpha for the point in the line
			//@If the intersectionPoint to this plane is smaller, then this is the one we keep, since multiple intersections aren't considered
			Vector3 curIntersection = FindIntersectionWithPlaneFromDistances(startingPoint, endPoint, dist1, dist2);
			if (curIntersection.Length() < intersectionPoint.Length()) intersectionPoint = curIntersection;
		}

		//Now we know exactly whether each line point is inside or outside our set of planes, we know which ones to keep
		//Sutherland-hodgmann rules
		if (inOrOut1 && inOrOut2) {
			contactPoints.push_back(endPoint);
		}
		else if (inOrOut1 && !inOrOut2) {
			contactPoints.push_back(intersectionPoint);
		}
		else if (!inOrOut1 && inOrOut2) {
			contactPoints.push_back(intersectionPoint);
			contactPoints.push_back(endPoint);
		}
	}

	return contactPoints;
}