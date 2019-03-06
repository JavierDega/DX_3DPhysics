#ifndef CONTACTSOLVER_H_
#define CONTACTSOLVER_H_
#include "RigidbodyComponent.h"
#include <vector>

struct ContactManifold {

	std::pair<RigidbodyComponent*, RigidbodyComponent*> m_rigidbodies;
	//@Four points tend to be enough for stable manifolds
	std::vector<DirectX::SimpleMath::Vector3> m_points;
	DirectX::SimpleMath::Vector3 m_normal;
	float m_maxPenetration;//Penetration value
};
//@Everything logged here is already 
class ContactSolver
{
public:
	ContactSolver();
	~ContactSolver();
	void Solve(float dt);

	//Variables
	std::vector<ContactManifold> m_collidingPairs;
	float m_frictionCoefficient;//@Same for all objects?
};
#endif /*CONTACTSOLVER_H_*/