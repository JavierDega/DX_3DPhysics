#ifndef SPHERE_H_
#define SPHERE_H_
#include "Shape.h"

class Sphere :
	public Shape
{
public:
	Sphere(float radius = 0.5f, DirectX::XMVECTOR color = DirectX::Colors::White);
	~Sphere();
	virtual AABB ComputeAABB();
	//Variables
	float m_radius;
};
#endif /*SPHERE_H_*/
