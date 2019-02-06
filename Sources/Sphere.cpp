#include "pch.h"
#include "..\Headers\Sphere.h"


using namespace DirectX;
using namespace SimpleMath;

Sphere::Sphere(float radius, XMVECTOR color)
	: m_radius(radius), Shape(color)
{

}


Sphere::~Sphere()
{
}
