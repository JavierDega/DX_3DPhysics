#include "pch.h"
#include "..\Headers\Capsule.h"

using namespace DirectX;
using namespace SimpleMath;

Capsule::Capsule(float radius, Vector3 pos1, Vector3 pos2, XMVECTOR color)
	: m_radius(radius), m_pos1(pos1), m_pos2(pos2), Shape(ShapeType::CAPSULE, color)
{
}


Capsule::~Capsule()
{
}
