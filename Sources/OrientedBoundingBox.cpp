#include "pch.h"
#include "..\Headers\OrientedBoundingBox.h"


using namespace DirectX;
using namespace SimpleMath;

OrientedBoundingBox::OrientedBoundingBox(Vector3 halfExtents, XMVECTOR color)
	: m_halfExtents(halfExtents), Shape( ShapeType::OBB, color)
{
}


OrientedBoundingBox::~OrientedBoundingBox()
{
}

