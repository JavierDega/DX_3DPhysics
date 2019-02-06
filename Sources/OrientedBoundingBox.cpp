#include "pch.h"
#include "..\Headers\OrientedBoundingBox.h"


using namespace DirectX;
using namespace SimpleMath;

OrientedBoundingBox::OrientedBoundingBox(Vector3 extents, XMVECTOR color)
	: m_extents(extents), Shape(color)
{
}


OrientedBoundingBox::~OrientedBoundingBox()
{
}

