#ifndef ORIENTEDBOUNDINGBOX_H_
#define ORIENTEDBOUNDINGBOX_H_
#include "Shape.h"

class OrientedBoundingBox :
	public Shape
{
public:
	OrientedBoundingBox(DirectX::SimpleMath::Vector3 halfExtents = DirectX::SimpleMath::Vector3::One, DirectX::XMVECTOR color = DirectX::Colors::White);
	virtual ~OrientedBoundingBox();

	//Variables
	DirectX::SimpleMath::Vector3 m_halfExtents;//@Half extents
};

#endif /*ORIENTEDBOUNDINGBOX_H_*/