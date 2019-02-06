#ifndef ORIENTEDBOUNDINGBOX_H_
#define ORIENTEDBOUNDINGBOX_H_
#include "Shape.h"

class OrientedBoundingBox :
	public Shape
{
public:
	OrientedBoundingBox(DirectX::SimpleMath::Vector3 extents = DirectX::SimpleMath::Vector3::One, DirectX::XMVECTOR color = DirectX::Colors::White);
	virtual ~OrientedBoundingBox();

	//Variables
	DirectX::SimpleMath::Vector3 m_extents;//@Half extents
};

#endif /*ORIENTEDBOUNDINGBOX_H_*/