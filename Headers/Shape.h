#ifndef SHAPE_H_
#define SHAPE_H_

struct AABB {
	DirectX::SimpleMath::Vector3 m_minExtent;
	DirectX::SimpleMath::Vector3 m_maxExtent;
};

//@Finds the common ground between all collider types, including those used for broad phase.
class Shape
{
public:
	Shape();
	virtual ~Shape();
	//Funcs
	virtual AABB ComputeAABB() = 0;

	//Variables
	DirectX::SimpleMath::Vector3 m_center;
	std::unique_ptr<DirectX::GeometricPrimitive> m_primitive;
};
#endif /*SHAPE_H_*/
