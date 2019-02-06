#ifndef SHAPE_H_
#define SHAPE_H_

//@Broad phase
//@BoundingVolumes
struct AABB {
	DirectX::SimpleMath::Vector3 m_minExtent;
	DirectX::SimpleMath::Vector3 m_maxExtent;
};


//@Finds the common ground between all collider types, including those used for broad phase.
class Shape
{
public:
	Shape(DirectX::XMVECTOR color = DirectX::Colors::White, DirectX::XMVECTOR AABBColor = DirectX::Colors::Red, DirectX::XMVECTOR sphereColor = DirectX::Colors::Red);
	virtual ~Shape();

	//Variables
	DirectX::SimpleMath::Color m_color;
	DirectX::SimpleMath::Color m_AABBColor;
	DirectX::SimpleMath::Color m_sphereColor;
	std::unique_ptr<DirectX::GeometricPrimitive> m_primitive;
	std::unique_ptr<DirectX::GeometricPrimitive> m_AABBPrimitive;
	std::unique_ptr<DirectX::GeometricPrimitive> m_spherePrimitive;
	AABB m_AABB;
};
#endif /*SHAPE_H_*/
