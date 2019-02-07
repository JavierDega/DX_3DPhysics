#include "pch.h"
#include "..\Headers\Shape.h"

using namespace DirectX;

Shape::Shape( ShapeType type, XMVECTOR Color, XMVECTOR AABBColor, XMVECTOR sphereColor)
	: m_type(type), m_color(Color), m_AABBColor(AABBColor), m_sphereColor(sphereColor)
{

}


Shape::~Shape()
{
}
