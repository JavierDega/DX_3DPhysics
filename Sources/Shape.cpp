#include "pch.h"
#include "..\Headers\Shape.h"

using namespace DirectX;

Shape::Shape(XMVECTOR Color, XMVECTOR AABBColor, XMVECTOR sphereColor)
	: m_color(Color), m_AABBColor(AABBColor), m_sphereColor(sphereColor)
{

}


Shape::~Shape()
{
}
