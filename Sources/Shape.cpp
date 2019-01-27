#include "pch.h"
#include "..\Headers\Shape.h"

using namespace DirectX;

Shape::Shape(XMVECTOR Color, XMVECTOR AABBColor)
	: m_color(Color), m_AABBColor(AABBColor)
{
}


Shape::~Shape()
{
}
