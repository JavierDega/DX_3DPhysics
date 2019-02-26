
#include "Shape.h"
class Capsule :
	public Shape
{
public:
	Capsule(float radius = 0.5f, DirectX::SimpleMath::Vector3 pos1 = DirectX::SimpleMath::Vector3::Zero, DirectX::SimpleMath::Vector3 pos2 = DirectX::SimpleMath::Vector3::Zero,
		DirectX::XMVECTOR color = DirectX::Colors::White);
	~Capsule();

	//Variables
	float m_radius;
	DirectX::SimpleMath::Vector3 m_pos1;
	DirectX::SimpleMath::Vector3 m_pos2;
};
