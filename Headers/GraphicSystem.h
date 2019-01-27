#ifndef GRAPHICSYSTEM_H_
#define GRAPHICSYSTEM_H_

#include "System.h"
//#include "RigidbodyComponent.h"
#include <string>
#include <vector>

class GraphicSystem : public System {
private:
	/*Here will be the instance stored*/
	static GraphicSystem* m_instance;
	/*Private constructor to prevent instancing*/
	GraphicSystem();
public:
	~GraphicSystem();
	//Singleton
	static GraphicSystem* GetInstance();

	///Functions
	//Events
	void Initialize(ID3D11Device1* device, ID3D11DeviceContext1 * deviceContext);
	void InitWindow(D3D11_VIEWPORT screenViewport);
	virtual void Update(float dt = 0);
	virtual void Reset();
	//Utility

	///Variables
	//@Font drawing
	std::unique_ptr<DirectX::SpriteBatch> m_spriteBatch;
	std::unique_ptr<DirectX::SpriteFont> m_font;
	//@Matrices - Coordinate spaces
	DirectX::SimpleMath::Matrix m_view;
	DirectX::SimpleMath::Matrix m_proj;
	//@Camera
	DirectX::SimpleMath::Vector3 m_cam;
	DirectX::SimpleMath::Vector3 m_look;
	float m_pitch, m_yaw;
};

#endif /*GRAPHICSYSTEM_H_*/