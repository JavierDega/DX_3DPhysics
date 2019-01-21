#include "pch.h"
#include "GraphicSystem.h"
#include "ObjectSystem.h"
#include "PhysicSystem.h"
#include "Sphere.h"
#include <stdio.h>

using namespace std;
using namespace DirectX;
using namespace SimpleMath;

//Instance
GraphicSystem * GraphicSystem::m_instance = NULL;
GraphicSystem * GraphicSystem::GetInstance()
{
	//Singleton
	if (m_instance == NULL)
	{
		m_instance = new GraphicSystem();
	}
	return m_instance;
}

//Constructor
GraphicSystem::GraphicSystem() 
	: m_pitch(0), m_yaw(0), m_cam(Vector3::Zero), m_look(Vector3::Forward)
{
}
//Destructor (Singleton so..?)
GraphicSystem::~GraphicSystem() {

}
//Init
void GraphicSystem::Initialize(ID3D11Device1* device, ID3D11DeviceContext1 * deviceContext) {
	//Initialize rigidbody components
	ObjectSystem * os = ObjectSystem::GetInstance();

	vector<RigidbodyComponent*> rigidbodies = os->GetRigidbodyComponentList();

	for (unsigned int i = 0; i < rigidbodies.size(); i++) {
		//Init each
		//Dynamic cast
		Sphere * theSphere = dynamic_cast<Sphere*>(rigidbodies[i]->m_shape);
		if (theSphere) {
			theSphere->m_primitive = GeometricPrimitive::CreateSphere(deviceContext,theSphere->m_radius * 2);
		}
	}
}
void GraphicSystem::InitWindow(D3D11_VIEWPORT screenViewport)
{
	//@Projection recreated everytime we resize window
	m_proj = Matrix::CreatePerspectiveFieldOfView(XM_PI / 4.f,
		screenViewport.Width/screenViewport.Height, 0.1f, 100.f);
}
//Update
void GraphicSystem::Update(float dt) {
	///Draw all shapes in rigidbodyComponents
	ObjectSystem * os = ObjectSystem::GetInstance();
	PhysicSystem * ps = PhysicSystem::GetInstance();

	float y = sinf(m_pitch);
	float r = cosf(m_pitch);
	float z = r * cosf(m_yaw);
	float x = r * sinf(m_yaw);

	m_look = m_cam + Vector3(x, y, z);
	//Cam look at
	m_view = Matrix::CreateLookAt(m_cam, m_look, Vector3::Up);
	//Render
	vector<RigidbodyComponent*> rigidbodies = os->GetRigidbodyComponentList();

	for (unsigned int i = 0; i < rigidbodies.size(); i++) {
		//Init each
		RigidbodyComponent * currentRb = rigidbodies[i];
		TransformComponent currentTransform = currentRb->m_owner->m_transform;

		Matrix m_world = Matrix::CreateTranslation(currentTransform.m_position);
		currentRb->m_shape->m_primitive->Draw( m_world, m_view, m_proj );
	}

}

void GraphicSystem::Reset()
{
	//Reset all shapes in RigidbodyComponents
	ObjectSystem * os = ObjectSystem::GetInstance();

	vector<RigidbodyComponent*> rigidbodies = os->GetRigidbodyComponentList();

	for (unsigned int i = 0; i < rigidbodies.size(); i++) {
		//Init each
		rigidbodies[i]->m_shape->m_primitive.reset();
	}
}
