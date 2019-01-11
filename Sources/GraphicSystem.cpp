#include "pch.h"
#include "GraphicSystem.h"
#include "ObjectSystem.h"
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
GraphicSystem::GraphicSystem() {
	m_cam = { 0.0F, 0.0F, 0.0F };
	m_up = { 0.0F, 1.0F, 0.0F };
	m_look = { 0.0F, 0.0F, -1.0F };
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
		rigidbodies[i]->m_shape = GeometricPrimitive::CreateSphere(deviceContext, rigidbodies[i]->m_radius * 2 );
	}
}
void GraphicSystem::InitWindow(D3D11_VIEWPORT screenViewport)
{
	m_view = Matrix::CreateLookAt(m_cam, m_look, m_up);
	m_proj = Matrix::CreatePerspectiveFieldOfView(XM_PI / 4.f,
		screenViewport.Width/screenViewport.Height, 0.1f, 100.f);
}
//Update
void GraphicSystem::Update(float dt) {
	///Draw all shapes in rigidbodyComponents
	ObjectSystem * os = ObjectSystem::GetInstance();

	//Cam look at
	m_view = Matrix::CreateLookAt(m_cam, m_look, m_up);
	//Render
	vector<RigidbodyComponent*> rigidbodies = os->GetRigidbodyComponentList();

	for (unsigned int i = 0; i < rigidbodies.size(); i++) {
		//Init each
		RigidbodyComponent * currentRb = rigidbodies[i];
		TransformComponent currentTransform = currentRb->m_owner->m_transform;

		Matrix m_world = Matrix::CreateTranslation(currentTransform.m_position);
		currentRb->m_shape->Draw( m_world, m_view, m_proj);
	}
}

void GraphicSystem::Reset()
{
	//Reset all shapes in RigidbodyComponents
	ObjectSystem * os = ObjectSystem::GetInstance();

	vector<RigidbodyComponent*> rigidbodies = os->GetRigidbodyComponentList();

	for (unsigned int i = 0; i < rigidbodies.size(); i++) {
		//Init each
		rigidbodies[i]->m_shape.reset();
	}
}
