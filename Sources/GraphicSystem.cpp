#include "pch.h"
#include "GraphicSystem.h"
#include "ObjectSystem.h"
#include "PhysicSystem.h"
#include "Sphere.h"
#include "OrientedBoundingBox.h"

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
GraphicSystem::~GraphicSystem() 
{

}
///Initialize components and shapes
void GraphicSystem::Initialize(ID3D11Device1* device, ID3D11DeviceContext1 * deviceContext) {
	ObjectSystem * os = ObjectSystem::GetInstance();
	vector<RigidbodyComponent*> rigidbodies = os->GetRigidbodyComponentList();
	
	m_spriteBatch = std::make_unique<SpriteBatch>(deviceContext);
	m_font = std::make_unique<SpriteFont>(device, L"Textures/myfile.spritefont");

	//@Init shapes
	for (unsigned int i = 0; i < rigidbodies.size(); i++) {
		//Downcast to create the right shape
		Sphere * theSphere = dynamic_cast<Sphere*>(rigidbodies[i]->m_shape);
		OrientedBoundingBox * theOBB = dynamic_cast<OrientedBoundingBox*>(rigidbodies[i]->m_shape);

		if (theSphere) {
			theSphere->m_primitive = GeometricPrimitive::CreateSphere(deviceContext, theSphere->m_radius * 2);
		}
		else if (theOBB) {
			theOBB->m_primitive = GeometricPrimitive::CreateBox(deviceContext, theOBB->m_halfExtents*2);
		}

		//@Get AABB size
		Vector3 AABBSize = rigidbodies[i]->m_shape->m_AABB.m_halfExtent*2;
		rigidbodies[i]->m_shape->m_AABBPrimitive = GeometricPrimitive::CreateBox(deviceContext, AABBSize);
	}
}
///Window size dependent resources
void GraphicSystem::InitWindow(D3D11_VIEWPORT screenViewport)
{
	//@Resize projection
	m_proj = Matrix::CreatePerspectiveFieldOfView(XM_PI / 4.f,
		screenViewport.Width/screenViewport.Height, 0.1f, 100.f);
}
///Draw all shapes and fonts
void GraphicSystem::Update(float dt) {
	ObjectSystem * os = ObjectSystem::GetInstance();
	PhysicSystem * ps = PhysicSystem::GetInstance();
	vector<RigidbodyComponent*> rigidbodies = os->GetRigidbodyComponentList();

	//@Euler to direction vector
	float y = sinf(m_pitch);
	float r = cosf(m_pitch);
	float z = r * cosf(m_yaw);
	float x = r * sinf(m_yaw);
	m_look = m_cam + Vector3(x, y, z);
	Matrix view = Matrix::CreateLookAt(m_cam, m_look, Vector3::Up);

	//@Render
	for (unsigned int i = 0; i < rigidbodies.size(); i++) {
		//Init each
		RigidbodyComponent * currentRb = rigidbodies[i];
		TransformComponent currentTransform = currentRb->m_owner->m_transform;
		Matrix translation = Matrix::CreateTranslation(currentTransform.m_position);
		Matrix rotation = Matrix::CreateFromQuaternion(currentTransform.m_rotation);
		Matrix scale = Matrix::CreateScale(currentTransform.m_scale);
		Matrix world = scale*rotation*translation;

		//@Scale set to one, for now.
		currentRb->m_shape->m_primitive->Draw( world, view, m_proj, currentRb->m_shape->m_color );

		//@Debug draw (Wireframes) (Only take into account position and shape properties, nor rotation nor scale)
		if (ps->m_AABBCulling.isEnabled) {
			//If its enabled, the PhysicsSystem, should of have computed, and updated, the AABB
			currentRb->m_shape->m_AABBPrimitive->Draw( translation, view, m_proj, currentRb->m_shape->m_AABBColor, nullptr, true);
		}
		if (ps->m_sphereCulling.isEnabled) {
		
		}
	}

	//@Render fonts
	m_spriteBatch->Begin();
	const wchar_t * fps = ps->m_fps.c_str();
	const wchar_t * AABBCulling = ps->m_AABBCulling.log.c_str();
	m_font->DrawString(m_spriteBatch.get(), fps, Vector2(10, 10 + 0*15 ), Colors::Black);
	m_font->DrawString(m_spriteBatch.get(), AABBCulling, Vector2(10, 10 + 1*15), Colors::Black);
	m_spriteBatch->End();

}
///Reset shapes and components
void GraphicSystem::Reset()
{
	ObjectSystem * os = ObjectSystem::GetInstance();
	vector<RigidbodyComponent*> rigidbodies = os->GetRigidbodyComponentList();

	//@Reset shapes
	for (unsigned int i = 0; i < rigidbodies.size(); i++) {
		rigidbodies[i]->m_shape->m_primitive.reset();
		rigidbodies[i]->m_shape->m_AABBPrimitive.reset();
		rigidbodies[i]->m_shape->m_spherePrimitive.reset();
	}

	//@Reset spriteBatch
	m_spriteBatch.reset();
}
//Utility
