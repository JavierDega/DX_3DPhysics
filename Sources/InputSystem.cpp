#include "pch.h"
#include "..\Headers\InputSystem.h"
#include "GraphicSystem.h"
extern void ExitGame();

using namespace DirectX;
using namespace SimpleMath;
using namespace std;

//Instance
InputSystem * InputSystem::m_instance = NULL;
InputSystem * InputSystem::GetInstance()
{
	//Singleton
	if (m_instance == NULL)
	{
		m_instance = new InputSystem();
	}
	return m_instance;
}
//Constructor
InputSystem::InputSystem()
{
	//@Get pointers to GraphicSystem's m_cam and m_look.
	m_cam = nullptr;
	m_look = nullptr;
	m_pitch = 0;
	m_yaw = 0;
}
//Destructor
InputSystem::~InputSystem()
{
	m_cam = nullptr;
	delete m_cam;
	m_look = nullptr;
	delete m_look;
	m_pitch = nullptr;
	delete m_pitch;
	m_yaw = nullptr;
	delete m_yaw;
}
void InputSystem::Initialize(ID3D11Device1 * device, ID3D11DeviceContext1 * deviceContext)
{
	GraphicSystem * gs = GraphicSystem::GetInstance();
	m_cam = &gs->m_cam;
	m_look = &gs->m_look;
	m_pitch = &gs->m_pitch;
	m_yaw = &gs->m_yaw;
	m_keyboard = std::make_unique<Keyboard>();
	m_mouse = std::make_unique<Mouse>();
	
}
void InputSystem::InitWindow(D3D11_VIEWPORT screenViewport)
{
}
void InputSystem::Update(float dt)
{
	//@Mouse
	auto mouse = m_mouse->GetState();
	if (mouse.positionMode == Mouse::MODE_RELATIVE)
	{
		Vector3 delta = Vector3(float(mouse.x), float(mouse.y), 0.f) * dt;
		*m_pitch -= delta.y;
		*m_yaw -= delta.x;
		// limit pitch to straight up or straight down
		// with a little fudge-factor to avoid gimbal lock
		float limit = XM_PI / 2.0f - 0.01f;
		*m_pitch = max(-limit, *m_pitch);
		*m_pitch = min(+limit, *m_pitch);
		// keep longitude in sane range by wrapping
		if (*m_yaw > XM_PI)
		{
			*m_yaw -= XM_PI * 2.0f;
		}
		else if (*m_yaw < -XM_PI)
		{
			*m_yaw += XM_PI * 2.0f;
		}
	}
	m_mouse->SetMode(mouse.leftButton ? Mouse::MODE_ABSOLUTE : Mouse::MODE_RELATIVE);

	//@Get directional vector from m_cam to m_look
	auto kb = m_keyboard->GetState();
	if (kb.Escape)
	{
		ExitGame();
	}
	//@Already normalized
	Vector3 relativeForward = *m_look - *m_cam;
	Vector3 relativeRight = relativeForward.Cross(Vector3::Up);
	relativeRight.Normalize();
	Vector3 relativeUp = relativeRight.Cross(relativeForward);
	//@Does this need to be done?
	relativeUp.Normalize();

	//Forward
	if (kb.W) {
		*m_cam += relativeForward * 0.1f;
	}
	if (kb.S) {
		*m_cam -= relativeForward * 0.1f;
	}
	//Right
	if (kb.D) {
		*m_cam += relativeRight * 0.1f;
	}
	if (kb.A) {
		*m_cam -= relativeRight * 0.1f;
	}
	//Up
	if (kb.Q) {
		*m_cam += relativeUp * 0.1f;
	}
	if (kb.E) {
		*m_cam -= relativeUp * 0.1f;
	}

}
void InputSystem::Reset()
{
}
void InputSystem::SetMouseWindow(HWND window)
{
	m_mouse->SetWindow(window);
}
void InputSystem::ScanInput()
{
}
