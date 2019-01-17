#include "pch.h"
#include "..\Headers\InputSystem.h"
#include "GraphicSystem.h"
extern void ExitGame();

using namespace DirectX;
using namespace SimpleMath;

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
}
void InputSystem::Initialize(ID3D11Device1 * device, ID3D11DeviceContext1 * deviceContext)
{
	GraphicSystem * gs = GraphicSystem::GetInstance();
	m_cam = &gs->m_cam;
	m_look = &gs->m_look;
	m_keyboard = std::make_unique<Keyboard>();
	m_mouse = std::make_unique<Mouse>();
	
}
void InputSystem::InitWindow(D3D11_VIEWPORT screenViewport)
{
}
void InputSystem::Update(float dt)
{
	//@Get directional vector from m_cam to m_look
	Vector3 moveAxis = (*m_look - *m_cam);
	moveAxis.Normalize();
	Vector3 move = Vector3::Zero;

	auto kb = m_keyboard->GetState();
	if (kb.Escape)
	{
		ExitGame();
	}
	if (kb.W) {
		
		move += moveAxis.Backward;
	}
	if (kb.S) {
	
		move += moveAxis.Forward;
	}
	if (kb.A) {
		move += moveAxis.Right;
	}
	if (kb.D) {
		move += moveAxis.Left;
	}
	if (kb.Q) {
		move += moveAxis.Up;
	}
	if (kb.E) {
		move += moveAxis.Down;
	}

	move *= 0.1f;
	*m_cam += move;

	/*Vector3 halfBound = (Vector3(ROOM_BOUNDS.v) / Vector3(2.f))
		- Vector3(0.1f, 0.1f, 0.1f);

	m_cameraPos = Vector3::Min(m_cameraPos, halfBound);
	m_cameraPos = Vector3::Max(m_cameraPos, -halfBound);*/

	auto mouse = m_mouse->GetState();
	if (mouse.positionMode == Mouse::MODE_RELATIVE)
	{
		Vector3 delta = Vector3(float(mouse.x), float(mouse.y), 0.f) * 0.01f;

		m_pitch -= delta.y;
		m_yaw -= delta.x;
		// limit pitch to straight up or straight down
		// with a little fudge-factor to avoid gimbal lock
		float limit = XM_PI / 2.0f - 0.01f;
		m_pitch = std::max(-limit, m_pitch);
		m_pitch = std::min(+limit, m_pitch);

		// keep longitude in sane range by wrapping
		if (m_yaw > XM_PI)
		{
			m_yaw -= XM_PI * 2.0f;
		}
		else if (m_yaw < -XM_PI)
		{
			m_yaw += XM_PI * 2.0f;
		}
	}

	Quaternion q = Quaternion::CreateFromYawPitchRoll(m_yaw, m_pitch, 0.f);
	*m_look =  *m_cam + Vector3::Transform(Vector3(0, 1.0f, -1.0f), q);
	m_mouse->SetMode(mouse.leftButton ? Mouse::MODE_ABSOLUTE : Mouse::MODE_RELATIVE);

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
