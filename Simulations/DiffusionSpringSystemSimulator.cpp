#include "DiffusionSpringSystemSimulator.h"

DiffusionSpringSystemSimulator::DiffusionSpringSystemSimulator() : springSystem_{ SpringSystem(10.0f, 80.0f, 1.0f, Vec3(0.0f), 0.0f) }
{
	m_iTestCase = 0;
}

const char* DiffusionSpringSystemSimulator::getTestCasesStr() {
	return "Demo";
}

void DiffusionSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	springSystem_ = SpringSystem(10.0f, 80.0f, 1.0f, Vec3(0.0f), 0.0f);
}

void DiffusionSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: { 
		
		break; }
	default:break;
	}
}

void DiffusionSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:{
		reset();

		int point1 = springSystem_.insertSpringMassPoint(0, Vec3(0.0f, 1.0f, 0.0f), 1.0f, 0.0f);
		springSystem_.insertSpringMassPoint(0, Vec3(1.0f, 0.0f, 0.0f), 1.0f, 0.0f);
		springSystem_.insertSpringMassPoint(0, Vec3(-1.0f, 0.0f, 0.0f), 1.0f, 0.0f);
		springSystem_.insertSpringMassPoint(0, Vec3(0.0f, -1.0f, 0.0f), 1.0f, 0.0f);

		int point2 = springSystem_.insertSpringMassPoint(point1, Vec3(0.0f, 2.0f, 0.0f), 1.0f, 0.0f);
		springSystem_.insertSpringMassPoint(point1, Vec3(1.0f, 1.0f, 0.0f), 1.0f, 0.0f);
		springSystem_.insertSpringMassPoint(point1, Vec3(-1.0f, 1.0f, 0.0f), 1.0f, 0.0f);

		springSystem_.insertSpringMassPoint(point2, Vec3(1.0f, 2.0f, 0.0f), 1.0f, 0.0f);

		break; }
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	
}

void DiffusionSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0: {
		springSystem_.calculateMidpointStep(timeStep, 0.05f);
		break; 
	}
	default:
		break;
	}
}

void DiffusionSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: {
		springSystem_.drawSprings(DUC);
		break; 
	}
	default: break;
	}
}

void DiffusionSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
