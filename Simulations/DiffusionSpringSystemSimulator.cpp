#include "DiffusionSpringSystemSimulator.h"

DiffusionSpringSystemSimulator::DiffusionSpringSystemSimulator() : springSystem_{ SpringSystem(10.0f, 80.0f, 1.0f, Vec3(0.0f), 0.0f) }
{
	m_iTestCase = 0;

	hotColor = { 252.0f / 255.0f, 3.0f / 255.0f, 80.0f / 255.0f };
	coldColor = { 0.0f / 255.0f, 234.0f / 255.0f, 255.0f / 255.0f };
	zeroColor = { 10.0f / 255.0f, 0.0f / 255.0f, 30.0f / 255.0f };
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
	TwAddVarRW(DUC->g_pTweakBar, "Start Simulation", TW_TYPE_BOOLCPP, &startSimulation_, "");
	TwAddVarRW(DUC->g_pTweakBar, "Hot Color", TW_TYPE_COLOR3F, &hotColor, "colormode=rgb");
	TwAddVarRW(DUC->g_pTweakBar, "Cold Color", TW_TYPE_COLOR3F, &coldColor, "colormode=rgb");
	TwAddVarRW(DUC->g_pTweakBar, "Zero Color", TW_TYPE_COLOR3F, &zeroColor, "colormode=rgb");

	switch (m_iTestCase)
	{
	case 0: { 
		
		break; 
	}
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

		try {

		int point1 = springSystem_.insertSpringMassPoint(0, Vec3(0.0f, 1.0f, 0.0f), 1.0f, false);
		springSystem_.insertSpringMassPoint(0, Vec3(1.0f, 0.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(0, Vec3(-1.0f, 0.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(0, Vec3(0.0f, -1.0f, 0.0f), 0.0f, false);

		int point2 = springSystem_.insertSpringMassPoint(point1, Vec3(0.0f, 2.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(point1, Vec3(1.0f, 1.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(point1, Vec3(-1.0f, 1.0f, 0.0f), 0.0f, false);

		springSystem_.insertSpringMassPoint(point2, Vec3(1.0f, 2.0f, 0.0f), 3.0f, false);
		springSystem_.insertSpringMassPoint(point2, Vec3(-1.0f, 2.0f, 0.0f), -3.0f, false);

	    } catch (std::exception e) {
			std::cerr << e.what() << std::endl;
		}

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
	if (!startSimulation_) return;

	switch (m_iTestCase)
	{
	case 0: {
		springSystem_.getDiffusion().diffuseTemperatureImplicit(timeStep, 0.05f);
		springSystem_.calculateMidpointStep(timeStep);
		break; 
	}
	default:
		break;
	}
}

void DiffusionSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	Vec3 hot{ hotColor.at(0), hotColor.at(1), hotColor.at(2) };
	Vec3 zero{ zeroColor.at(0), zeroColor.at(1), zeroColor.at(2) };
	Vec3 cold{ coldColor.at(0), coldColor.at(1), coldColor.at(2) };

	switch (m_iTestCase)
	{
	case 0: {
		springSystem_.drawSprings(DUC, hot, zero, cold);
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
