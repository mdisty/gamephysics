#include "DiffusionSpringSystemSimulator.h"

DiffusionSpringSystemSimulator::DiffusionSpringSystemSimulator() : springSystem_{ SpringSystem(10.0f, 80.0f, 1.0f, Vec3(0.0f), 0.0f) }
{
	m_iTestCase = 0;

	hotColor = { 252.0f / 255.0f, 3.0f / 255.0f, 80.0f / 255.0f };
	coldColor = { 0.0f / 255.0f, 234.0f / 255.0f, 255.0f / 255.0f };
	zeroColor = { 10.0f / 255.0f, 0.0f / 255.0f, 30.0f / 255.0f };
}

const char* DiffusionSpringSystemSimulator::getTestCasesStr() {
	return "Demo,Demo2,Demo3,Demo4";
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
	
	TwAddSeparator(DUC->g_pTweakBar, "", NULL);
	TwAddButton(DUC->g_pTweakBar, "Simulation Tools", NULL, NULL, "");
	TwAddVarRW(DUC->g_pTweakBar, "Start Simulation", TW_TYPE_BOOLCPP, &startSimulation_, "");
	TwEnumVal Tools[] = { {ADDNODE, "Add Node"}, {CHANGETEMP, "Change Temperature"} };
	TwType ToolsTwType = TwDefineEnum("ToolType", Tools, 2);
	TwAddVarRW(DUC->g_pTweakBar, "Tool", ToolsTwType, &m_tool, "");
	TwAddVarRW(DUC->g_pTweakBar, "Temperature", TW_TYPE_FLOAT, &temperature_, "");
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &alpha_, "");
	TwAddSeparator(DUC->g_pTweakBar, "", NULL);
	TwAddButton(DUC->g_pTweakBar, "Temperature Colors", NULL, NULL, "");
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
	case 0: {
		reset();

		try {

		springSystem_.insertSpringMassPoint(0, Vec3(0.0f, 1.0f, 0.0f), 1.0f, false);
		springSystem_.insertSpringMassPoint(0, Vec3(1.0f, 0.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(0, Vec3(-1.0f, 0.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(0, Vec3(0.0f, -1.0f, 0.0f), 0.0f, false);

		springSystem_.insertSpringMassPoint(1, Vec3(0.0f, 2.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(1, Vec3(1.0f, 1.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(1, Vec3(-1.0f, 1.0f, 0.0f), 0.0f, false);

		springSystem_.insertSpringMassPoint(5, Vec3(1.0f, 2.0f, 0.0f), 3.0f, false);
		springSystem_.insertSpringMassPoint(5, Vec3(-1.0f, 2.0f, 0.0f), -3.0f, false);
		
	    } catch (std::exception e) {
			std::cerr << e.what() << std::endl;
		}

		break; }
	case 1: {
		reset();

		break; }
	case 2: {
		reset();

		try {
			std::random_device rand_device;
			std::mt19937 generator(rand_device());
			std::uniform_real_distribution<double> distribution(-10.0, 10.0);

			int size = 2000;

			for (int idx = 0; idx < size; ++idx) {
				Vec3 mp = springSystem_.getMassPoint(idx).position;

				springSystem_.insertSpringMassPoint(idx, mp + Vec3(0.0f, 0.5f, 0.0f), distribution(generator), false);
				springSystem_.insertSpringMassPoint(idx, mp + Vec3(0.5f, 0.0f, 0.0f), distribution(generator), false);
				springSystem_.insertSpringMassPoint(idx, mp + Vec3(-0.5f, 0.0f, 0.0f), distribution(generator), false);
				springSystem_.insertSpringMassPoint(idx, mp + Vec3(0.0f, -0.5f, 0.0f), distribution(generator), false);
			}

		} catch (std::exception e) {
			std::cerr << e.what() << std::endl;
		}

		break; }
	case 3: {
		reset();

		// Up
		springSystem_.insertSpringMassPoint(0, Vec3(0.0f, 1.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(1, Vec3(0.0f, 2.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(2, Vec3(0.0f, 3.0f, 0.0f), -5.0f, false);

		// Left
		springSystem_.insertSpringMassPoint(3, Vec3(-1.0f, 3.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(4, Vec3(-1.0f, 2.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(5, Vec3(-1.0f, 1.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(6, Vec3(-1.0f, 0.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(7, Vec3(-1.0f, -1.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(8, Vec3(-1.0f, -2.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(9, Vec3(-1.0f, -3.0f, 0.0f), -5.0f, false);

		springSystem_.insertSpringMassPoint(4, Vec3(-2.0f, 3.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(11, Vec3(-2.0f, 2.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(12, Vec3(-2.0f, 1.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(13, Vec3(-2.0f, 0.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(14, Vec3(-2.0f, -1.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(15, Vec3(-2.0f, -2.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(16, Vec3(-2.0f, -3.0f, 0.0f), -5.0f, false);

		springSystem_.insertSpringMassPoint(11, Vec3(-3.0f, 3.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(18, Vec3(-3.0f, 2.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(19, Vec3(-3.0f, 1.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(20, Vec3(-3.0f, 0.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(21, Vec3(-3.0f, -1.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(22, Vec3(-3.0f, -2.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(23, Vec3(-3.0f, -3.0f, 0.0f), -5.0f, false);

		// Right
		springSystem_.insertSpringMassPoint(3, Vec3(1.0f, 3.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(25, Vec3(1.0f, 2.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(26, Vec3(1.0f, 1.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(27, Vec3(1.0f, 0.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(28, Vec3(1.0f, -1.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(29, Vec3(1.0f, -2.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(30, Vec3(1.0f, -3.0f, 0.0f), -5.0f, false);

		springSystem_.insertSpringMassPoint(25, Vec3(2.0f, 3.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(32, Vec3(2.0f, 2.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(33, Vec3(2.0f, 1.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(34, Vec3(2.0f, 0.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(35, Vec3(2.0f, -1.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(36, Vec3(2.0f, -2.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(37, Vec3(2.0f, -3.0f, 0.0f), -5.0f, false);

		springSystem_.insertSpringMassPoint(32, Vec3(3.0f, 3.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(39, Vec3(3.0f, 2.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(40, Vec3(3.0f, 1.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(41, Vec3(3.0f, 0.0f, 0.0f), 0.0f, false);
		springSystem_.insertSpringMassPoint(42, Vec3(3.0f, -1.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(43, Vec3(3.0f, -2.0f, 0.0f), -5.0f, false);
		springSystem_.insertSpringMassPoint(44, Vec3(3.0f, -3.0f, 0.0f), -5.0f, false);

		// Down
		springSystem_.insertSpringMassPoint(0, Vec3(0.0f, -1.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(46, Vec3(0.0f, -2.0f, 0.0f), 5.0f, false);
		springSystem_.insertSpringMassPoint(47, Vec3(0.0f, -3.0f, 0.0f), 0.0f, false);

		// Middle
		springSystem_.setTemperature(0, 5.0);

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

	springSystem_.getDiffusion().diffuseTemperatureImplicit(timeStep, alpha_);
	springSystem_.calculateMidpointStep(timeStep);
}

void DiffusionSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	Vec3 hot{ hotColor.at(0), hotColor.at(1), hotColor.at(2) };
	Vec3 zero{ zeroColor.at(0), zeroColor.at(1), zeroColor.at(2) };
	Vec3 cold{ coldColor.at(0), coldColor.at(1), coldColor.at(2) };

	springSystem_.drawSprings(DUC, hot, zero, cold);
}

void DiffusionSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;

	Ray r = getRay(m_trackmouse);

	switch (m_tool)
	{
	case ADDNODE:
	{
		if (selectedSphere_) {
			if (m_tool == ADDNODE) {
				springSystem_.insertSpringMassPoint(sphereIndex, r.intersectPlane(Vec3(0., 0., -1.)), temperature_, false);
				selectedSphere_ = false;
				springSystem_.setSelected(false, sphereIndex);
			}
			return;
		}

		sphereIndex = springSystem_.getSphereIndex(r);
		if (sphereIndex >= 0) {
			springSystem_.setSelected(true, sphereIndex);
			selectedSphere_ = true;
		}
		else cout << "No sphere was hit" << endl;

		return;
	}
	case CHANGETEMP:
	{
		sphereIndex = springSystem_.getSphereIndex(r);
		if (sphereIndex >= 0) {
			springSystem_.setTemperature(sphereIndex, temperature_);
		}
		else cout << "Couldnt find sphere to change color" << endl;
	}
	default:
		break;
	}
}

void DiffusionSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

Ray DiffusionSpringSystemSimulator::getRay(Point2D mouse)
{
	Vec3 origin = Vec3(DUC->g_camera.GetEyePt());

	long height = DXUTGetWindowHeight();
	long width = DXUTGetWindowWidth();

	// mouse -> ndc [-1,1]
	double mouseWidth = (2 * (double)mouse.x) / width - 1;

	double mouseHeight = (-1) * ((2 * (double)mouse.y) / height - 1);
	// z coordinate we ignore :D is 0.

	// create 3d vec because we need to use mat4 multiplication :D
	Vec3 MouseNDC = Vec3(mouseWidth, mouseHeight, 0.);

	// ndc -> view frustrum
	Mat4 projectionMatrix = Mat4(DUC->g_camera.GetProjMatrix());
	Vec3 frustrumMouse = projectionMatrix.inverse().transformVector(MouseNDC);

	// view frustrum -> world space
	Mat4 viewMatrix = Mat4(DUC->g_camera.GetViewMatrix());
	Vec3 worldMouse = viewMatrix.inverse().transformVector(frustrumMouse);

	Vec3 direction = getNormalized(worldMouse - origin);

	return Ray(direction, origin);
}
