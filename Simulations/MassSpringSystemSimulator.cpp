#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	// Data Attributes
	m_fMass = 10;
	m_fStiffness = 40;
	m_fDamping = 0.0f;
	m_iIntegrator = 0.1f;
	m_massPoints = std::vector<MassPoint>();
	m_springs = std::vector<Spring>();
	m_useMidpoint = false;

	// UI Attributes
	m_externalForce = Vec3(0.0f, 0.0f, 0.0f);
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: break;
	case 1: break;
	case 2: break;
	case 3: 
		TwAddVarRW(DUC->g_pTweakBar, "Use Midpoint Integration", TW_TYPE_BOOL8, &m_useMidpoint, "min=false");
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01f");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0.01f");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0.01f");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.01f");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_DIR3F, &m_externalForce, "");
		break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: break;
	case 1: // Euler
		for (int i = 0; i < m_massPoints.size(); ++i)
		{
			DUC->drawSphere(m_massPoints[i].position, 0.1f);
		}

		DUC->beginLine();
		for (int i = 0; i < m_springs.size(); ++i)
		{
			DUC->drawLine(getPositionOfMassPoint(m_springs[i].MassPoint1), Vec3(1), getPositionOfMassPoint(m_springs[i].MassPoint2), Vec3(1));
		}
		DUC->endLine();
		break;
	case 2: // Midpoint
		DUC->setUpLighting(Vec3(1.0f), Vec3(1.0f), 100, Vec3(1.0f));
		for (int i = 0; i < m_massPoints.size(); ++i)
		{
			DUC->drawSphere(m_massPoints[i].position, 0.1f);
		}

		DUC->beginLine();
		for (int i = 0; i < m_springs.size(); ++i)
		{
			DUC->drawLine(getPositionOfMassPoint(m_springs[i].MassPoint1), Vec3(1), getPositionOfMassPoint(m_springs[i].MassPoint2), Vec3(1));
		}
		DUC->endLine();
		break;
	case 3: // Complex
		DUC->setUpLighting(Vec3(1.0f), Vec3(1.0f), 100, Vec3(1.0f));
		for (int i = 0; i < m_massPoints.size(); ++i)
		{
			DUC->drawSphere(m_massPoints[i].position, 0.01f);
		}

		DUC->beginLine();
		for (int i = 0; i < m_springs.size(); ++i)
		{
			if (i < 3) // Wäscheleine (Gelb)
			{
				DUC->drawLine(getPositionOfMassPoint(m_springs[i].MassPoint1), Vec3(1, 1, 0), getPositionOfMassPoint(m_springs[i].MassPoint2), Vec3(1, 1, 0));
			}
			else if (i < 20) // Tuch (Lila)
			{
				DUC->drawLine(getPositionOfMassPoint(m_springs[i].MassPoint1), Vec3(0.5f, 0, 0.5f), getPositionOfMassPoint(m_springs[i].MassPoint2), Vec3(0.5f, 0.0f, 0.5f));
			}
			else if (i < 34) // Gestell (Weiß)
			{
				DUC->drawLine(getPositionOfMassPoint(m_springs[i].MassPoint1), Vec3(1), getPositionOfMassPoint(m_springs[i].MassPoint2), Vec3(1));
			}
			else // Stützen (Black)
			{
				DUC->drawLine(getPositionOfMassPoint(m_springs[i].MassPoint1), Vec3(0.05f), getPositionOfMassPoint(m_springs[i].MassPoint2), Vec3(0.05f));
			}
		}
		DUC->endLine();
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1!\n";
		m_massPoints.clear();
		m_springs.clear();
		setMass(40);
		setStiffness(40);
		setDampingFactor(0);
		applyExternalForce(Vec3(0.0f));

		// Do Euler
		// Insert Spring
		addSpring(0, 1, 1.0f);
		// Insert Points
	 	addMassPoint(Vec3(0.0f, 0.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f), false);
		cout << "P1 Position: " << m_massPoints[0].position << " Velocity: " << m_massPoints[0].velocity;
		addMassPoint(Vec3(0.0f, 2.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), false);
		cout << " P2 Position: " << m_massPoints[1].position << " Velocity: " << m_massPoints[1].velocity << "\n";

		eulerIntegration(0.1f);
		cout << "After Euler:\n";
		cout << "P1 Position: " << m_massPoints[0].position << " Velocity: " << m_massPoints[0].velocity;
		cout << " P2 Position: " << m_massPoints[1].position << " Velocity: " << m_massPoints[1].velocity << "\n";

		// Do Midpoint
		m_massPoints.clear();
		addMassPoint(Vec3(0.0f, 0.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f), false);
		addMassPoint(Vec3(0.0f, 2.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), false);
		midpointIntegration(0.1f);
		cout << "After Midpoint:\n";
		cout << "P1 Position: " << m_massPoints[0].position << " Velocity: " << m_massPoints[0].velocity;
		cout << " P2 Position: " << m_massPoints[1].position << " Velocity: " << m_massPoints[1].velocity << "\n";
		
		break;
	case 1: // Euler
		cout << "Demo 2!\n";
		m_massPoints.clear();
		m_springs.clear();

		addMassPoint(Vec3(0.0f, 0.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f), false);
		addMassPoint(Vec3(0.0f, 2.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), false);
		addSpring(0, 1, 1);

		setMass(10);
		setStiffness(40);
		setDampingFactor(1);
		applyExternalForce(Vec3(0.0f));
		// TODO: Change Timestep
		break;
	case 2: // Midpoint
		cout << "Demo 3!\n";
		m_massPoints.clear();
		m_springs.clear();

		addMassPoint(Vec3(0.0f, 0.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f), false);
		addMassPoint(Vec3(0.0f, 2.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), false);
		addSpring(0, 1, 1);

		setMass(10);
		setStiffness(40);
		setDampingFactor(1);
		applyExternalForce(Vec3(0.0f));
		// TODO: Change Timestep
		break;
	case 3: // Complex
		cout << "Demo 4!\n";
		m_massPoints.clear();
		m_springs.clear();

		/*
		// Add Masspoints
		addMassPoint(Vec3(-0.5f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), true); // 0
		addMassPoint(Vec3(-0.25f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), true); // 1
		addMassPoint(Vec3(0.25f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), true); // 2
		addMassPoint(Vec3(0.5f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), true); // 3

		addMassPoint(Vec3(-0.5f, 0.17f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), false); // 4
		addMassPoint(Vec3(-0.25f, 0.17f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), false); // 5
		addMassPoint(Vec3(0.25f, 0.17f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), false); // 6
		addMassPoint(Vec3(0.5f, 0.17f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), false); // 7

		addMassPoint(Vec3(-0.5f, 0.33f, 0.0f), Vec3(0.0f, 1.0f, 1.0f), false); // 8
		addMassPoint(Vec3(-0.25f, 0.33f, 0.0f), Vec3(0.0f, 1.0f, 1.0f), false); // 9
		addMassPoint(Vec3(0.25f, 0.33f, 0.0f), Vec3(0.0f, 1.0f, 1.0f), false); // 10
		addMassPoint(Vec3(0.5f, 0.33f, 0.0f), Vec3(0.0f, 1.0f, 1.0f), false); // 11

		addMassPoint(Vec3(-0.5f, 0.5f, 0.0f), Vec3(0.0f, 0.0f, 1.0f), false); // 12
		addMassPoint(Vec3(-0.25f, 0.5f, 0.0f), Vec3(0.0f, 0.0f, 1.0f), false); // 13
		addMassPoint(Vec3(0.25f, 0.5f, 0.0f), Vec3(0.0f, 0.0f, 1.0f), false); // 14
		addMassPoint(Vec3(0.5f, 0.5f, 0.0f), Vec3(0.0f, 0.0f, 1.0f), false); // 15

		// Add Springs
		// Vertical
		addSpring(0, 4, 0.17f);
		addSpring(4, 8, 0.17f);
		addSpring(8, 12, 0.17f);

		addSpring(1, 5, 0.17f);
		addSpring(5, 9, 0.17f);
		addSpring(9, 13, 0.17f);

		addSpring(2, 6, 0.17f);
		addSpring(6, 10, 0.17f);
		addSpring(10, 14, 0.17f);

		addSpring(3, 7, 0.17f);
		addSpring(7, 11, 0.17f);
		addSpring(11, 15, 0.17f);
		// Horizontal
		addSpring(4, 5, 0.25f);
		addSpring(5, 6, 0.45f);
		addSpring(6, 7, 0.25f);

		addSpring(8, 9, 0.25f);
		addSpring(9, 10, 0.45f);
		addSpring(10, 11, 0.25f);

		addSpring(12, 13, 0.25f);
		addSpring(13, 14, 0.45f);
		addSpring(14, 15, 0.25f);
		*/

		// Black
		addMassPoint(Vec3(10.0f, 10.0f, 10.0f), Vec3(0.0f), true); // 0
		addMassPoint(Vec3(-1.0f, 1.0f, 0.0f), Vec3(0.0f), false); // 1
		addMassPoint(Vec3(-0.5f, 1.0f, 0.0f), Vec3(0.0f), false); // 2
		addMassPoint(Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f), false); // 3
		addMassPoint(Vec3(0.5f, 1.0f, 0.0f), Vec3(0.0f), false); // 4
		addMassPoint(Vec3(1.0f, 1.0f, 0.0f), Vec3(0.0f), false); // 5
		addMassPoint(Vec3(-0.5f, 1.5f, 0.0f), Vec3(0.0f), false); // 6
		addMassPoint(Vec3(0.0f, 1.5f, 0.0f), Vec3(0.0f), false); // 7
		addMassPoint(Vec3(0.5f, 1.5f, 0.0f), Vec3(0.0f), false); // 8
		addMassPoint(Vec3(-0.5f, 2.0f, 0.0f), Vec3(0.0f), false); // 9
		addMassPoint(Vec3(0.0f, 2.0f, 0.0f), Vec3(0.0f), false); // 10
		addMassPoint(Vec3(0.5f, 2.0f, 0.0f), Vec3(0.0f), false); // 11
		addMassPoint(Vec3(-0.5f, 2.5f, 0.0f), Vec3(0.0f, 0.0f, 0.25f), false); // 12
		addMassPoint(Vec3(0.0f, 2.5f, 0.0f), Vec3(0.0f, 0.0f, 0.25f), false); // 13
		addMassPoint(Vec3(0.5f, 2.5f, 0.0f), Vec3(0.0f, 0.0f, 0.25f), false); // 14
		addMassPoint(Vec3(-1.0f, -0.5f, 0.0f), Vec3(0.0f), false); // 15
		addMassPoint(Vec3(-1.5f, -1.0f, -0.5f), Vec3(0.0f), false); // 16
		addMassPoint(Vec3(-0.5f, -1.0f, -0.5f), Vec3(0.0f), false); // 17
		addMassPoint(Vec3(-1.0f, -1.0f, 0.5f), Vec3(0.0f), false); // 18
		addMassPoint(Vec3(1.0f, -1.0f, -0.5f), Vec3(0.0f), false); // 19
		addMassPoint(Vec3(1.5f, -1.0f, 0.5f), Vec3(0.0f), false); // 20
		addMassPoint(Vec3(0.5f, -1.0f, 0.5f), Vec3(0.0f), false); // 21
		addMassPoint(Vec3(1.0f, -0.5f, 0.0f), Vec3(0.0f), false); // 22

		// Wäscheleine 0-2
		addSpring(1, 2, 0.5f);
		addSpring(5, 4, 0.5f);
		addSpring(5, 1, 2.0f);

		// Tuch 3-19
		addSpring(3, 2, 0.5f);
		addSpring(3, 4, 0.5f);
		addSpring(6, 7, 0.5f);
		addSpring(8, 7, 0.5f);
		addSpring(9, 10, 0.5f);
		addSpring(11, 10, 0.5f);
		addSpring(12, 13, 0.5f);
		addSpring(14, 13, 0.5f);
		addSpring(2, 6, 0.5f);
		addSpring(9, 6, 0.5f);
		addSpring(9, 12, 0.5f);
		addSpring(3, 7, 0.5f);
		addSpring(10, 7, 0.5f);
		addSpring(10, 13, 0.5f);
		addSpring(4, 8, 0.5f);
		addSpring(11, 8, 0.5f);
		addSpring(11, 14, 0.5f);

		// Gestell 20-33
		addSpring(1, 15, 1.5f);
		addSpring(16, 15, 0.8660f);
		addSpring(17, 15, 0.8660f);
		addSpring(18, 15, 0.7071f);
		addSpring(5, 22, 1.5f);
		addSpring(21, 22, 0.8660f);
		addSpring(20, 22, 0.8660f);
		addSpring(19, 22, 0.7071f);
		addSpring(16, 17, 1.0f);
		addSpring(16, 18, 1.118f);
		addSpring(18, 17, 1.118f);
		addSpring(21, 20, 1.0f);
		addSpring(19, 20, 1.118f);
		addSpring(19, 21, 1.118f);
		
		// Assistenz (Hide) 34-...
		addSpring(16, 1, 2.1213f);
		addSpring(17, 1, 2.1212f);
		addSpring(18, 1, 2.0616f);
		addSpring(20, 5, 2.1213f);
		addSpring(21, 5, 2.1212f);
		addSpring(19, 5, 2.0616f);
		addSpring(15, 5, 2.5f);
		addSpring(22, 1, 2.5f);


		setMass(10);
		setStiffness(40);
		setDampingFactor(0.5f);
		applyExternalForce(Vec3(0.0f, -0.5f, 0.0f));
		break;
	default:
		cout << "Empty Demo!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) // TODO: determine if this should even be
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	/*
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		for (int i = 0; i < m_massPoints.size(); i++) 
		{
			m_massPoints[i].position = (m_massPoints[i].finalPosition + inputWorld) * !m_massPoints[i].isFixed;
		}
	}
	else {
		for (int i = 0; i < m_massPoints.size(); i++)
		{
			m_massPoints[i].finalPosition = m_massPoints[i].position;
		}
	}
	*/
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		break;
	case 1: // Euler
		eulerIntegration(0.005f);
		break;
	case 2:
		midpointIntegration(0.005f);
		break;
	case 3: // Complex
		if (m_useMidpoint)
		{
			midpointIntegration(timeStep);
		}
		else
		{
			eulerIntegration(timeStep);
		}
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	for (int i = 0; i < m_massPoints.size(); ++i)
	{
		m_massPoints[i].mass = mass;
	}
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	for (int i = 0; i < m_springs.size(); ++i)
	{
		m_springs[i].stiffness = stiffness;
	}
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	for (int i = 0; i < m_massPoints.size(); ++i)
	{
		m_fDamping = damping;
	}
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	m_massPoints.push_back(MassPoint(position, Velocity, Vec3(0.0f), m_fMass, isFixed));
	
	return getNumberOfMassPoints();
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_springs.push_back(Spring(masspoint1, masspoint2, initialLength, m_fStiffness));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_massPoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_massPoints[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}

void MassSpringSystemSimulator::eulerIntegration(float h)
{
	clearForces();
	forcesCalculations();
	for (int i = 0; i < m_massPoints.size(); ++i)
	{
		if (m_massPoints[i].isFixed) continue;

		m_massPoints[i].position = m_massPoints[i].position + h * m_massPoints[i].velocity;
		if (m_massPoints[i].position.y < -1.0) m_massPoints[i].position.y = -1.0;
		m_massPoints[i].velocity = m_massPoints[i].velocity + h * m_massPoints[i].force;
	}
}

void MassSpringSystemSimulator::midpointIntegration(float h)
{
	std::vector<Vec3> storePos(m_massPoints.size());
	std::vector<Vec3> storeVel(m_massPoints.size());

	clearForces();
	forcesCalculations();
	
	for (int i = 0; i < m_massPoints.size(); ++i)
	{
		if (m_massPoints[i].isFixed) continue;

		storePos[i] = getPositionOfMassPoint(i); // Store old Pos and Vel for second Phase
		storeVel[i] = getVelocityOfMassPoint(i);
		m_massPoints[i].position = m_massPoints[i].position + h * 0.5f * m_massPoints[i].velocity; // Calculate Pos and Vel at Midpoint
		if (m_massPoints[i].position.y < -1.0) m_massPoints[i].position.y = -1.0;
		m_massPoints[i].velocity = m_massPoints[i].velocity + h * 0.5f * m_massPoints[i].force;
	}

	forcesCalculations(); // Calculate new a(x(t))

	for (int i = 0; i < m_massPoints.size(); ++i)
	{
		if (m_massPoints[i].isFixed) continue;

		m_massPoints[i].position = storePos[i] + h * m_massPoints[i].velocity; // Calculate final Pos and Vel
		if (m_massPoints[i].position.y < -1.0) m_massPoints[i].position.y = -1.0;
		m_massPoints[i].velocity = storeVel[i] + h * m_massPoints[i].force;
	}
}

void MassSpringSystemSimulator::forcesCalculations()
{
	// Internal
	for (int i = 0; i < m_springs.size(); ++i)
	{

		Vec3 pos1 = getPositionOfMassPoint(m_springs[i].MassPoint1);
		Vec3 pos2 = getPositionOfMassPoint(m_springs[i].MassPoint2);
		Vec3 dirVec = pos1 - pos2;
		float currentLength = normalize(dirVec);
		Vec3 internalForce = m_fStiffness * (currentLength - m_springs[i].initialLenght) * dirVec;
		if (!m_massPoints[m_springs[i].MassPoint1].isFixed) m_massPoints[m_springs[i].MassPoint1].force -= internalForce;
		if (!m_massPoints[m_springs[i].MassPoint2].isFixed) m_massPoints[m_springs[i].MassPoint2].force += internalForce;
	}

	// External and Damp and Mass application
	for (int i = 0; i < m_massPoints.size(); ++i)
	{
		if (m_massPoints[i].isFixed) continue;

		m_massPoints[i].force += m_externalForce;
		m_massPoints[i].force -= m_fDamping * m_massPoints[i].velocity;
		m_massPoints[i].force = m_massPoints[i].force / m_fMass;
	}
}

void MassSpringSystemSimulator::clearForces()
{
	for (int i = 0; i < m_massPoints.size(); ++i)
	{
		m_massPoints[i].force = Vec3(0.0f);
	}
}