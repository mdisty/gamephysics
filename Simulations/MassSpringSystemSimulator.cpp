#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	massPoints = vector<MassPoint>();
	springs = vector<Spring>();
	m_externalForce = Vec3(0.0f, 0.0f, 0.0f );
	individualExternalForce = Vec3( 0.0f, 0.0f, 0.0f );
	m_iIntegrator = EULER;
	gravity = Vec3(0.0f, -9.81f, 0.0f);
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawDemo2()
{
	DUC->setUpLighting(Vec3(),
		0.4 * Vec3(1, 1, 1), 
		100, 
		Vec3(237.0 / 255.0, 36.0 / 255.0, 255.0 / 255.0));
	DUC->drawSphere(massPoints.at(0).position, 0.05f);

	DUC->setUpLighting(Vec3(), 
		0.4 * Vec3(1, 1, 1), 
		100, 
		Vec3(70.0 / 255.0, 52.0 / 255.0, 235.0 / 255.0));
	DUC->drawSphere(massPoints.at(1).position, 0.05f);
	
	DUC->beginLine();
	DUC->drawLine(massPoints.at(0).position, 
		Vec3(1.0, 1.0, 1.0), 
		massPoints.at(1).position, 
		Vec3(1.0, 1.0, 1.0));
	DUC->endLine();
}

void MassSpringSystemSimulator::drawDemo3()
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 
		100, 
		Vec3(237.0 / 255.0, 36.0 / 255.0, 255.0 / 255.0));
	DUC->drawSphere(massPoints.at(0).position, 0.05f);

	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 
		100,
		Vec3(70.0 / 255.0, 52.0 / 255.0, 235.0 / 255.0));
	DUC->drawSphere(massPoints.at(1).position, 0.05f);

	DUC->beginLine();
	DUC->drawLine(massPoints.at(0).position, 
		Vec3(1.0, 1.0, 1.0), 
		massPoints.at(1).position, 
		Vec3(1.0, 1.0, 1.0));
	DUC->endLine();
}

void MassSpringSystemSimulator::drawDemo4()
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	for (int i = 0; i < getNumberOfMassPoints(); ++i) {
		DUC->setUpLighting(Vec3(), 
			0.4 * Vec3(1, 1, 1), 
			100, Vec3(1.5 * randCol(eng), 
				0.6 * randCol(eng), 1.5 * randCol(eng)));
		DUC->drawSphere(massPoints.at(i).position, 0.05f);
	}
	
	for (int i = 0; i < getNumberOfSprings(); ++i) {
		DUC->beginLine();
		DUC->drawLine(massPoints.at(springs.at(i).masspoint1).position,  
			Vec3(1.5 * randCol(eng), 0.6 * randCol(eng), 1.5 * randCol(eng)),
			massPoints.at(springs.at(i).masspoint2).position,
			Vec3(1.5 * randCol(eng), 0.6*randCol(eng), 1.5 *randCol(eng)));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: break;
	case 1:
		drawDemo2();
		break;
	case 2: 
		drawDemo3();
		break;
	case 3: 
		drawDemo4();
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	switch (m_iTestCase) {
	case 0: {
		massPoints.clear();
		springs.clear();

		setMass(10);
		setStiffness(40);
		setDampingFactor(0);
		setExternalForce(Vec3( 0.0f, 0.0f, 0.0f ));
		setIntegrator(EULER);
		
		Vec3 p0{ 0., 0., 0. };
		Vec3 p1{ 0., 2., 0. };
		Vec3 v0{ -1., 0., 0. };
		Vec3 v1{ 1., 0., 0. };

		int ip0 = addMassPoint(p0, v0, false);
		int ip1 = addMassPoint(p1, v1, false);

		addSpring(ip0, ip1, 1.0);

		simulateTimestep(0.1);

		cout << "---- DEMO 1 ----" << endl;
		cout << "One timeStep (0.1) with Euler:" << endl;
		cout << "--Point 0: " << massPoints.at(0).position << endl;
		cout << "--Point 1: " << massPoints.at(1).position << endl;
		cout << "--Velocity 0: " << massPoints.at(0).veloctiy << endl;
		cout << "--Velocity 1: " << massPoints.at(1).veloctiy << endl;

		massPoints.clear();
		springs.clear();

		setMass(10);
		setStiffness(40);
		setDampingFactor(0);
		setExternalForce(Vec3(0.0f, 0.0f, 0.0f));
		setIntegrator(MIDPOINT);

		ip0 = addMassPoint(p0, v0, false);
		ip1 = addMassPoint(p1, v1, false);

		addSpring(ip0, ip1, 1.0);

		simulateTimestep(0.1);

		cout << "One timeStep (0.1) with Midpoint:" << endl;
		cout << "--Point 0: " << massPoints.at(0).position << endl;
		cout << "--Point 1: " << massPoints.at(1).position << endl;
		cout << "--Velocity 0: " << massPoints.at(0).veloctiy << endl;
		cout << "--Velocity 1: " << massPoints.at(1).veloctiy << endl;

		break;
	}
	case 1: {
		massPoints.clear();
		springs.clear();

		setMass(10);
		setStiffness(40);
		setDampingFactor(0);
		setExternalForce(Vec3{ 0.0f, 0.0f, 0.0f });
		setIntegrator(EULER);

		cout << "---- DEMO 2 ----" << endl;
		cout << "Timestep: " << "0.005" << endl;

		Vec3 p0{ 0., 0., 0. };
		Vec3 p1{ 0., 2., 0. };
		Vec3 v0{ -1., 0., 0. };
		Vec3 v1{ 1., 0., 0. };

		int ip0 = addMassPoint(p0, v0, false);
		int ip1 = addMassPoint(p1, v1, false);

		addSpring(ip0, ip1, 1.0);

		break;
	}
	case 2: {
		massPoints.clear();
		springs.clear();

		setMass(10);
		setStiffness(40);
		setDampingFactor(0);
		setExternalForce(Vec3{ 0.0f, 0.0f, 0.0f });
		setIntegrator(MIDPOINT);

		cout << "---- DEMO 3 ----" << endl;
		cout << "Timestep: " << "0.005" << endl;

		Vec3 p0{ 0., 0., 0. };
		Vec3 p1{ 0., 2., 0. };
		Vec3 v0{ -1., 0., 0. };
		Vec3 v1{ 1., 0., 0. };

		int ip0 = addMassPoint(p0, v0, false);
		int ip1 = addMassPoint(p1, v1, false);

		addSpring(ip0, ip1, 1.0);

		break;
	}
	case 3: {
		TwType integratorType = TwDefineEnum("Integrator", NULL, 0);
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", integratorType, &m_iIntegrator, " enum='0 {Euler}, 2 {Midpoint}' ");
		TwAddVarRO(DUC->g_pTweakBar, "How to interact", TW_TYPE_STDSTRING, &info, "");
		TwAddVarRO(DUC->g_pTweakBar, " ", TW_TYPE_STDSTRING, &info1, "");
		TwAddVarRO(DUC->g_pTweakBar, "  ", TW_TYPE_STDSTRING, &info2, "");
		TwAddVarRO(DUC->g_pTweakBar, "   ", TW_TYPE_STDSTRING, &info3, "");
		TwAddVarRO(DUC->g_pTweakBar, "    ", TW_TYPE_STDSTRING, &info4, "");
		TwAddVarRO(DUC->g_pTweakBar, "     ", TW_TYPE_STDSTRING, &info5, "");

		cout << "---- DEMO 4 ----" << endl;
		cout << "How to interact: " << endl;
		cout << "Click and drag your mouse to apply" << endl << "external forces to the objects in the scene." << endl;

		massPoints.clear();
		springs.clear();

		setMass(10);
		setStiffness(80);
		setDampingFactor(1);
		setExternalForce(gravity);

		// First tower

		Vec3 p0{ 0.0f, 2.0f              , 0.0f };

		Vec3 p1{ 0.0f        , 2.0f * 2.0f / 3.0f, -2.0f / 3.0f };
		Vec3 p2{ 2.0f / 3.0f , 2.0f * 2.0f / 3.0f, 2.0f / 3.0f };
		Vec3 p3{ -2.0f / 3.0f, 2.0f * 2.0f / 3.0f, 2.0f / 3.0f };
		
		Vec3 p4{ 0.0f        , 2.0f / 3.0f, -2.0f / 3.0f };
		Vec3 p5{ 2.0f / 3.0f , 2.0f / 3.0f, 2.0f / 3.0f };
		Vec3 p6{ -2.0f / 3.0f, 2.0f / 3.0f, 2.0f / 3.0f };

		Vec3 p7{ 0.0f        , 0, -2.0f / 3.0f };
		Vec3 p8{ 2.0f / 3.0f , 0, 2.0f / 3.0f };
		Vec3 p9{ -2.0f / 3.0f, 0, 2.0f / 3.0f };

		Vec3 v{ 0.0f, 0.0f, 0.0f };

		int ip0 = addMassPoint(p0, v, true);
		int ip1 = addMassPoint(p1, v, false);
		int ip2 = addMassPoint(p2, v, false);
		int ip3 = addMassPoint(p3, v, false);
		int ip4 = addMassPoint(p4, v, false);
		int ip5 = addMassPoint(p5, v, false);
		int ip6 = addMassPoint(p6, v, false);
		int ip7 = addMassPoint(p7, v, false);
		int ip8 = addMassPoint(p8, v, false);
		int ip9 = addMassPoint(p9, v, false);

		addSpring(ip0, ip1, 2.0f / 3.0f);
		addSpring(ip0, ip2, 2.0f / 3.0f);
		addSpring(ip0, ip3, 2.0f / 3.0f);

		addSpring(ip1, ip2, 2.0f / 3.0f);
		addSpring(ip2, ip3, 2.0f / 3.0f);
		addSpring(ip3, ip1, 2.0f / 3.0f);

		addSpring(ip1, ip4, 2.0f / 3.0f);
		addSpring(ip2, ip5, 2.0f / 3.0f);
		addSpring(ip3, ip6, 2.0f / 3.0f);

		addSpring(ip4, ip5, 2.0f / 3.0f);
		addSpring(ip5, ip6, 2.0f / 3.0f);
		addSpring(ip6, ip4, 2.0f / 3.0f);

		addSpring(ip4, ip7, 2.0f / 3.0f);
		addSpring(ip5, ip8, 2.0f / 3.0f);
		addSpring(ip6, ip9, 2.0f / 3.0f);

		addSpring(ip7, ip8, 2.0f / 3.0f);
		addSpring(ip8, ip9, 2.0f / 3.0f);
		addSpring(ip9, ip7, 2.0f / 3.0f);

		// Second quad

		Vec3 p01{ 2.0f, 2.0f, 1.0f };
		Vec3 p11{ 3.0f, 2.0f, 1.0f };
		Vec3 p21{ 2.0f, 1.0f, 1.0f };
		Vec3 p31{ 3.0f, 1.0f, 1.0f };

		Vec3 p41{ 2.0f, 2.0f, -1.0f };
		Vec3 p51{ 3.0f, 2.0f, -1.0f };
		Vec3 p61{ 2.0f, 1.0f, -1.0f };
		Vec3 p71{ 3.0f, 1.0f, -1.0f };

		int ip01 = addMassPoint(p01, v, false);
		int ip11 = addMassPoint(p11, v, false);
		int ip21 = addMassPoint(p21, v, false);
		int ip31 = addMassPoint(p31, v, false);
		int ip41 = addMassPoint(p41, v, false);
		int ip51 = addMassPoint(p51, v, false);
		int ip61 = addMassPoint(p61, v, false);
		int ip71 = addMassPoint(p71, v, false);

		addSpring(ip01, ip11, 1.0f);
		addSpring(ip01, ip21, 1.0f);
		addSpring(ip11, ip31, 1.0f);
		addSpring(ip21, ip31, 1.0f);

		addSpring(ip41, ip51, 1.0f);
		addSpring(ip41, ip61, 1.0f);
		addSpring(ip51, ip71, 1.0f);
		addSpring(ip61, ip71, 1.0f);

		addSpring(ip01, ip41, 1.0f);
		addSpring(ip11, ip51, 1.0f);
		addSpring(ip21, ip61, 1.0f);
		addSpring(ip31, ip71, 1.0f);

	    float diagLen = norm(p21 - p51);
		addSpring(ip21, ip51, diagLen);
		addSpring(ip01, ip71, diagLen);
		addSpring(ip31, ip41, diagLen);
		addSpring(ip11, ip61, diagLen);

		// Third pyramid

		Vec3 p02{ -2.0f, 1.0f, 1.0f };
		Vec3 p12{ -3.0f, 1.0f, 1.0f };
		Vec3 p22{ -2.5f, 1.0f, 0.0f };
		Vec3 p32{ -2.5f, 2.0f, 0.5f };

		int ip02 = addMassPoint(p02, v, false);
		int ip12 = addMassPoint(p12, v, false);
		int ip22 = addMassPoint(p22, v, false);
		int ip32 = addMassPoint(p32, v, false);

		addSpring(ip02, ip12, 1.0f);
		addSpring(ip12, ip22, 1.0f);
		addSpring(ip22, ip02, 1.0f);
		addSpring(ip32, ip02, 1.0f);
		addSpring(ip32, ip12, 1.0f);
		addSpring(ip32, ip22, 1.0f);

		break;
	}
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	if(m_iTestCase == 3) {
		// Rotate direction depending on camera view point
		Vec3 defaultViewDirectinon{ 0.0f, 0.0f, 1.0f };
		Vec3 viewDirection{ DUC->g_camera.GetLookAtPt() - DUC->g_camera.GetEyePt() };
		normalize(viewDirection);

		float theta = acos(dot(defaultViewDirectinon, viewDirection) / (norm(defaultViewDirectinon) * norm(viewDirection) ));

		theta = Vec3(DUC->g_camera.GetEyePt()).x < 0 ? theta : -theta;

		float cosTheta{ cos(theta) };
		float sinTheta{ sin(theta) };

		if (DUC->g_camera.IsMouseLButtonDown()) {
			Vec3 direction = Vec3(m_trackmouse.x - m_oldtrackmouse.x, m_trackmouse.y - m_oldtrackmouse.y, 0.0f);

			float x = cosTheta*direction.x + sinTheta*direction.z;
			float y = direction.y;
			float z = -sinTheta * direction.x + cosTheta * direction.z;

			Vec3 rotDirection{ x,y,z };
			normalize(rotDirection);

			m_externalForce = gravity + 4 * rotDirection;
		}
		else {
			if (m_externalForce.x > gravity.x) {
				m_externalForce -= Vec3(0.5 * timeElapsed, 0, 0);
			}

			if (m_externalForce.y > gravity.y) {
				m_externalForce -= Vec3(0, 0.5 * timeElapsed, 0);
			}
			
			if (m_externalForce.z > gravity.z) {
				m_externalForce -= Vec3(0, 0, 0.5 * timeElapsed);
			}

			if (m_externalForce.x < gravity.x) {
				m_externalForce += Vec3(0.5 * timeElapsed, 0, 0);
			}

			if (m_externalForce.y < gravity.y) {
				m_externalForce += Vec3(0, 0.5 * timeElapsed, 0);
			}

			if (m_externalForce.z < gravity.z) {
				m_externalForce += Vec3(0, 0, 0.5 * timeElapsed);
			}
		}
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase == 1 || m_iTestCase == 2) {
		timeStep = 0.005;
	}

	switch (m_iIntegrator) {
		case EULER: {
			calculateExplicitEulerStep(timeStep);
			break;
		}
		case MIDPOINT: {
			calculateMidpointStep(timeStep);
			break;
		}
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	oldClick.x = click.x;
	oldClick.y = click.y;
	click.x = x;
	click.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = m_trackmouse.x;
	m_oldtrackmouse.y = m_trackmouse.y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	MassPoint massPoint{ position, velocity, isFixed };
	massPoints.emplace_back(massPoint);
	return getNumberOfMassPoints() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring spring{ masspoint1, masspoint2, initialLength };
	springs.emplace_back(spring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return massPoints.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return massPoints.at(index).veloctiy;
}

void MassSpringSystemSimulator::setExternalForce(Vec3 force) 
{
	m_externalForce = force;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce += force;
}

// Integration Methods

void MassSpringSystemSimulator::calculateExplicitEulerStep(float timeStep)
{
	vector<Vec3> newPositions;
	vector<Vec3> newVelocities;

	for (size_t i = 0; i < massPoints.size(); ++i) {
		MassPoint currentPoint = massPoints.at(i);

		if (currentPoint.isFixed) {
			newVelocities.emplace_back(currentPoint.veloctiy);
			newPositions.emplace_back(currentPoint.position);
			continue;
		}

		// Calculate new velocity
		vector<Spring> pSprings;

		for (Spring s : springs) {
			if (s.masspoint1 == i) {
				pSprings.emplace_back(s);
			}
			else if (s.masspoint2 == i) {
				pSprings.emplace_back(s);
			}
		}

		Vec3 force;

		for (Spring s : pSprings) {
			force += calculateForce(s, i);
		}

		force += m_externalForce;

		Vec3 acceleration = (force - m_fDamping * currentPoint.veloctiy) / m_fMass;
		Vec3 pVelocityNext = currentPoint.veloctiy + timeStep * acceleration;

		// Calculate new position
		Vec3 pPositionNext = currentPoint.position + timeStep * currentPoint.veloctiy;

		newVelocities.emplace_back(pVelocityNext);
		newPositions.emplace_back(pPositionNext);
	}

	// Set new values
	for (size_t i = 0; i < massPoints.size(); ++i) {
		// Collision detection with ground plane
		if (newPositions.at(i).y <= -1.0f) {
			Vec3 n{ 0.0f, 1.0f, 0.0f };
			Vec3 v = newVelocities.at(i) - 2 * dot(newVelocities.at(i), n) * n;
			massPoints.at(i).veloctiy = 0.9 * v;
		}
		else {
			massPoints.at(i).position = newPositions.at(i);
			massPoints.at(i).veloctiy = newVelocities.at(i);
		}
	}
}

void MassSpringSystemSimulator::calculateMidpointStep(float timeStep)
{
	vector<Vec3> newPositions;
	vector<Vec3> newVelocities;
	vector<MassPoint> oldMassPoints(getNumberOfMassPoints());

	for (size_t i = 0; i < massPoints.size(); ++i) {
		MassPoint currentPoint = massPoints.at(i);
		oldMassPoints.at(i) = currentPoint;

		if (currentPoint.isFixed) {
			newVelocities.emplace_back(currentPoint.veloctiy);
			newPositions.emplace_back(currentPoint.position);
			continue;
		}

		vector<Spring> pSprings;

		// Calculate midpoint velocity
		for (Spring s : springs) {
			if (s.masspoint1 == i) {
				pSprings.emplace_back(s);
			}
			else if (s.masspoint2 == i) {
				pSprings.emplace_back(s);
			}
		}

		Vec3 force;

		for (Spring s : pSprings) {
			force += calculateForce(s, i);
		}

		force += m_externalForce;
		if (i == selectedMassPoint) {
			force += individualExternalForce;
		}

		Vec3 acceleration = force / m_fMass;
		Vec3 pVelocityMidstep = currentPoint.veloctiy + 0.5 * timeStep * acceleration;

		// Calculate midpoint position
		Vec3 pPositionMidstep = currentPoint.position + 0.5 * timeStep * currentPoint.veloctiy;

		newVelocities.emplace_back(pVelocityMidstep);
		newPositions.emplace_back(pPositionMidstep);
	}

	// Set midstep values
	for (size_t i = 0; i < massPoints.size(); ++i) {
		massPoints.at(i).veloctiy = newVelocities.at(i);
		massPoints.at(i).position = newPositions.at(i);
	}

	newVelocities.clear();
	newPositions.clear();

	for (size_t i = 0; i < massPoints.size(); ++i) {
		MassPoint currentPoint = massPoints.at(i);

		if (currentPoint.isFixed) {
			newVelocities.emplace_back(currentPoint.veloctiy);
			newPositions.emplace_back(currentPoint.position);
			continue;
		}

		vector<Spring> pSprings;

		// Calculate new velocity
		for (Spring s : springs) {
			if (s.masspoint1 == i) {
				pSprings.emplace_back(s);
			}
			else if (s.masspoint2 == i) {
				pSprings.emplace_back(s);
			}
		}

		Vec3 force;

		for (Spring s : pSprings) {
			force += calculateForce(s, i);
		}

		force += m_externalForce;
		if (i == selectedMassPoint) {
			force += individualExternalForce;
		}

		Vec3 accelerationMidstep = (force - m_fDamping * oldMassPoints.at(i).veloctiy) / m_fMass;
		Vec3 pVelocityNext = oldMassPoints.at(i).veloctiy + timeStep * accelerationMidstep;

		// Calculate new position
		Vec3 pPositionNext = oldMassPoints.at(i).position + timeStep * currentPoint.veloctiy;

		newVelocities.emplace_back(pVelocityNext);
		newPositions.emplace_back(pPositionNext);
	}

	// Set new values
	for (size_t i = 0; i < massPoints.size(); ++i) {
		// Collision detection with ground plane
		if (newPositions.at(i).y <= -1.0f) {
			Vec3 n{ 0.0f, 1.0f, 0.0f };
			Vec3 oldV = newVelocities.at(i);
			Vec3 v = oldV - 2 * dot(oldV, n) * n;
			massPoints.at(i).veloctiy = 0.9 * v;
		} else {
			massPoints.at(i).position = newPositions.at(i);
			massPoints.at(i).veloctiy = newVelocities.at(i);
		}
	}
}

Vec3 MassSpringSystemSimulator::calculateForce(Spring s, int pointIndex)
{
	Vec3 d{};
	if (s.masspoint1 == pointIndex) {
		d = massPoints.at(s.masspoint1).position - massPoints.at(s.masspoint2).position;
	}
	else {
		d = massPoints.at(s.masspoint2).position - massPoints.at(s.masspoint1).position;
	}

	float length = normalize(d);

	return -m_fStiffness * (length - s.length) * d;
}
