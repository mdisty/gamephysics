#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	massPoints = vector<MassPoint>();
	springs = vector<Spring>();
	//TODO
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
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(237.0 / 255.0, 36.0 / 255.0, 255.0 / 255.0));
	DUC->drawSphere(massPoints.at(0).position, 0.05f);

	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(70.0 / 255.0, 52.0 / 255.0, 235.0 / 255.0));
	DUC->drawSphere(massPoints.at(1).position, 0.05f);
	
	DUC->beginLine();
	DUC->drawLine(massPoints.at(0).position, Vec3(1.0, 1.0, 1.0), massPoints.at(1).position, Vec3(1.0, 1.0, 1.0));
	DUC->endLine();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: break; // Demo 1
	case 1:
		drawDemo2();
		break;
	case 2: break; // Demo 3
	case 3: break; // Demo 4
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
		
		Vec3 p0{ 0., 0., 0. };
		Vec3 p1{ 0., 2., 0. };
		Vec3 v0{ -1., 0., 0. };
		Vec3 v1{ 1., 0., 0. };

		addMassPoint(p0, v0, false);
		addMassPoint(p1, v1, false);

		addSpring(0, 1, 1.0);

		calculateExplicitEulerStep(0.1);

		cout << "---- DEMO 1 ----" << endl;
		cout << "Point 0: " << massPoints.at(0).position << endl;
		cout << "Point 1: " << massPoints.at(1).position << endl;
		cout << "Velocity 0: " << massPoints.at(0).veloctiy << endl;
		cout << "Velocity 1: " << massPoints.at(1).veloctiy << endl;

		break;
	}
	case 1: {
		massPoints.clear();
		springs.clear();

		setMass(10);
		setStiffness(40);
		setDampingFactor(0);

		Vec3 p0{ 0., 0., 0. };
		Vec3 p1{ 0., 2., 0. };
		Vec3 v0{ 1., 0., 0. };
		Vec3 v1{ -1., 0., 0. };

		addMassPoint(p0, v0, false);
		addMassPoint(p1, v1, false);

		addSpring(0, 1, 1.0);

		break;
	}
	case 2: break;
	case 3: break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// TODO: Do some external force calculations. For example mouse input
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0: {
		break;
	}
	case 1: 
		calculateExplicitEulerStep(0.005);
		break; // Demo 2
	case 2: break; // Demo 3
	case 3: break; // Demo 4
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

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	// TODO
}

// Integration Methods

void MassSpringSystemSimulator::calculateExplicitEulerStep(float timeStep)
{
	vector<Vec3> newPositions{};
	vector<Vec3> newVelocities{};

	for (size_t i = 0; i < massPoints.size(); ++i) {
		MassPoint currentPoint = massPoints.at(i);

		// Calculate new velocity
		vector<Spring> pSprings{};

		for (Spring s : springs) {
			if (s.masspoint1 == i) {
				pSprings.emplace_back(s);
			}
			else if (s.masspoint2 == i) {
				pSprings.emplace_back(s);
			}
		}

		Vec3 force{};

		for (Spring s : pSprings) {
			force += calculateForce(s, i);
		}

		Vec3 acceleration = (force - m_fDamping * currentPoint.veloctiy) / m_fMass;
		Vec3 pVelocityNext = currentPoint.veloctiy + timeStep * acceleration;

		// Calculate new position
		Vec3 pPositionNext = currentPoint.position + timeStep * currentPoint.veloctiy;

		newVelocities.emplace_back(pVelocityNext);
		newPositions.emplace_back(pPositionNext);
	}

	// Set new values
	for (size_t i = 0; i < massPoints.size(); ++i) {
		massPoints.at(i).position = newPositions.at(i);
		massPoints.at(i).veloctiy = newVelocities.at(i);
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
