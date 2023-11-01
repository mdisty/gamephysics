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

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: break; // Demo 1
	case 1: break; // Demo 2
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

		setMass(10);
		setStiffness(40);
		setDampingFactor(0);
		
		Vec3 p0{ 0, 0, 0 };
		Vec3 p1{ 0, 2, 0 };
		Vec3 v0{ -1, 0, 0 };
		Vec3 v1{ 1, 0, 0 };

		addMassPoint(p0, v0, false);
		addMassPoint(p1, v1, false);

		break;
	}
	case 1: break;
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
	//TODO
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
