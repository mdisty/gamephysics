#include "MassSpringSystemSimulator.h"



//--------------------------------------------------------------------------------------
// Constructors
//--------------------------------------------------------------------------------------


MassSpringSystemSimulator::MassSpringSystemSimulator()
{

	massPoints = {};
	springs = {};

	m_iTestCase = 0;

	timeStep = 0.1;

	float m_fMass = 10.;
	float m_fStiffness = 40.;
	float m_fDamping = 0.;
	int m_iIntegrator = 0;
	
	addSpring(addMassPoint(Vec3{ 0,0,0 }, Vec3{ -1,0,0 }, false), 
		addMassPoint(Vec3{ 0,2,0 }, Vec3{ 1,0,0 }, false),1);
}

//--------------------------------------------------------------------------------------
//  UI Functions
//--------------------------------------------------------------------------------------


const char* MassSpringSystemSimulator::getTestCasesStr()
{
	//tabs on UI
	return "Demo1, Demo2, Demo3, Demo4, Demo5";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	case 3:break;
	case 4:break;
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
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;


	switch (m_iTestCase)
	{
	case 0:
	{	
		massPoints.clear();
		springs.clear();

		timeStep = 0.1;

		float m_fMass = 10.;
		float m_fStiffness = 40.;
		float m_fDamping = 0.;
		int m_iIntegrator = 0;

		addSpring(addMassPoint(Vec3{ 0,0,0 }, Vec3{ -1,0,0 }, false),
			addMassPoint(Vec3{ 0,2,0 }, Vec3{ 1,0,0 }, false), 1);
		
		eulerIntegration(0.1);

		break; 
	}
	case 1:
		break;
	case 2:
		break;
	case 3:
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
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
		m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	if (m_iTestCase == 0) {
		return;
	}
	switch (m_iIntegrator)
	{// handling different cases
	case EULER:
		eulerIntegration(timeStep);
		break;
	case LEAPFROG:
		break;
	case MIDPOINT:
		midpointIntegration(timeStep);
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



//--------------------------------------------------------------------------------------
//  Frame Functions
//--------------------------------------------------------------------------------------

void MassSpringSystemSimulator::drawRandomObjects()
{
	//DUC->setUpLighting(Vec3 EmissiveColor, Vec3 SpecularColor, float SpecularPower, Vec3 DiffuseColor);
	//DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1), 100, 0.6*Vec3(0.97,0.86,1));
}


//--------------------------------------------------------------------------------------
//  Specific Functions
//--------------------------------------------------------------------------------------


void MassSpringSystemSimulator::eulerIntegration(float step)
{
	//euler for one step

	MassPoint m1 = massPoints.at(0);
	MassPoint m2 = massPoints.at(1);

	Spring s = springs.at(0);
	
	// calc distance
	Vec3 d = m1.m_position - m2.m_position;
	float currentDistance = sqrt(m1.m_position.squaredDistanceTo(m2.m_position));
	float norm = normalize(d);

	std::cout << "Distance:" << d << "\n";
	std::cout << "CurrentDistance: " << currentDistance << "\n";

	//calculate force
	Vec3 f1 = -m_fStiffness * (currentDistance - s.initialLength)*norm;
	Vec3 f2 = -f1;

	std::cout << "F1:" << f1 << "\n";
	std::cout << "F2:" << f2 << "\n";

	//calculate Acceleration
	Vec3 a1 = f1/m_fMass;
	Vec3 a2 = f2/m_fMass;
	
	std::cout << "a1:" << a1 << "\n";
	std::cout << "a2:" << a2 << "\n";

	// calculate newPosition
	m1.m_position += step * m1.m_velocity;
	m2.m_position += step * m2.m_velocity;
	
	std::cout << "new Pos1:" << m1.m_position << "\n";
	std::cout << "new Pos2:" << m2.m_position << "\n";

	//calculate newVelocity
	m1.m_velocity =+ step * a1;
	m2.m_velocity =+ step * a2;

	std::cout << "new Vel1:" << m1.m_velocity << "\n";
	std::cout << "new Vel2:" << m2.m_velocity << "\n";
}

void MassSpringSystemSimulator::midpointIntegration(float step)
{
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

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	MassPoint mp{ position, Velocity, isFixed };
	massPoints.push_back(mp);
	return size(massPoints)-1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring s{ masspoint1, masspoint2, initialLength };
	springs.push_back(s);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return size(massPoints);
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return size(springs);
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return massPoints.at(index).m_position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return massPoints.at(index).m_velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}

/*
	Vec3 MassSpringSystemSimulator::calculateElasticForces(Spring s)
{
	Vec3 d = s.point1.m_position - s.point2.m_position;
	float currentDistance = sqrt(s.point1.m_position.squaredDistanceTo(s.point2.m_position));
	Vec3 force = -s.stiffness * (s.currentLength - s.initialLength) * (d / s.currentLength);
	
	return force;
}
*/

