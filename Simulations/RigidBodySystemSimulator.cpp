#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;

	// data about objects in the simulation TODO
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo1, Demo2, Demo3, Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	// possibly more stuff TODO
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:break; // Manual calculation -> prints solution
	case 1: break; // Rigid box
	case 2: break; // Two rigid boxes
	case 3: break; // at least four boxes
	}

	// TODO: draw frames for demo 1, 2 and 4 -> do maybe individual function
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: {
		cout << ".------------.\n | Demo 1 | \n '------------' \n";
		//cout << "| Text Fancy |";
		//cout << "'------------'";


		rigidbodies.clear(); 

		simulateTimestep(0.1);





		break;
	}

	default: {
		cout << ".------------.\n | Default | \n '------------' \n";
		//cout << "| Text Fancy |";
		//cout << "'------------'";
		break;
	}
		
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// todo for Demo 2 & 3 ! -> for mouse interaction stuff !
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase) {
	case 0: timeStep = 2; break;
	case 1: timeStep = 0.01; 
		explicitEulerIntegration(timeStep);
		break;
	case 2: 
		explicitEulerIntegration(timeStep);
		break;
	case 3: 
		explicitEulerIntegration(timeStep);
		break;
	}

	// TODO: Set different simulation timesteps
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	// TODO: maybe change this
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	// TODO: maybe change this
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return rigidbodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return rigidbodies.at(i).position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return rigidbodies.at(i).linearVel;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return rigidbodies.at(i).angularVel;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody rigidbody{ position, size, mass, Vec3(0.), Vec3(0.), Vec3(0.), Quat(0.)};
	rigidbodies.emplace_back(rigidbody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidbodies.at(i).orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidbodies.at(i).velocity = velocity;
}














