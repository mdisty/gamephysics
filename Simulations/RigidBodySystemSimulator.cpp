#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	int m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: // Demo 1
		break;
	case 1: // Demo 2
		// TODO: Implement initUI for Demo 2
		break;
	case 2: // Demo 3
		// TODO: Implement initUI for Demo 3
		break;
	case 3: // Demo 4
		// TODO: Implement initUI for Demo 4
		break;
	default:break;
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: // Demo 1
		break;
	case 1: // Demo 2
		// TODO: Implement drawFrame for Demo 2
		break;
	case 2: // Demo 3
		// TODO: Implement drawFrame for Demo 3
		break;
	case 3: // Demo 4
		// TODO: Implement drawFrame for Demo 4
		break;
	default:break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// TODO: Implement externalForces Calculations
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0: // Demo 1
		break;
	default:
		simulateExplicitEuler(timeStep);
		break;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
	{
		cout << "Demo 1!\n";
		m_rigidBodies.clear();
		addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), static_cast<float>(M_PI) * 0.5f));

		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
		simulateExplicitEuler(2.0f);

		Vec3 xa_world = Vec3(-0.3f, -0.5f, -0.25f) - getPositionOfRigidBody(0);
		Vec3 velocityA = getLinearVelocityOfRigidBody(0) + cross(getAngularVelocityOfRigidBody(0), xa_world);
		cout << "Updated Linear Velocity: " << getLinearVelocityOfRigidBody(0) << endl;
		cout << "Updated Angular Velocity: " << getAngularVelocityOfRigidBody(0) << endl;
		cout << "Updated World Velocity: " << velocityA << endl;
	}
	break;
	case 1:
		cout << "Demo 2!\n";
		// TODO: Implement notifyCaseChanged/SceneReset for Demo 2
		break;
	case 2:
		cout << "Demo 3!\n";
		// TODO: Implement notifyCaseChanged/SceneReset for Demo 3
		break;
	case 3:
		cout << "Demo 4!\n";
		// TODO: Implement notifyCaseChanged/SceneReset for Demo 4
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_rigidBodies.at(i).v3_pos_cm;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_rigidBodies.at(i).v3_vel_cm;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_rigidBodies.at(i).v3_w;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_rigidBodies.at(i).v3_force += force;
	Vec3 pointRel = loc - m_rigidBodies.at(i).v3_pos_cm;
	Vec3 crossProduct = cross((pointRel), force);
	m_rigidBodies.at(i).v3_torque += crossProduct;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 pos, Vec3 size, int mass)
{
	m_rigidBodies.push_back(RigidBody(pos, size, mass));
}

void RigidBodySystemSimulator::simulateExplicitEuler(float timeElapsed)
{
	for (auto& RB : m_rigidBodies)
	{
		// Euler Update Position & Velocity
		RB.v3_pos_cm = RB.v3_pos_cm + timeElapsed * RB.v3_vel_cm;
		RB.v3_vel_cm = RB.v3_vel_cm + (timeElapsed * RB.v3_force / static_cast<float>(RB.i_Mass));

		// Update Rotation
		RB.q_rotation = RB.q_rotation + (timeElapsed / 2.0f) * Quat(RB.v3_w.x, RB.v3_w.y, RB.v3_w.z, 0.0f) * RB.q_rotation;
		RB.q_rotation = RB.q_rotation.unit();

		// Update Momentum
		RB.v3_L = RB.v3_L + timeElapsed * RB.v3_torque;

		// Update Intertia
		Mat4 RotMatTrans = RB.q_rotation.getRotMat();
		RotMatTrans.transpose();
		RB.m4_I_inv = RB.q_rotation.getRotMat() * RB.m4_I_0_inv * RotMatTrans;

		// Update Angular Velocity
		RB.v3_w = RB.m4_I_inv * RB.v3_L; // TODO: If something breaks, look at Tip 8
	}
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_rigidBodies.at(i).q_rotation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_rigidBodies.at(i).v3_vel_cm = velocity;
}
