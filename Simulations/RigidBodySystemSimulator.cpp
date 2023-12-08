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
		
		// step by step solution with 1 rigid body h = 2


		// INITIALIZE ----------------------------------------

		rigidbodies.clear();

		addRigidBody(Vec3{ 0.,0.,0. }, Vec3{ 1., 0.6, 0.5 }, 2);
		// Quaternion (const vector3Dim<Scalar>& axis, Scalar angle) -> angle in Radians ?
		setOrientationOf(0, Quat(0, 0, 0, M_PI/2));
		setVelocityOf(0, Vec3{0.,0.,0.});
		setExternalForce(Vec3{ 1.,1.,0 });
		rigidbodies.at(0).torque = Vec3{ 0.,0.,0. };
		rigidbodies.at(0).torque = Vec3{ 0.3,0.5,0.25 };

		simulateTimestep(2.0);

		Vec3 pos = Vec3(-0.3, -0.5, -0.25);
		Vec3 linvel = getLinearVelocityOfRigidBody(0);
		Vec3 angvel = getAngularVelocityOfRigidBody(0);


		cout << "Point:" << pos << endl;
		cout << "Velocity: " << linvel + cross(angvel, pos) << endl;
		cout << "Linear Velocity: " << linvel << endl;
		cout << "Angular Velocity: " << angvel << endl;

		//update the World Space positions of the Points based on the new orientation r
		//

		break;
	}

	default: {
		cout << ".------------.\n | Default | \n '------------' \n";
		//cout << "| Default |";
		//cout << "'------------'";
		break;
	}
		
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// todo for Demo 2 & 3 !
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	
	for (auto& rigidb : rigidbodies)
	{
		// 1) Torque -------------------
		// calculate forces & convert them to torque q

		rigidb.torque += cross(rigidb.loc - rigidb.position, rigidb.force);

		// 2) Euler Step ----------------
		// position: x_cm <- x_cm + h*v_cm

		rigidb.position += rigidb.velocity * timeStep;

		//velocity: v_cm <- v_cm + h*F/M

		rigidb.velocity += rigidb.force / rigidb.mass * timeStep;

		// 3) Orientation  ----------------
		// integrate the orientation r using the angular velocity w
		// quaternion multiplication
		rigidb.orientation = rigidb.orientation + ((timeStep / 2) *
			Quat(rigidb.angularVel.x, rigidb.angularVel.y, rigidb.angularVel.z, 0.) * rigidb.orientation);
		//normalization -> unit function
		rigidb.orientation = rigidb.orientation.unit();

		// 4) Angular velocity  ----------------
		//integrate the angular momentum: L(t+h) = h*q
		rigidb.angularMomentum = rigidb.angularMomentum + timeStep * rigidb.torque;

		// 5) Intertia Tensor
		// update (inverse) I
		rigidb.inverseInertia = rigidb.orientation.getRotMat() * rigidb.inverseInertia *
								rigidb.orientation.getRotMat().inverse();
		// 6) Angular Velocity
		//update angular velocity using Intertia Tensor I and Angular momentum L
		rigidb.angularVel = rigidb.inverseInertia * rigidb.angularMomentum;

		//clear
		rigidb.force = Vec3(0., 0., 0.);
		rigidb.torque = Vec3(0., 0., 0.);

	}

	// calculate collisions

	//TODO

		
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
	rigidbodies.at(i).force += force;
	rigidbodies.at(i).loc = loc;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	// Initial inverse Intertia Tensor
	// since we have to multiplicate with Mat4d -> calculate diagonal double values first
	// (custom analytic expressions for rectangular box from the Slide)

	/*
	width w 
	height h
	depth d
	mass m

	1/12 * m (h^2 + d^2		0				0
			0			1/12m(w^2+h^2)		0
			0				0			1/12m(w^2+d^2)
	*/

	RigidBody rigidbody{ position, size, mass };

	float w = size.x; // ! CHECK IF RIGHT ?
	float h = size.z;
	float d = size.y;

	//double m_00 = (1.0 / 12.0) * mass * (pow(h, 2) + pow(d, 2));
	//double m_11 = (1.0 / 12.0) * mass * (pow(w, 2) + pow(h, 2));
	//double m_22 = (1.0 / 12.0) * mass * (pow(w, 2) + pow(d, 2));

	Mat4 inertiaTensor;
	inertiaTensor.initId();
	inertiaTensor.value[0][0] = (1.0 / 12.0) * mass * (pow(h, 2) + pow(d, 2));
	inertiaTensor.value[1][1] = (1.0 / 12.0) * mass * (pow(w, 2) + pow(h, 2));
	inertiaTensor.value[2][2] = (1.0 / 12.0) * mass * (pow(w, 2) + pow(d, 2));

	rigidbody.inverseInertia = inertiaTensor.inverse();

	rigidbodies.emplace_back(rigidbody);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidbodies.at(i).orientation = orientation.unit();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidbodies.at(i).velocity = velocity;
}

void RigidBodySystemSimulator::setExternalForce(Vec3 force)
{
	m_externalForce = force;
}