#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	click = { 0, 0 };
	oldClick = { 0, 0 };
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	rigidbodies.clear();

	click = { 0, 0 };
	oldClick = { 0, 0 };
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };
}

void RigidBodySystemSimulator::drawAllRigidBodies() {
	for (const auto& rigidbody : rigidbodies) {
		DUC->setUpLighting(Vec3(),
			0.4 * Vec3(1, 1, 1),
			100,
			Vec3(237.0 / 255.0, 36.0 / 255.0, 255.0 / 255.0));

		Mat4d scale{};
		scale.initScaling(rigidbody.size.x, rigidbody.size.y, rigidbody.size.z);
		Mat4d translation{};
		translation.initTranslation(rigidbody.position.x, rigidbody.position.y, rigidbody.position.z);

		Mat4 body = scale * rigidbody.orientation.getRotMat() * translation;

		DUC->drawRigidBody(body);
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: { 
		drawAllRigidBodies();
		break;
	}
	case 1: {
		drawAllRigidBodies();
		break;
	}
	case 2: {
		drawAllRigidBodies();
		break; 
	}
	case 3: break;
	default: break;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	switch (m_iTestCase)
	{
	case 0: {
		reset();

		addRigidBody(Vec3{0.0f, 0.0f, 0.0f}, Vec3{1.0f, 0.6f, 0.5f}, 2);
		setOrientationOf(0, Quat(Vec3{0.0f, 0.0f, 1.0f}, 90.0f * (M_PI / 180.0f)));
		applyForceOnBody(0, Vec3{ 0.3, 0.5, 0.25 }, Vec3{ 1.0f, 1.0f, 0.0f });

		calculateTimeStepForRigidbodies(2.0f);

		cout << "Linear Velocity: " << rigidbodies.at(0).velocity << endl;
		cout << "Angular Velocity: " << rigidbodies.at(0).angularVelocity << endl;
		break; 
	}
	case 1: {
		reset();

		addRigidBody(Vec3{ 0.0f, 0.0f, 0.0f }, Vec3{ 1.0f, 0.6f, 0.5f }, 2);
		setOrientationOf(0, Quat(Vec3{ 0.0f, 0.0f, 1.0f }, 90.0f * (M_PI / 180.0f)));
		applyForceOnBody(0, Vec3{ 0.3, 0.5, 0.25 }, Vec3{ 1.0f, 1.0f, 0.0f });

		break; 
	}
	case 2: {
		// TODO: Change to actually Demo 2
		reset();

		addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		applyForceOnBody(0, Vec3(0.0, 0.0f, 0.0), Vec3(0, 0, 200));
		for (int i = 0; i < 4; i++)
			simulateTimestep(0.1);

		cout << rigidbodies.at(0).angularVelocity << endl;
		break; 
	}
	case 3: break;
	default: break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// TODO: Calculate external forces
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase == 0) return;

	calculateTimeStepForRigidbodies(timeStep);
	// TODO: Create method to calculate one rigidbody timestep
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	oldClick.x = click.x;
	oldClick.y = click.y;
	click.x = x;
	click.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = m_trackmouse.x;
	m_oldtrackmouse.y = m_trackmouse.y;
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
	return rigidbodies.at(i).velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return rigidbodies.at(i).angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	// Add position and force as tuple to rigidbody
	rigidbodies.at(i).forces.emplace_back(tuple<Vec3, Vec3>{loc, force});
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	// Add rigidbody to vector
	rigidbodies.emplace_back(Rigidbody{position, size, mass});
	
	// Calculate initial inertia tensor
	rigidbodies.at(getNumberOfRigidBodies() - 1).inertiaTensorZero = calculateInitialInertiaTensor(getNumberOfRigidBodies() - 1);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidbodies.at(i).orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidbodies.at(i).velocity = velocity;
}

Mat4 RigidBodySystemSimulator::calculateInitialInertiaTensor(int rigidbodyIndex)
{
	Rigidbody body{ rigidbodies.at(rigidbodyIndex) };

	array<float, 3> p1{ - body.size.x / 2.0f, + body.size.y / 2.0f, + body.size.z / 2.0f };
	array<float, 3> p2{ + body.size.x / 2.0f, + body.size.y / 2.0f, + body.size.z / 2.0f };
	array<float, 3> p3{ - body.size.x / 2.0f, - body.size.y / 2.0f, + body.size.z / 2.0f };
	array<float, 3> p4{ + body.size.x / 2.0f, - body.size.y / 2.0f, + body.size.z / 2.0f };
	array<float, 3> p5{ - body.size.x / 2.0f, + body.size.y / 2.0f, - body.size.z / 2.0f };
	array<float, 3> p6{ + body.size.x / 2.0f, + body.size.y / 2.0f, - body.size.z / 2.0f };
	array<float, 3> p7{ - body.size.x / 2.0f, - body.size.y / 2.0f, - body.size.z / 2.0f };
	array<float, 3> p8{ + body.size.x / 2.0f, - body.size.y / 2.0f, - body.size.z / 2.0f };

	array<array<float, 3>, 8> points{ p1, p2, p3, p4, p5, p6, p7, p8 };
	array<float, 9> components{};

	int i = 0;

	// C_j,k = sum_n(m_n * x_n,j * x_n,k)
	for (int j = 0; j < 3; ++j) {
		for (int k = 0; k < 3; ++k) {
			float c = 0;
			for (int n = 0; n < 8; ++n) {
				c += points.at(n).at(j) * points.at(n).at(k);
			}
			c *= body.mass / 24.0f; // Why the fuck 24?!
			components.at(i) = c;
			++i;
		}
	}

	float traceC = components.at(0) + components.at(4) + components.at(8);

	Mat4 C{components.at(0), components.at(1), components.at(2), 0.0f,
		   components.at(3), components.at(4), components.at(5), 0.0f,
		   components.at(6), components.at(7), components.at(8), 0.0f,
		   0.0f			   , 0.0f			 , 0.0f			   , 0.0f };

	Mat4 id_trace{};
	id_trace.initScaling(traceC, traceC, traceC);

	Mat4 I0 = id_trace - C;
	I0.value[3][3] = 1.0f;

	Mat4 invI0 = I0.inverse();

	return invI0;
}

void RigidBodySystemSimulator::updateInertiaTensor(Rigidbody& rigidbody) {
	Mat4 rotMat = rigidbody.orientation.getRotMat();
	Mat4 rotMatTransposed = rigidbody.orientation.getRotMat();
	rotMatTransposed.transpose();

	rigidbody.inertiaTensor = rotMat * rigidbody.inertiaTensorZero * rotMatTransposed;
}

void RigidBodySystemSimulator::calculateTimeStepForRigidbodies(float timeStep) {
	for (auto& rigidbody : rigidbodies) {

		// Calculate Forces
		Vec3 F{ 0.0f, 0.0f, 0.0f };
		for (const auto& force : rigidbody.forces) {
			F += get<1>(force);
		}

		// Calculate new torque
		Vec3 newTorque{ 0.0f, 0.0f, 0.0f };
		for (const auto& force : rigidbody.forces) {
			Vec3 relativePosition = get<0>(force) - rigidbody.position;

			newTorque += cross(relativePosition, get<1>(force));
		}
		rigidbody.torque = newTorque;

		// Calculate new positions etc.
		rigidbody.position = rigidbody.position + timeStep * rigidbody.velocity;

		rigidbody.velocity = rigidbody.velocity + timeStep * (F / rigidbody.mass);

		Quat angularVelocity{ rigidbody.angularVelocity.x, rigidbody.angularVelocity.y, rigidbody.angularVelocity.z, 0.0f };
	
		rigidbody.orientation = rigidbody.orientation + (timeStep / 2.0f) * angularVelocity * rigidbody.orientation;
		rigidbody.orientation = rigidbody.orientation.norm() < 2e-8 ? rigidbody.orientation : rigidbody.orientation.unit(); // Normalize? Not sure if this is necessary

		rigidbody.angularMomentum = rigidbody.angularMomentum + timeStep * rigidbody.torque;

		updateInertiaTensor(rigidbody);

		rigidbody.angularVelocity = rigidbody.inertiaTensor * rigidbody.angularMomentum;

		// Clear forces and torque
		rigidbody.forces.clear();
		rigidbody.torque = Vec3{ 0.0f, 0.0f, 0.0f };
	}
}