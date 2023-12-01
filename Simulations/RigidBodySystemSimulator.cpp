#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	click = { 0, 0 };
	oldClick = { 0, 0 };
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };
	m_externalForce = { 0.0f , 0.0f, 0.0f };
	gravity = 0.0f;
	speedFactor = 1.0f;
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
	m_externalForce = { 0.0f , 0.0f, 0.0f };
	gravity = 0.0f;
	speedFactor = 1.0f;
}

void RigidBodySystemSimulator::drawAllRigidBodies() {
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	for (const auto& rigidbody : rigidbodies) {
		DUC->setUpLighting(Vec3(),
			0.4 * Vec3(1, 1, 1),
			100,
			Vec3(randCol(eng), randCol(eng), randCol(eng)));

		Mat4 body = toObjectToWordMatrix(rigidbody);

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
	case 3: {
		drawAllRigidBodies();
		break; 
	}
	default: break;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	switch (m_iTestCase)
	{
	case 0: {
		cout << "----- Demo 1 -----" << endl;

		reset();

		addRigidBody(Vec3{0.0f, 0.0f, 0.0f}, Vec3{1.0f, 0.6f, 0.5f}, 2);
		setOrientationOf(0, Quat(Vec3{0.0f, 0.0f, 1.0f}, 90.0f * (M_PI / 180.0f)));
		applyForceOnBody(0, Vec3{ 0.3, 0.5, 0.25 }, Vec3{ 1.0f, 1.0f, 0.0f });

		calculateTimeStepForRigidbodies(2.0f);

		Vec3 x = Vec3(-0.3f, -0.5f, -0.25f) - getPositionOfRigidBody(0);
		Vec3 v = getLinearVelocityOfRigidBody(0) + cross(getAngularVelocityOfRigidBody(0), x);

		cout << "Linear Velocity: " << rigidbodies.at(0).velocity << endl;
		cout << "Angular Velocity: " << rigidbodies.at(0).angularVelocity << endl;
		cout << "World space velocity at (-0.3, -0.5, -0.25): " << v << endl;
		break; 
	}
	case 1: {
		cout << "----- Demo 2 -----" << endl;

		reset();

		addRigidBody(Vec3{ 0.0f, 0.0f, 0.0f }, Vec3{ 1.0f, 0.6f, 0.5f }, 2);
		setOrientationOf(0, Quat(Vec3{ 0.0f, 0.0f, 1.0f }, 90.0f * (M_PI / 180.0f)));
		applyForceOnBody(0, Vec3{ 0.3, 0.5, 0.25 }, Vec3{ 1.0f, 1.0f, 0.0f });

		break; 
	}
	case 2: {
		cout << "----- Demo 3 -----" << endl;

		TwAddVarRW(DUC->g_pTweakBar, "Speed Factor", TW_TYPE_FLOAT, &speedFactor, "step=0.1 min=0.0001");

		reset();

		addRigidBody(Vec3{ 1.0f, 0.0f, 0.0f }, Vec3{ 1.0f, 0.6f, 0.5f }, 2);
		setOrientationOf(0, Quat(Vec3{ 1.0f, 1.0f, 1.0f }, 30.0f * (M_PI / 180.0f)));
		applyForceOnBody(0, Vec3{ 1.0f, 0.0f, 0.0f }, Vec3{ -10.0f, 0.0f, 0.0f });

		addRigidBody(Vec3{ -1.0f, 0.0f, 0.0f }, Vec3{ 1.0f, 0.6f, 0.5f }, 2);
		applyForceOnBody(1, Vec3{ -1.0f, 0.0f, 0.0f }, Vec3{ 10.0f, 0.0f, 0.0f });

		break; 
	}
	case 3: {
		cout << "----- Demo 4 -----" << endl;

		TwAddVarRW(DUC->g_pTweakBar, "Speed Factor", TW_TYPE_FLOAT, &speedFactor, "step=0.1 min=0.0001");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &gravity, "step=0.1");

		reset();
		
		setGravity(-0.5f);

		addRigidBody(Vec3(1.0f, 1.0f, 0.0f), Vec3{ 1.0f, 0.6f, 0.5f }, 2);
		setOrientationOf(0, Quat(Vec3{ 1.0f, 1.0f, 1.0f }, 30.0f * (M_PI / 180.0f)));
		applyForceOnBody(0, Vec3{ 1.0f, 1.0f, 0.0f }, Vec3{ 0.0f, -1.0f, 0.0f });

		addRigidBody(Vec3{ 0.0f, -1.0f, 0.0f }, Vec3{ 6.0f, 0.6f, 6.0f  }, 2);
		setFixedOf(1, true);
		break;
	}
	default: break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Rotate direction depending on camera view point
	Vec3 defaultViewDirectinon{ 0.0f, 0.0f, 1.0f };
	Vec3 viewDirection{ DUC->g_camera.GetLookAtPt() - DUC->g_camera.GetEyePt() };
	normalize(viewDirection);

	float theta = acos(dot(defaultViewDirectinon, viewDirection) / (norm(defaultViewDirectinon) * norm(viewDirection)));

	theta = Vec3(DUC->g_camera.GetEyePt()).x < 0 ? theta : -theta;

	float cosTheta{ cos(theta) };
	float sinTheta{ sin(theta) };

	if (DUC->g_camera.IsMouseLButtonDown()) {
		Vec3 direction = Vec3(m_trackmouse.x - m_oldtrackmouse.x, m_oldtrackmouse.y - m_trackmouse.y, 0.0f);

		float x = cosTheta * direction.x + sinTheta * direction.z;
		float y = direction.y;
		float z = -sinTheta * direction.x + cosTheta * direction.z;

		Vec3 rotDirection{ x,y,z };
		normalize(rotDirection);

		m_externalForce = Vec3(0.0f, gravity, 0.0f) + 0.1f * rotDirection;
	}
	else {
		m_externalForce = Vec3(0.0f, gravity, 0.0f);
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase == 0) return;

	timeStep *= speedFactor;
	calculateTimeStepForRigidbodies(timeStep);
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
	rigidbodies.emplace_back(Rigidbody{position, size, static_cast<float>(mass)});
	
	// Calculate initial inertia tensor
	Mat4 inertiaTensorZero = calculateInitialInertiaTensor(getNumberOfRigidBodies() - 1);
	rigidbodies.at(getNumberOfRigidBodies() - 1).inertiaTensorZero = inertiaTensorZero;
	rigidbodies.at(getNumberOfRigidBodies() - 1).inertiaTensor = inertiaTensorZero;
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigidbodies.at(i).orientation = orientation.unit();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigidbodies.at(i).velocity = velocity;
}

void RigidBodySystemSimulator::setFixedOf(int i, bool fixed) {
	rigidbodies.at(i).fixed = fixed;
}

void RigidBodySystemSimulator::setGravity(float g) {
	gravity = g;
}

Mat4 RigidBodySystemSimulator::calculateInitialInertiaTensor(int rigidbodyIndex)
{
	// Formula from wikipedia
	Rigidbody body{ rigidbodies.at(rigidbodyIndex) };

	array<float, 3> components{};
	components.at(0) = 1.0f / 12.0f * body.mass * (body.size.y * body.size.y + body.size.z * body.size.z);
	components.at(1) = 1.0f / 12.0f * body.mass * (body.size.x * body.size.x + body.size.z * body.size.z);
	components.at(2) = 1.0f / 12.0f * body.mass * (body.size.x * body.size.x + body.size.y * body.size.y);
	Mat4 I{ components.at(0), 0.0f			  , 0.0f			, 0.0f,
			0.0f			, components.at(1), 0.0f			, 0.0f,
			0.0f			, 0.0f			  , components.at(2), 0.0f,
		    0.0f			, 0.0f			  , 0.0f			, 1.0f };

	return I.inverse();
}

void RigidBodySystemSimulator::calculateTimeStepForRigidbodies(float timeStep) {
	for (auto& rigidbody : rigidbodies) {
		if (rigidbody.fixed) continue;

		// Add gravity
		rigidbody.forces.push_back(tuple<Vec3, Vec3>(rigidbody.position, m_externalForce));

		// Calculate new torque
		Vec3 F{ 0.0f, 0.0f, 0.0f };
		Vec3 newTorque{ 0.0f, 0.0f, 0.0f };
		for (const auto& force : rigidbody.forces) {
			Vec3 relativePosition = get<0>(force) - rigidbody.position;

			F += get<1>(force);
			newTorque += cross(relativePosition, get<1>(force));
		}
		rigidbody.torque = newTorque;

		// Calculate new positions etc.
		rigidbody.position += timeStep * rigidbody.velocity;
		rigidbody.velocity += timeStep * (F / rigidbody.mass);

		Quat angularVelocity(rigidbody.angularVelocity.x, rigidbody.angularVelocity.y, rigidbody.angularVelocity.z, 0.0f );
		rigidbody.orientation += (timeStep / 2.0f) * angularVelocity * rigidbody.orientation;
		rigidbody.orientation = rigidbody.orientation.unit();

		rigidbody.angularMomentum += timeStep * rigidbody.torque;

		updateInertiaTensor(rigidbody);

		rigidbody.angularVelocity = rigidbody.inertiaTensor * rigidbody.angularMomentum;

		// Clear forces and torque
		rigidbody.forces.clear();
		rigidbody.torque = Vec3{ 0.0f, 0.0f, 0.0f };
	}

	for (int i = 0; i < rigidbodies.size(); ++i) {
		for (int k = i + 1; k < rigidbodies.size(); ++k) {
			calculateCollision(rigidbodies.at(i), rigidbodies.at(k), 1.0f);
		}
	}
}

void RigidBodySystemSimulator::updateInertiaTensor(Rigidbody& rigidbody) {
	Mat4 rotMat = rigidbody.orientation.getRotMat();
	Mat4 rotMatTransposed = rigidbody.orientation.getRotMat();
	rotMatTransposed.transpose();

	rigidbody.inertiaTensor = rotMat * rigidbody.inertiaTensorZero * rotMatTransposed;
}

void RigidBodySystemSimulator::calculateCollision(Rigidbody& rigidbodyA, Rigidbody& rigidbodyB, float bouncyness) {
	if (rigidbodyB == rigidbodyA) return;

	Mat4 objectA = toObjectToWordMatrix(rigidbodyA);
	Mat4 objectB = toObjectToWordMatrix(rigidbodyB);

	CollisionInfo collision = checkCollisionSAT(objectA, objectB);

	if (collision.isValid) {
		Vec3 n = collision.normalWorld;

		Vec3 xA = collision.collisionPointWorld - rigidbodyA.position;
		Vec3 xB = collision.collisionPointWorld - rigidbodyB.position;

		Vec3 vA = rigidbodyA.velocity + cross(rigidbodyA.angularVelocity, xA);
		Vec3 vB = rigidbodyB.velocity + cross(rigidbodyB.angularVelocity, xB);

		Vec3 relativeVel = vA - vB;
		if (dot(relativeVel, n) > 0) return;
		
		// Calculate Impule
		float impulse = calculateImpulse(rigidbodyA, rigidbodyB, relativeVel, n, xA, xB, bouncyness);

		// Update velocity, Angular momentum
		if (!rigidbodyA.fixed) rigidbodyA.velocity += impulse * n / rigidbodyA.mass;
		if (!rigidbodyB.fixed) rigidbodyB.velocity -= impulse * n / rigidbodyB.mass;

		if (!rigidbodyA.fixed) rigidbodyA.angularMomentum += cross(xA, n * impulse);
		if (!rigidbodyB.fixed) rigidbodyB.angularMomentum -= cross(xB, n * impulse);
	}
}

Mat4 RigidBodySystemSimulator::toObjectToWordMatrix(const Rigidbody& rigidbody) {
	Mat4 scale; scale.initScaling(rigidbody.size.x, rigidbody.size.y, rigidbody.size.z);
	Mat4 rotation = rigidbody.orientation.getRotMat();
	Mat4 translation; translation.initTranslation(rigidbody.position.x, rigidbody.position.y, rigidbody.position.z);

	return scale * rotation * translation;
}

const float RigidBodySystemSimulator::calculateImpulse(Rigidbody& rigidbodyA, Rigidbody& rigidbodyB, const Vec3& relativeVel, const Vec3& n, const Vec3& xA, const Vec3& xB, float bouncyness) {
	float impulseNumerator = dot((-relativeVel - bouncyness * relativeVel), n);

	float aFixed = rigidbodyA.fixed ? 0.0f : 1.0f;
	float bFixed = rigidbodyB.fixed ? 0.0f : 1.0f;

	float impulseDenominator = aFixed * 1.0f / rigidbodyA.mass + bFixed * 1.0f / rigidbodyB.mass +
		dot(aFixed * cross(rigidbodyA.inertiaTensor * cross(xA, n), xA) +
			bFixed * cross(rigidbodyB.inertiaTensor * cross(xB, n), xB), n);
	return impulseNumerator / impulseDenominator;
}

// Rigidbody class

bool RigidBodySystemSimulator::Rigidbody::operator==(const Rigidbody& b) const
{
	bool pos = this->position.x == b.position.x && this->position.y == b.position.y && this->position.z == b.position.z;
	bool size = this->size.x == b.size.x && this->size.y == b.size.y && this->size.z == b.size.z;
	bool mass = this->mass == b.mass;
	bool orientation = this->orientation.x == b.orientation.x && 
		this->orientation.y == b.orientation.y && 
		this->orientation.z == b.orientation.z &&
		this->orientation.w == b.orientation.w;
	bool velocity = this->velocity.x == b.velocity.x && this->velocity.y == b.velocity.y && this->velocity.z == b.velocity.z;
	return pos && size && mass && orientation && velocity;
}
