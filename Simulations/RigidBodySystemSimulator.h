#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "collisionDetect.h"
#include <array>

//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h"

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();

	// Struct
	struct Rigidbody {
		Vec3 position;
		Vec3 size;
		float mass;
		Quat orientation{ 0.0f, 0.0f, 0.0f, 1.0f };
		Vec3 velocity{ 0.0f, 0.0f, 0.0f };
		Vec3 angularVelocity{ 0.0f, 0.0f, 0.0f };
		Vec3 angularMomentum{ 0.0f, 0.0f, 0.0f };
		Mat4 inertiaTensor{ 0.0f }; // Traegheitsmoment
		Mat4 inertiaTensorZero{ 0.0f };

		vector<tuple<Vec3, Vec3>> forces; // <position, force>
		Vec3 torque{ 0.0f, 0.0f, 0.0f };

		bool fixed{ false };

		bool operator==(const Rigidbody& b) const;

		Rigidbody(Vec3 position, Vec3 size, float mass) : position{ position }, size{ size }, mass{ mass } {};
	};

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawAllRigidBodies();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);
	void setFixedOf(int i, bool fixed);
	void setGravity(float gravtiy);

	// Physics calculations
	Mat4 calculateInitialInertiaTensor(int rigidbodyIndex);
	void calculateTimeStepForRigidbodies(float timeStep);
	void updateInertiaTensor(Rigidbody& rigidbody);
	void calculateCollision(Rigidbody& rigidbody, Rigidbody& rigidbodyB, float bouncyness);
	Mat4 toObjectToWordMatrix(const Rigidbody& rigidbody);
	const float calculateImpulse(const Rigidbody& rigidbodyA, const Rigidbody& rigidbodyB, const Vec3& relativeVel, const Vec3& n, const Vec3& xA, const Vec3& xB, float bouncyness);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem;
	Vec3 m_externalForce;
	vector<Rigidbody> rigidbodies{};
	float gravity;

	// UI Attributes
	Point2D click;
	Point2D oldClick;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif