#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,

#define TESTCASEUSEDTORUNTEST 2



class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();

	//Structs
	struct RigidBody {
		Vec3 position;
		Vec3 size;
		float mass;
		Vec3 velocity;
		Vec3 angularVel;
		Vec3 linearVel;
		Vec3 angularMomentum;
		Quat orientation;

		Mat4f InertiaTensor;
		Mat4f interiaTensorInit;
		vector<tuple<Vec3, Vec3>> forces; // pos, force
		Vec3 torque;

		bool isFixed{ false };
		//bool operator==(const RigidBody& b) const;

		RigidBody(Vec3 position, Vec3 size, float mass) : position{ position }, size{ size }, mass{ mass } {};
	};

	struct RigidBodySystem {

	};
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
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

	// TODO name
	void setExternalForce(Vec3 force);
	Vec3 calculateForces(int pointIndex);
	void calculateTorque(Vec3 pos, Vec3 force, int i);
	void integrateOrientation(Vec3 w, Quat orientation, float timestep);
	void calculateAngularMomentum(Vec3 L, float timestep, Vec3 torque);
	void updateIntertiaTensor(Mat4 rot, Mat4 intertiaTensor);
	void eulerStep(float timestep);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	vector<RigidBody> rigidbodies;

	Vec3 m_externalForce;
	Mat4f xRotation;
	Mat4f yRotation;
	Mat4f zRotation;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif