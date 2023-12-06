#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

struct RigidBody
{
	Vec3 v3_size;
	int i_Mass;
	Vec3 v3_pos_cm;
	Vec3 v3_vel_cm{ Vec3(0.0f, 0.0f, 0.0f) };
	Vec3 v3_force{ Vec3(0.0f, 0.0f, 0.0f) };
	Vec3 v3_acc_cm{ Vec3(0.0f, 0.0f, 0.0f) };
	Quat q_rotation{ Quat(0.0f, 0.0f, 0.0f, 1.0f) };
	Vec3 v3_w{ Vec3(0.0f, 0.0f, 0.0f) };
	Vec3 v3_torque{ Vec3(0.0f, 0.0f, 0.0f) };
	Mat4 m4_I_0_inv{ Mat4(0.0f) };
	Mat4 m4_I_inv{ Mat4(0.0f) };
	Vec3 v3_L{ Vec3(0.0f, 0.0f, 0.0f) };

	RigidBody(Vec3 v3_pos_cm, Vec3 v3_size, int i_Mass) : v3_pos_cm{ v3_pos_cm }, v3_size{v3_size}, i_Mass{i_Mass}
	{
		// Init Inertia Tensor (0 and non-0)
		float ZeroZero = static_cast<float>(i_Mass) * (v3_size.y * v3_size.y + v3_size.z * v3_size.z) / 12.0f;
		float OneOne = static_cast<float>(i_Mass) * (v3_size.x * v3_size.x + v3_size.y * v3_size.y) / 12.0f;
		float TwoTwo = static_cast<float>(i_Mass) * (v3_size.x * v3_size.x + v3_size.z * v3_size.z) / 12.0f;
		m4_I_0_inv.initScaling(ZeroZero, TwoTwo, OneOne);
		m4_I_0_inv = m4_I_0_inv.inverse();
		m4_I_inv = m4_I_0_inv;
	}
};

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
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
	void simulateExplicitEuler(float timeElapsed);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce{};
	vector<RigidBody> m_rigidBodies{};

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif