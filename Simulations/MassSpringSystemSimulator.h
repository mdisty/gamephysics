#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

#include <vector>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();

	// Structs
	struct MassPoint {
		Vec3 position;
		Vec3 veloctiy;
		bool isFixed;
	};

	struct Spring {
		int masspoint1;
		int masspoint2;
		float length;
	};
	
	// UI Functions

	/*
	* Creates the test cases in the menu
	*/
	const char * getTestCasesStr();

	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawDemo2();
	void drawDemo3();
	void drawDemo4();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void setExternalForce(Vec3 force);
	void applyExternalForce(Vec3 force);

	// Integration Methods
	void calculateExplicitEulerStep(float timeStep);
	void calculateMidpointStep(float timeStep);
	Vec3 calculateForce(Spring s, int pointIndex);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	int selectedMassPoint;
	Vec3 gravity;

	char info[100] = "Click and drag";
	char info1[100] = "your mouse";
	char info2[100] = "to apply external forces";
	char info3[100] = "to the objects in the scene.";

	vector<MassPoint> massPoints;
	vector<Spring> springs;

	// UI Attributes
	Vec3 m_externalForce;
	Vec3 individualExternalForce;
	Point2D m_mouse;
	Point2D click;
	Point2D oldClick;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif