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
	/*
	Simulates one timeStep depending on the selected demo.
	*/
	void simulateTimestep(float timeStep);
	/*
	Updates the oldClick variable and click variable with the
	old and current click position.
	*/
	void onClick(int x, int y);
	/*
	Updates the m_oldtrackmouse variable and m_trackmouse variable with the
	old and current mouse position.
	*/
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

	/*
	Sets the external force to the passed value.
	*/
	void setExternalForce(Vec3 force);

	/*
	Adds the passed value to the external force.
	*/
	void applyExternalForce(Vec3 force);

	// Integration Methods
	
	/*
	Calculates one explicit euler step with the given timeStep.
	Saves everything in the massPoints and springs vectors.
	*/
	void calculateExplicitEulerStep(float timeStep);

	/*
	Calculates one midstep step with the given timeStep. 
	Saves everything in the massPoints and springs vectors.
	*/
	void calculateMidpointStep(float timeStep);

	/*
	Calculates the force for a specific point and a spring with Hookslaw.
	\return The calculated force as a Vec3.
	*/
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

	string info = "Click and drag";
	string info1 = "your mouse";
	string info2 = "to apply";
	string info3 = "external forces";
	string info4 = "to the objects";
	string info5 = "in the scene.";

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