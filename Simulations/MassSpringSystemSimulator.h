#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
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
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void eulerIntegration(float h);
	void midpointIntegration(float h);
	void forcesCalculations();
	void clearForces();
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Structs
	struct MassPoint {
		Vec3 position, finalPosition, velocity, force;
		float mass;
		bool isFixed;

		// NOTE: This constructor was auto-generated with Visual Studio 2022, not written by me!
		

		MassPoint(const Vec3& position, const Vec3& velocity, const Vec3& force, float mass, bool isFixed)
			: position(position), velocity(velocity), finalPosition(position), force(force), mass(mass), isFixed(isFixed)
		{
		}
	};

	struct Spring {
		int MassPoint1, MassPoint2;
		float initialLenght, stiffness;

		// NOTE: This constructor was auto-generated with Visual Studio 2022, not written by me!
		Spring(int MassPoint1, int Masspoint2, float initialLenght, float stiffness)
			: MassPoint1(MassPoint1), MassPoint2(Masspoint2), initialLenght(initialLenght), stiffness(stiffness)
		{
		}
	};

	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	bool m_useMidpoint;

	std::vector<MassPoint> m_massPoints;
	std::vector<Spring> m_springs;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif