#ifndef DIFFUSIONSPRINGSYSTEMSIMULATOR_h
#define DIFFUSIONSPRINGSYSTEMSIMULATOR_h

#include "Simulator.h"
#include "SpringSystem.h"
#include "Ray.h"

typedef enum { ADDNODE, CHANGETEMP } TOOL;

class DiffusionSpringSystemSimulator: public Simulator{
public:
	// Construtors
	DiffusionSpringSystemSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Ray Functions
	Ray getRay(Point2D mouse);

	// Attributes
	Point2D m_mouse{};
	Point2D m_trackmouse{};
	Point2D m_oldtrackmouse{};
private:
	SpringSystem springSystem_;

	bool startSimulation_{ false };

	std::array<float, 3> hotColor;
	std::array<float, 3> coldColor;
	std::array<float, 3> zeroColor;

	bool changeTemp_{ false };
	bool selectedSphere_{ false };
	float temperature_{ 0.0 };

	TOOL m_tool = ADDNODE;

	int sphereIndex = -1;
};

#endif