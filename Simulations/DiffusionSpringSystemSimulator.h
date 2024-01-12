#ifndef DIFFUSIONSPRINGSYSTEMSIMULATOR_h
#define DIFFUSIONSPRINGSYSTEMSIMULATOR_h

#include "Simulator.h"

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

	// Specific Functions

	// Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
private:
};

#endif