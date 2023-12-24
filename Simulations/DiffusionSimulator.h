#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

class Grid {

/* Converting a 3D grid into a 1D array
* 
* width, height, depth of grid
* (x, y, z) input
* x + y*width + z*width*depth = elements(x)(y)(z)
*/ 

/*

Beispiel

	X_1 X_2 X_3 X_4 - width 4
	X_5 X_6 X_7 X_8
				  - height 2
Wanting the 3rd value becomes 
*/



public:

	//constructors
	Grid();
	Grid(int m, int n, int t);

	//functions
	float getValue(int i, int j, int m);

private:
	int m = 0; //width
	int n = 0; //height
	int t = 0; //time/depth

};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(float timestep);
	void diffuseTemperatureImplicit(float timestep);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid T;
	int n = 16; //width
	int m = 16; //length
	float alpha = 0.2; //diffusion coefficient
	float boundary_temp = 0.;
	float max_temp = 90.;
};

#endif