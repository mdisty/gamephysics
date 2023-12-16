#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

#include <vector>

class Grid {
public:
	Grid();
	Grid(int32_t w, int32_t h, float value);

	std::vector<std::vector<float>>& getGrid();

	float getValue(size_t x, size_t y) const;
	void setValue(size_t x, size_t y, float v);

	int32_t getWidth() const;
	int32_t getHeight() const;

	void reset(int32_t w, int32_t h, float value);
private:
	std::vector<std::vector<float>> temperaturGrid_{};
	int32_t w_;
	int32_t h_;
};



class DiffusionSimulator : public Simulator{
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
	void diffuseTemperatureExplicit();
	void diffuseTemperatureImplicit();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid T;
};

#endif