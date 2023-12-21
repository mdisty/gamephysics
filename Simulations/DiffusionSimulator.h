#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

#include <vector>
#include <algorithm>
#include <array>

class Grid {
public:
	Grid();
	Grid(int32_t w, int32_t h, double value);

	std::vector<std::vector<double>>& getGrid();

	double getValue(size_t x, size_t y) const;
	void setValue(size_t x, size_t y, double v);

	int32_t getWidth() const;
	int32_t getHeight() const;
	
	/*
	@return: The min value of the whole matrix
	*/
	double getMin() const;
	/*
	@return: The max value of the whole matrix
	*/
	double getMax() const;

	/*
	Resets the grid with a new width, height and a default value
	*/
	void reset(int32_t w, int32_t h, double value);
	/*
	Resets the grid with a new width, height and a random default value in the range [min, max)
	*/
	void resetRandom(int32_t w, int32_t h, double min, double max);

	/*
	@return: The whole matrix as a vector with the size = w * h (colums stacked)
	*/
	std::vector<double> toVector() const;
	/*
	Inserts a whole vector with the size = w * h into the matrix (colums stacked)
	*/
	void insertVector(const std::vector<double>& v);

	void setBoundaryToZero();
private: 
	std::vector<std::vector<double>> temperaturGrid_{};
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
	void diffuseTemperatureExplicit(const float dt);
	void diffuseTemperatureImplicit(const float dt);


private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	
	Grid T;
	int32_t N;
	int32_t M;
	int32_t N_old;
	int32_t M_old;
	float Alpha;

	std::array<float, 3> hotColor;
	std::array<float, 3> coldColor;
	std::array<float, 3> zeroColor;
};

#endif