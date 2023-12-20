#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

struct Cell
{
	Vec3 pos{ 1.0f, 1.0f, 1.0f };
	float size{ 1.0f };
	float temp{ 0.0f };
	Cell(Vec3 pos, float size, float temp = 0.0f) : pos(pos), size(size), temp(temp) {}
};

class Grid {
public:
	// to be implemented
	int m{4};
	int n{4};
	vector<vector<Cell>> vvCells;

	void set_boundaries(float val = 0.0f)
	{
		for (int i = 0; i < m; ++i)
		{
			vvCells.at(i).at(0).temp = val;
			vvCells.at(i).at(n-1).temp = val;
		}
		for (int j = 0; j < n; ++j)
		{
			vvCells.at(0).at(j).temp = val;
			vvCells.at(m-1).at(j).temp = val;
		}
	}

	void reset_cells()
	{
		std::random_device rand_device;
		std::mt19937 generator(rand_device());
		std::uniform_real_distribution<float> distribution(-50.0f, 50.0f);

		for (auto& vector : vvCells)
		{
			for (auto& cell : vector)
			{
				cell.temp = distribution(generator);
			}
		}

		set_boundaries();
	}

	void resize(int i, int j)
	{
		if (i < 4) { i = 4; }
		if (j < 4) { j = 4; }

		m = i;
		n = j;
		vvCells.clear();

		float w_size = 1.0f / m;
		float h_size = 1.0f / n;
		float sphere_size = (w_size + h_size) * 0.5f;

		for (int i = 0; i < m; ++i)
		{
			vector<Cell> vCell;

			for (int j = 0; j < n; ++j)
			{
				
				vCell.push_back(Cell(Vec3(w_size*0.5f + i*w_size - 0.5f, h_size*0.5f + j*h_size - 0.5f, 0.0f), sphere_size));
			}

			vvCells.push_back(vCell);
		}

		reset_cells();
	}

	Grid(int m = 2, int n = 2) : m(m), n(n)
	{
		resize(m, n);
	}
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
	void diffuseTemperatureExplicit(float delta_time);
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
	int M;
	int N;
	float diff_con;
};

#endif