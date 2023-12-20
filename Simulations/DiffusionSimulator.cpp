#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// rest to be implemented,
	T = Grid();
	M = 8;
	N = 8;
	diff_con = 0.5f;
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "M", TW_TYPE_INT8, &M, "min = 4");
	TwAddVarRW(DUC->g_pTweakBar, "N", TW_TYPE_INT8, &N, "min = 4");
	TwAddVarRW(DUC->g_pTweakBar, "Diffusion Constant", TW_TYPE_FLOAT, &diff_con, "min = 0");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	// to be implemented
	//

	M = 8;
	N = 8;
	T.resize(M, N);

	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float delta_time) {
// to be implemented
	int idx_i = -1;
	for (auto& vec : T.vvCells)
	{
		idx_i++;
		if (idx_i == 0 || idx_i == M-1) { continue;}
		int idx_j = -1;

		for (auto& cell : vec)
		{
			idx_j++;
			if (idx_j == 0 || idx_j == N-1) { continue; }

			cell.temp = delta_time * diff_con * 
				(
					(T.vvCells.at(idx_i+1).at(idx_j).temp - 2.0f * T.vvCells.at(idx_i).at(idx_j).temp + T.vvCells.at(idx_i - 1).at(idx_j).temp) // / ((1.0f/M) * (1.0f/M))
					+ 
					(T.vvCells.at(idx_i).at(idx_j+1).temp - 2.0f * T.vvCells.at(idx_i).at(idx_j).temp + T.vvCells.at(idx_i).at(idx_j - 1).temp) // / ((1.0f/N) * (1.0f/N))
				)
				- cell.temp;
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {
	// solve A T = b

	// This is just an example to show how to work with the PCG solver,
	const int nx = 5;
	const int ny = 5;
	const int nz = 5;
	const int N = nx * ny * nz;

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);

	// This is the part where you have to assemble the system matrix A and the right-hand side b!

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	// Final step is to extract the grid temperatures from the solution vector x
	// to be implemented
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	// Check is User Changed Grid Size, Resize/Reset if
	if (M != T.m|| N != T.n)
	{
		T.resize(M, N);
	}

	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	for (auto& vec : T.vvCells)
	{
		for (auto& cell : vec)
		{
			Vec3 col = Vec3(0.5f + cell.temp / 100.0f * 0.5f, 0.0f, 0.5f - cell.temp / 100.0f * 0.5f);
			DUC->setUpLighting(col, col, 0.1f, col);
			DUC->drawSphere(cell.pos, cell.size);
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
