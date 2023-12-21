#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// Values obtained from Demo Video
	T = Grid();
	M = 15;
	N = 15;
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
	// Fields for Grid Size and Diffusion Constant
	TwAddVarRW(DUC->g_pTweakBar, "M", TW_TYPE_INT32, &M, "min = 4");
	TwAddVarRW(DUC->g_pTweakBar, "N", TW_TYPE_INT32, &N, "min = 4");
	TwAddVarRW(DUC->g_pTweakBar, "Diffusion Constant", TW_TYPE_DOUBLE, &diff_con, "min = 0");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	
	// Reset Grid on CaseChange

	M = 15;
	N = 15;
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

void DiffusionSimulator::diffuseTemperatureExplicit(double delta_time) {
	// Calculate all new Temps, skipping the outer borders with continue
	int idx_i = -1;
	for (auto& vec : T.vvCells)
	{
		idx_i++;
		if (idx_i == 0 || idx_i == T.m-1) { continue;}
		int idx_j = -1;

		for (auto& cell : vec)
		{
			idx_j++;
			if (idx_j == 0 || idx_j == T.n-1) { continue; }

			/*							(T_m+1,n - 2 * T_m,n + T_m-1,n		T_m,n+1 - 2 * T_m,n + T_m,n-1)
			 * T^t+1 = delt_t * alpha *	(-----------------------------	+	-----------------------------) + T^t
			 *							(	deltaM²								deltaN²					 )
			 */
			cell.temp = delta_time * diff_con *
				(
					(T.vvCells.at(idx_i+1).at(idx_j).temp - 2.0f * T.vvCells.at(idx_i).at(idx_j).temp + T.vvCells.at(idx_i - 1).at(idx_j).temp)
					+ 
					(T.vvCells.at(idx_i).at(idx_j+1).temp - 2.0f * T.vvCells.at(idx_i).at(idx_j).temp + T.vvCells.at(idx_i).at(idx_j - 1).temp)
				)
				+ cell.temp;
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(double delta_time) {
	// solve A T = b

	// This is just an example to show how to work with the PCG solver,
	int nx = T.m;
	int ny = T.n;
	const int nz = 1;
	int Q = nx * ny * nz;

	SparseMatrix<Real> A(Q);
	std::vector<Real> b;

	// This is the part where you have to assemble the system matrix A and the right-hand side b!
	// Fill b
	for (int i = 0; i < T.m; ++i)
	{
		for (int j = 0; j < T.n; ++j)
		{
			// Catch Boundary Values
			if (i == 0 || j == 0 || i == T.m-1 || j == T.n-1)
			{
				b.push_back(0.0f);
			}
			else
			{
				// Fill b
				b.push_back(T.vvCells.at(i).at(j).temp);
			}
		}
	}

	// Fill A
	// Fill in Multi-Diagonal with Lambdas
	double minus_lambda = -1.0f * diff_con * delta_time;
	double diagonal = 1 + 4 * diff_con * delta_time;
	for (int k = 0; k < Q; ++k)
	{
		if (k < T.n || k % T.n == 0 || k % T.n == T.n - 1 || k >= Q - T.n)
		{
			// Fill in 1s for the diagonals of boundary cells
			A.set_element(k, k, 1.0f);
		}
		else
		{
			// Fill Diagonal Value
			A.set_element(k, k, diagonal);

			// Fill Neighbors where possible
			if (k + 1 < Q) // T_n+1
			{
				A.set_element(k + 1, k, minus_lambda);
			}
			if (k - 1 >= 0) // T_n-1
			{
				A.set_element(k - 1, k, minus_lambda);
			}
			if (k + T.n < Q) // T_m+1
			{
				A.set_element(k, k + T.n, minus_lambda);
			}
			if (k - T.n >= 0) // T_m-1
			{
				A.set_element(k, k - T.n, minus_lambda);
			}
		}
	}

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(Q);
	for (int j = 0; j < Q; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	// Final step is to extract the grid temperatures from the solution vector x
	// to be implemented
	for (int k = 0; k < Q; ++k)
	{
		T.vvCells.at(k / T.n).at(k % T.n).temp = x.at(k);
	}

	T.set_boundaries();
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
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	// Draw every cell as Sphere, change lighting to match temperature
	for (auto& vec : T.vvCells)
	{
		for (auto& cell : vec)
		{
			// RGB
			Vec3 col = Vec3(abs(cell.temp), abs(cell.temp * 0.5f - abs(cell.temp) * 0.5f), cell.temp * 0.5f + abs(cell.temp) * 0.5f);

			DUC->setUpLighting(Vec3(0.0f), Vec3(0.1f), 1.0f, col);
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
