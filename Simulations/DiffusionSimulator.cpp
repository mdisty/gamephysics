#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();

	T = Grid(16, 16, 0.1f);
	// rest to be implemented
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	T.reset(16, 16, 0.5);
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	// to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		reset();

		cout << "Explicit solver!\n";

		T.setValue(8, 8, 1.0f);
		for (const auto& v : T.getGrid()) {
			for (const auto& c : v) {
				cout << c << " ";
			}
			cout << endl;
		}

		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit() {
	float dt = 0.001f;
	float dx = 1.0f;
	float dy = 1.0f;
	float alpha = 0.5f;
	float lamda = alpha * dt / (4.0f * dx * dy);

	Grid newT = Grid(T.getWidth(), T.getHeight(), 0.1f);

	for (size_t i = 1; i < T.getWidth() - 1; ++i) {
		for (size_t j = 1; j < T.getHeight() - 1; ++j) {
			float newValue = T.getValue(i, j) + lamda * (T.getValue(i + 1, j + 1) - T.getValue(i - 1, j + 1) - T.getValue(i + 1, j - 1) + T.getValue(i - 1, j - 1));
			newT.setValue(i, j, newValue);
		}
	}

	T = newT;
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
	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit();

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
	auto& grid = T.getGrid();

	for (size_t x = 0; x < T.getWidth(); ++x) {
		for (size_t y = 0; y < T.getHeight(); ++y) {
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(T.getValue(x, y), 0.0f, 0.0f));
			DUC->drawSphere(Vec3(x, y, 0.0f), Vec3(1.0f, 1.0f, 1.0f));
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

// Grid class

Grid::Grid() : w_{ 0 }, h_{ 0 } {};

Grid::Grid(int32_t w, int32_t h, float value) : w_{ w }, h_{ h } {
	temperaturGrid_.resize(w, std::vector<float>(h, value));
};

std::vector<std::vector<float>>& Grid::getGrid()
{
	return temperaturGrid_;
}

float Grid::getValue(size_t x, size_t y) const
{
	return temperaturGrid_.at(x).at(y);
}

void Grid::setValue(size_t x, size_t y, float v)
{
	temperaturGrid_.at(x).at(y) = v;
}

int32_t Grid::getWidth() const
{
	return w_;
}

int32_t Grid::getHeight() const
{
	return h_;
}

void Grid::reset(int32_t w, int32_t h, float value)
{
	w_ = w;
	h_ = h;
	temperaturGrid_.clear();
	temperaturGrid_.resize(w, std::vector<float>(h, value));
}
