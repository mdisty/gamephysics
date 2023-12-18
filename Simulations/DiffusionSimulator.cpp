#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();

	T = Grid(16, 16, 0.0f);
	// rest to be implemented
}

const char * DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	T.reset(16, 16, 0.0);
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
	case 0:{
		reset();

		cout << "Explicit solver!\n";
		
		T.resetRandom(16, 16, -1.0f, 1.0f);

		break; 
	}
	case 1:
		reset();

		cout << "Implicit solver!\n";

		T.resetRandom(16, 16, -1.0f, 1.0f);
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit() {
	float dt = 0.01f;
	float dx = 1.0f;
	float dy = 1.0f;
	float alpha = 0.05f;
	//float lamda = alpha * dt / (4.0f * dx * dy);
	//float lamda = alpha * dt * (dy * dy - dx * dx) / (dx * dx * dy * dy);

	Grid newT = Grid(T.getWidth(), T.getHeight(), 0.0f);

	for (size_t i = 1; i < T.getWidth() - 1; ++i) {
		for (size_t j = 1; j < T.getHeight() - 1; ++j) {
			float newValue = T.getValue(i, j) + dt * alpha * ((T.getValue(i + 1, j) - 2.0f * T.getValue(i, j) + T.getValue(i - 1, j)) / (dx * dx) + 
							(T.getValue(i, j + 1) - 2.0f * T.getValue(i, j) + T.getValue(i, j - 1)) / (dy * dy));

			newT.setValue(i, j, newValue);
		}
	}

	T = newT;
}


void DiffusionSimulator::diffuseTemperatureImplicit() {
	// solve A T = b

	int n = (T.getWidth() - 2) * (T.getHeight() - 2) + 2;

	float dt = 0.01f;
	float dx = 1.0f;
	float dy = 1.0f;
	float alpha = 0.05f;
	float lamda = dt * alpha / dx * dx;
	float mu = dt * alpha / dy * dy;
	float a = 1.0f + 2.0f * lamda + 2.0f * mu;

	SparseMatrix<double> result(n);


	for (int i = 1; i < n - 1; ++i) {
		for (int j = 1; j < n - 1; ++j) {
			if (i == j) {
				result.set_element(i, j, a);
			}
			if (i == j - (T.getWidth() - 2)) {
				result.set_element(i, j, -mu);
			}
			if (i == j + (T.getWidth() - 2)) {
				result.set_element(i, j, -lamda);
			}
			if (i == j - 1) {
				if (i % (T.getWidth() - 2) != 0) {
					result.set_element(i, j, -mu);
				}
			}
			if (i == j + 1) {
				if ((i - 1) % (T.getWidth() - 2) != 0) {
					result.set_element(i, j, -lamda);
				}
			}
		}
	}

	result.set_element(0, 0, 1.0);
	result.set_element(n-1, n-1, 1.0);

	/*for (auto& v : result.value) {
		for (auto& i : v) {
			std::cout << i << ", ";
		}
		std::cout << endl;
	}*/

	/*for (int i = 1; i < n - 1; ++i) {
		for (int j = 1; j < n - 1; ++j) {
			cout << result(i, j) << ", ";
		}
		cout << endl;
	}*/

	/*for (const auto& v : T.getGrid()) {
		for (auto& i : v) {
			std::cout << i << ", ";
		}
		std::cout << endl;
	}*/

	//std::vector<Real> b(N);
	std::vector<Real> b = T.toVector();

	// This is the part where you have to assemble the system matrix A and the right-hand side b!

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(n);
	for (int j = 0; j < n; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(result, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	// Final step is to extract the grid temperatures from the solution vector x
	T.insertVector(x);
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		diffuseTemperatureExplicit();
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	//visualization
	auto& grid = T.getGrid();

	double min{ T.getMin() };
	double max{ T.getMax() };

	Vec3 orange{ 235.0f/255.0f, 113.0f/255.0f, 52.0f/255.0f };
	Vec3 black{ 0.0f, 0.0f, 0.0f };
	Vec3 white{ 1.0f, 1.0f, 1.0f };

	for (size_t x = 0; x < T.getWidth(); ++x) {
		for (size_t y = 0; y < T.getHeight(); ++y) {

			double temp{ T.getValue(x, y) };
			Vec3 color;

			if (temp < 0.0f) {
				color = temp / min * white;
			}else if (temp > 0.0f) {
				color = temp / max * orange;
			}
			else {
				color = black;
			}

			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);
			DUC->drawSphere(Vec3(x, T.getHeight() - y, 0.0f), Vec3(1.0f, 1.0f, 1.0f));
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

Grid::Grid(int32_t w, int32_t h, double value) : w_{ w }, h_{ h } {
	temperaturGrid_.resize(w, std::vector<double>(h, value));
};

std::vector<std::vector<double>>& Grid::getGrid()
{
	return temperaturGrid_;
}

double Grid::getValue(size_t x, size_t y) const
{
	return temperaturGrid_.at(x).at(y);
}

void Grid::setValue(size_t x, size_t y, double v)
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

double Grid::getMin() const
{
	if (w_ == 0 || h_ == 0) return 0.0f;

	double min = temperaturGrid_.at(0).at(0);
	std::for_each(temperaturGrid_.begin(), temperaturGrid_.end(), [&min](const auto& v) { min = std::min(*std::min_element(v.begin(), v.end()), min); });
	return min;
}

double Grid::getMax() const
{
	if (w_ == 0 || h_ == 0) return 0.0f;

	double max = temperaturGrid_.at(0).at(0);
	std::for_each(temperaturGrid_.begin(), temperaturGrid_.end(), [&max](const auto& v) { max = std::max(*std::max_element(v.begin(), v.end()), max); });
	return max;
}

void Grid::reset(int32_t w, int32_t h, double value)
{
	w_ = w;
	h_ = h;
	temperaturGrid_.clear();
	temperaturGrid_.resize(w, std::vector<double>(h, value));
}

void Grid::resetRandom(int32_t w, int32_t h, double min, double max) {
	std::mt19937 eng(time(nullptr));
	std::uniform_real_distribution<double> randVal(min, max);

	this->reset(w, h, 0.0f);

	for (size_t i = 1; i < w - 1; ++i) {
		for (size_t j = 1; j < h - 1; ++j) {
			temperaturGrid_.at(i).at(j) = randVal(eng);
		}
	}
}

std::vector<double> Grid::toVector() const
{
	/*std::vector<double> v;

	for (size_t j = 0; j < w_; ++j) {
		for (size_t i = 0; i < h_; ++i) {
			v.emplace_back(temperaturGrid_.at(i).at(j));
		}
	}

	return v;*/
	std::vector<double> v;

	v.emplace_back(0.0);

	for (size_t j = 1; j < w_ - 1; ++j) {
		for (size_t i = 1; i < h_ - 1; ++i) {
			v.emplace_back(temperaturGrid_.at(i).at(j));
		}
	}

	v.emplace_back(0.0);

	return v;
}

void Grid::insertVector(const std::vector<double>& v) {
	/*for (size_t j = 0; j < w_; ++j) {
		for (size_t i = 0; i < h_; ++i) {
			temperaturGrid_.at(i).at(j) = v.at( j * h_ + i);
		}
	}*/
	for (size_t j = 1; j < w_-1; ++j) {
		for (size_t i = 1; i < h_-1; ++i) {
			temperaturGrid_.at(i).at(j) = v.at((j - 1) * (h_-2) + i);
		}
	}
}
