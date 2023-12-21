#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator() : N{ 16 }, M{ 16 }, N_old{ 16 }, M_old{ 16 }, Alpha{ 0.05f }
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();

	T = Grid(16, 16, 0.0f);
	hotColor = { 252.0f / 255.0f, 3.0f / 255.0f, 80.0f / 255.0f };
	coldColor = { 0.0f / 255.0f, 234.0f / 255.0f, 255.0f / 255.0f };
	zeroColor = { 10.0f / 255.0f, 0.0f / 255.0f, 30.0f / 255.0f };
}

const char * DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	T.reset(N, M, 0.0);
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "N", TW_TYPE_INT32, &N, "step=1 min=3");
	TwAddVarRW(DUC->g_pTweakBar, "M", TW_TYPE_INT32, &M, "step=1 min=3");
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &Alpha, "step=0.001 min=0.001");
	TwAddVarRW(DUC->g_pTweakBar, "Hot Color", TW_TYPE_COLOR3F, &hotColor, "colormode=rgb");
	TwAddVarRW(DUC->g_pTweakBar, "Cold Color", TW_TYPE_COLOR3F, &coldColor, "colormode=rgb");
	TwAddVarRW(DUC->g_pTweakBar, "Zero Color", TW_TYPE_COLOR3F, &zeroColor, "colormode=rgb");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);

	switch (m_iTestCase)
	{
	case 0:{
		reset();

		cout << "Explicit solver!\n";
		
		T.resetRandom(N, M, -1.0f, 1.0f);

		break; 
	}
	case 1:
		reset();

		cout << "Implicit solver!\n";

		T.resetRandom(N, M, -1.0f, 1.0f);

		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(const float dt) {
	float dx = 1.0f;
	float dy = 1.0f;

	Grid newT = Grid(T.getWidth(), T.getHeight(), 0.0f);

	for (size_t i = 1; i < T.getWidth() - 1; ++i) {
		for (size_t j = 1; j < T.getHeight() - 1; ++j) {
			float newValue = T.getValue(i, j) + dt * Alpha * ((T.getValue(i + 1, j) - 2.0f * T.getValue(i, j) + T.getValue(i - 1, j)) / (dx * dx) +
							(T.getValue(i, j + 1) - 2.0f * T.getValue(i, j) + T.getValue(i, j - 1)) / (dx * dx));

			newT.setValue(i, j, newValue);
		}
	}

	T = newT;
}

void DiffusionSimulator::diffuseTemperatureImplicit(const float dt) {
	// solve A T = b

	int n = T.getWidth() * T.getHeight();

	float dx = 1.0f;
	float dy = 1.0f;
	float lamda = dt * Alpha / (dx * dx);
	float mu = dt * Alpha / (dy * dy);
	float a = 1.0f + 2.0f * lamda + 2.0f * mu;

	SparseMatrix<double> result(n);

	for (int i = 0; i < n; ++i) {
		if (i % T.getHeight() == (T.getHeight() - 1) || i % T.getHeight() == 0 || i < T.getHeight() || i >= n - T.getHeight()) {
			result.set_element(i, i, 1.0);
		}
		else {
			result.set_element(i, i, a);
			result.set_element(i - 1, i, -lamda);
			result.set_element(i + 1, i, -mu);
			result.set_element(i + T.getHeight(), i, -mu);
			result.set_element(i - T.getHeight(), i, -lamda);
		}
	}

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
	T.setBoundaryToZero();
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
	if (N_old != N || M_old != M) {
		notifyCaseChanged(m_iTestCase);
		N_old = N;
		M_old = M;
		return;
	}

	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	//visualization
	auto& grid = T.getGrid();

	double min{ T.getMin() };
	double max{ T.getMax() };

	Vec3 hot{ hotColor.at(0), hotColor.at(1), hotColor.at(2) };
	Vec3 black{ zeroColor.at(0), zeroColor.at(1), zeroColor.at(2) };
	Vec3 cold{ coldColor.at(0), coldColor.at(1), coldColor.at(2) };

	for (size_t x = 0; x < T.getWidth(); ++x) {
		for (size_t y = 0; y < T.getHeight(); ++y) {

			double temp{ T.getValue(x, y) };
			Vec3 color;

			if (temp < 0.0f) {
				if (std::abs(min) > 1.0f) temp = temp / min;
				color = std::abs(temp) * cold + (1.0 - std::abs(temp)) * black;
			} else if (temp > 0.0f) {
				if (std::abs(max) > 1.0f) temp = temp / max;
				color = std::abs(temp) * hot + (1.0 - std::abs(temp)) * black;
			} else {
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
	if (w < 0 || h < 0) throw std::runtime_error("The width/height can not be negative!");
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
	if (w < 0 || h < 0) throw std::runtime_error("The width/height can not be negative!");
	w_ = w;
	h_ = h;
	temperaturGrid_.clear();
	temperaturGrid_.resize(w, std::vector<double>(h, value));
}

void Grid::resetRandom(int32_t w, int32_t h, double min, double max) {
	if (w < 0 || h < 0) throw std::runtime_error("The width/height can not be negative!");
	if (min > max) throw std::runtime_error("The minimum is bigger than the maximum value!");
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
	std::vector<double> v{};
	v.resize(w_ * h_, 0.0);

	for (size_t i = 0; i < w_; ++i) {
		for (size_t j = 0; j < h_; ++j) {
			v.at(i * h_ + j) = temperaturGrid_.at(i).at(j);
		}
	}
	
	return v;
}

void Grid::insertVector(const std::vector<double>& v) {
	if (v.size() != w_ * h_) throw std::runtime_error("Vector size invalid: size should be grid.width * grid.height!");
	for (size_t i = 0; i < w_; ++i) {
		for (size_t j = 0; j < h_; ++j) {
			temperaturGrid_.at(i).at(j) = v.at(i * h_ + j);
		}
	}
}

void Grid::setBoundaryToZero()
{
	temperaturGrid_.at(0) = std::vector<double>(h_, 0.0);
	temperaturGrid_.at(w_ - 1) = std::vector<double>(h_, 0.0);

	for (size_t i = 1; i < w_ - 1; ++i) {
		temperaturGrid_.at(i).at(0) = 0.0;
		temperaturGrid_.at(i).at(h_ - 1) = 0.0;
	}
}
