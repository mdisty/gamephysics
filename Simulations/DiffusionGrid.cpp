#include "DiffusionGrid.h"

// DiffusionGrid class

DiffusionGrid::DiffusionGrid() : w_{ 0 }, h_{ 0 } {};

DiffusionGrid::DiffusionGrid(int32_t w, int32_t h) : w_{ w }, h_{ h } {
	if (w < 0 || h < 0) throw std::runtime_error("The width/height can not be negative!");
	temperaturGrid_.resize(w, std::deque<double>(h, 0.0));
	mask_.resize(w, std::deque<bool>(h, false));
}

double DiffusionGrid::getValue(size_t x, size_t y) const
{
	return temperaturGrid_.at(x).at(y);
}

void DiffusionGrid::setValue(size_t x, size_t y, double v)
{
	temperaturGrid_.at(x).at(y) = v;
	mask_.at(x).at(y) = true;
}

void DiffusionGrid::resetValue(size_t x, size_t y)
{
	temperaturGrid_.at(x).at(y) = 0.0;
	mask_.at(x).at(y) = false;
}

bool DiffusionGrid::insertNeighbour(size_t x, size_t y, double v)
{
	if (!mask_.at(x).at(y)) {
		if (isOnBoundary(x, y)) {
			growGrid(1);
			x += 1;
			y += 1;
		}

		temperaturGrid_.at(x).at(y) = v;
		mask_.at(x).at(y) = true;

		return true;
	}

	return false;
}

int32_t DiffusionGrid::getWidth() const
{
	return w_;
}

int32_t DiffusionGrid::getHeight() const
{
	return h_;
}

std::deque<std::deque<bool>>& DiffusionGrid::getMask()
{
	return mask_;
}

void DiffusionGrid::setTemperaturGrid(std::deque<std::deque<double>>&& newGrid)
{
	temperaturGrid_ = std::move(newGrid);
}

double DiffusionGrid::getMin() const
{
	if (w_ == 0 || h_ == 0) return 0.0f;

	double min = temperaturGrid_.at(0).at(0);
	std::for_each(temperaturGrid_.begin(), temperaturGrid_.end(), [&min](const auto& v) { min = std::min(*std::min_element(v.begin(), v.end()), min); });
	return min;
}

double DiffusionGrid::getMax() const
{
	if (w_ == 0 || h_ == 0) return 0.0f;

	double max = temperaturGrid_.at(0).at(0);
	std::for_each(temperaturGrid_.begin(), temperaturGrid_.end(), [&max](const auto& v) { max = std::max(*std::max_element(v.begin(), v.end()), max); });
	return max;
}

void DiffusionGrid::reset(int32_t w, int32_t h, double value)
{
	if (w < 0 || h < 0) throw std::runtime_error("The width/height can not be negative!");
	w_ = w;
	h_ = h;
	temperaturGrid_.clear();
	temperaturGrid_.resize(w, std::deque<double>(h, value));
}

void DiffusionGrid::resetRandom(int32_t w, int32_t h, double min, double max) {
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

void DiffusionGrid::growGrid(size_t factor)
{
	h_ += 2 * factor;
	w_ += 2 * factor;
	temperaturGrid_.push_front(std::deque<double>(h_, 0.0));
	temperaturGrid_.push_back(std::deque<double>(h_, 0.0));
	mask_.push_front(std::deque<bool>(h_, false));
	mask_.push_back(std::deque<bool>(h_, false));

	for (auto& column : temperaturGrid_) {
		for (size_t i = 0; i < factor; ++i) {
			column.push_front(0.0);
			column.push_back(0.0);
		}
	}

	for (auto& column : mask_) {
		for (size_t i = 0; i < factor; ++i) {
			column.push_front(false);
			column.push_back(false);
		}
	}
}

std::vector<double> DiffusionGrid::toVector() const
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

void DiffusionGrid::insertVector(const std::vector<double>& v) {
	if (v.size() != w_ * h_) throw std::runtime_error("Vector size invalid: size should be grid.width * grid.height!");
	for (size_t i = 0; i < w_; ++i) {
		for (size_t j = 0; j < h_; ++j) {
			temperaturGrid_.at(i).at(j) = v.at(i * h_ + j);
		}
	}
}

void DiffusionGrid::setBoundaryToZero()
{
	temperaturGrid_.at(0) = std::deque<double>(h_, 0.0);
	temperaturGrid_.at(w_ - 1) = std::deque<double>(h_, 0.0);

	for (size_t i = 1; i < w_ - 1; ++i) {
		temperaturGrid_.at(i).at(0) = 0.0;
		temperaturGrid_.at(i).at(h_ - 1) = 0.0;
	}
}

bool DiffusionGrid::isOnBoundary(size_t x, size_t y)
{
	if (x == 0 || y == 0) return true;
	if (x == w_ - 1 || y == h_ - 1) return true;
	return false;
}

void DiffusionGrid::applyMask()
{
	for (size_t i = 0; i < w_; ++i) {
		for (size_t u = 0; u < h_; ++u) {
			if (!mask_.at(i).at(u)) {
				temperaturGrid_.at(i).at(u) = 0.0;
			}
		}
	}
}

// Diffusion class

Diffusion::Diffusion() : diffusionGrid_(5, 5) {}

Diffusion::Diffusion(DiffusionGrid grid) : diffusionGrid_{grid} {}

DiffusionGrid& Diffusion::getDiffusionGrid()
{
	return diffusionGrid_;
}

void Diffusion::diffuseTemperatureExplicit(const float dt, const float alpha)
{
	float dx = 1.0f;
	float dy = 1.0f;

	std::deque<std::deque<double>> newGrid;
	newGrid.resize(diffusionGrid_.getWidth(), std::deque<double>(diffusionGrid_.getHeight(), 0.0));

	for (size_t i = 1; i < diffusionGrid_.getWidth() - 1; ++i) {
		for (size_t j = 1; j < diffusionGrid_.getHeight() - 1; ++j) {
			float newValue = diffusionGrid_.getValue(i, j) + dt * alpha * ((diffusionGrid_.getValue(i + 1, j) - 2.0f * diffusionGrid_.getValue(i, j) + diffusionGrid_.getValue(i - 1, j)) / (dx * dx) +
				(diffusionGrid_.getValue(i, j + 1) - 2.0f * diffusionGrid_.getValue(i, j) + diffusionGrid_.getValue(i, j - 1)) / (dx * dx));

			newGrid.at(i).at(j) = newValue;
		}
	}

	diffusionGrid_.setTemperaturGrid(std::move(newGrid));

	diffusionGrid_.applyMask();
}

void Diffusion::diffuseTemperatureImplicit(const float dt, const float alpha)
{
	// solve A T = b

	int n = diffusionGrid_.getWidth() * diffusionGrid_.getHeight();

	float dx = 1.0f;
	float dy = 1.0f;
	float lamda = dt * alpha / (dx * dx);
	float mu = dt * alpha / (dy * dy);
	float a = 1.0f + 2.0f * lamda + 2.0f * mu;

	SparseMatrix<double> result(n);

	for (int i = 0; i < n; ++i) {
		if (i % diffusionGrid_.getHeight() == (diffusionGrid_.getHeight() - 1) || 
			i % diffusionGrid_.getHeight() == 0 || 
			i < diffusionGrid_.getHeight() || 
			i >= n - diffusionGrid_.getHeight()) {
			result.set_element(i, i, 1.0);
		}
		else {
			result.set_element(i, i, a);
			result.set_element(i - 1, i, -lamda);
			result.set_element(i + 1, i, -mu);
			result.set_element(i + diffusionGrid_.getHeight(), i, -mu);
			result.set_element(i - diffusionGrid_.getHeight(), i, -lamda);
		}
	}

	std::vector<double> b = diffusionGrid_.toVector();

	// This is the part where you have to assemble the system matrix A and the right-hand side b!

	// perform solve
	double pcg_target_residual = 1e-05;
	double pcg_max_iterations = 1000;
	double ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<double> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<double> x(n);
	for (int j = 0; j < n; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(result, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	// Final step is to extract the grid temperatures from the solution vector x
	diffusionGrid_.insertVector(x);
	diffusionGrid_.applyMask();
}
