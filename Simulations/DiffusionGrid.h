#pragma once

#include <array>
#include <algorithm>
#include <stdexcept>
#include <random>
#include <deque>
#include <tuple>
#include <iostream>

class DiffusionGrid {
public:
	DiffusionGrid();
	DiffusionGrid(int32_t w, int32_t h);

	double getValue(size_t x, size_t y) const;
	void setValue(size_t x, size_t y, double v);
	void resetValue(size_t x, size_t y);

	bool insertNeighbour(size_t x, size_t y, double value);

	int32_t getWidth() const;
	int32_t getHeight() const;

	std::deque<std::deque<bool>>& getMask();

	/*
	Set the current temeratur grid to newGrid. 
	The mask, width and height of this object have to match the new grid! They won't
	be updated.
	*/
	void setTemperaturGrid(std::deque<std::deque<double>>&& newGrid);

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
	void reset(int32_t w, int32_t h, double v);
	/*
	Resets the grid with a new width, height and a random default value in the range [min, max)
	*/
	void resetRandom(int32_t w, int32_t h, double min, double max);

	void growGrid(size_t factor = 1);

	/*
	@return: The whole matrix as a vector with the size = w * h (colums stacked)
	*/
	std::vector<double> toVector() const;
	/*
	Inserts a whole vector with the size = w * h into the matrix (colums stacked)
	*/
	void insertVector(const std::vector<double>& v);

	void setBoundaryToZero();

	bool isOnBoundary(size_t x, size_t y);

	void applyMask();
//private:
	//std::vector<std::vector<double>> temperaturGrid_{};
	//std::vector<std::vector<bool>> mask_{};

	std::deque<std::deque<double>> temperaturGrid_{};
	std::deque<std::deque<bool>> mask_{};

	int32_t w_;
	int32_t h_;
};

class Diffusion {
public:
	Diffusion();
	Diffusion(DiffusionGrid grid);

	DiffusionGrid& getDiffusionGrid();

	void diffuseTemperatureExplicit(const float dt, const float alpha);
//private:
	DiffusionGrid diffusionGrid_;
};