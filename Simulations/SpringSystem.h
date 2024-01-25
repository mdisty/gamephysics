#pragma once

#include "Simulator.h"
#include "DiffusionGrid.h"

#include <variant>

class MassPoint {
public:
	MassPoint(Vec3 position = Vec3(0.0f), Vec3 veloctiy = Vec3(0.0f), std::array<size_t, 2> gridPosition = {0, 0}, bool isFixed = false);

	Vec3 position;
	Vec3 veloctiy;
	std::array<size_t, 2> gridPosition;
	bool isFixed;
};

class Spring {
public:
	Spring(int32_t masspoint1, int32_t masspoint2, float length);

	int32_t masspoint1;
	int32_t masspoint2;
	float length;
};

class SpringSystem {
public:
	SpringSystem(float mass = 10.0f, float stiffness = 80.0f, float damping = 1.0f, Vec3 startMassPoint = Vec3(0.0f), float temperatur = 1.0f);

	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDamping(float damping);

	/*
	@return index of the newly added MassPoint or -1 if not successful
	*/
	int insertSpringMassPoint(int massPointIndex, Vec3 newMassPointPos, float initialLength, float temperatur);

	/*
	Calculates one midstep step with the given timeStep.
	Saves everything in the massPoints and springs vectors.
	*/
	void calculateMidpointStep(float timeStep, float alpha);

	/*
	Calculates the force for a specific point and a spring with Hookslaw.
	\return The calculated force as a Vec3.
	*/
	Vec3 calculateForce(Spring s, int pointIndex);

	void drawSprings(DrawingUtilitiesClass* DUC) const;
//private:
	int addMassPoint(Vec3 position, Vec3 velocity, std::array<size_t, 2> gridPosition, float temperatur, bool isFixed);
	void addSpring(int massPoint1, int massPoint2, float length);

	void addMissingSpringsToMassPoint(int massPointIndex, float springLength);
	int findMassPointByGridPosition(size_t x, size_t y);

	std::vector<Spring> springs_;
	std::vector<MassPoint> massPoints_;
	
	Diffusion diffusion_;

	float mass_;
	float stiffness_;
	float damping_;
};