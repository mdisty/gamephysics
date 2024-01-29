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
	bool selected { false };
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

	Diffusion& getDiffusion();

	/*
	* @return index of the newly added MassPoint or -1 if not successful
	*/
	int insertSpringMassPoint(int massPointIndex, Vec3 newMassPointPos, float temperatur, bool fixed);

	/*
	* Calculates one midstep step with the given timeStep.
	* Saves everything in the massPoints and springs vectors.
	*/
	void calculateMidpointStep(float timeStep);

	/*
	* Calculates the force for a specific point and a spring with Hookslaw.
	* @return The calculated force as a Vec3.
	*/
	Vec3 calculateForce(Spring s, int pointIndex);

	/*
	* Draws all the springs with the corresponding temperature colors
	*/
	void drawSprings(DrawingUtilitiesClass* DUC, Vec3 hot, Vec3 zero, Vec3 cold);
private:
	/*
	* Adds a new MassPoint in worldspace and into the diffusion grid
	* @return Index of the new MassPoint in the massPoints_ vector
	*/
	int addMassPoint(Vec3 position, Vec3 velocity, std::array<size_t, 2> gridPosition, float temperatur, bool isFixed);
	void addSpring(int massPoint1, int massPoint2, float length);

	/*
	* Adds all the missing springs to a MassPoint depending on the diffusion grid. If the MassPoint has 
	* a neighbour in the grid that it isn't connected with a spring will be added.
	*/
	void addMissingSpringsToMassPoint(int massPointIndex, float springLength);
	/*
	* @return Index of the MassPoint in the massPoints_ vector
	*/
	int findMassPointByGridPosition(size_t x, size_t y);

	std::tuple<size_t, size_t> calculateGridPositionForNewMassPoint(MassPoint& other, Vec3& newMassPointPos) const;

	std::vector<Spring> springs_;
	std::vector<MassPoint> massPoints_;
	
	Diffusion diffusion_;

	float mass_;
	float stiffness_;
	float damping_;
};