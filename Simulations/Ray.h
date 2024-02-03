#pragma once
#include "Simulator.h"

class Ray
{
public:
	Ray(Vec3 direction, Vec3 origin) : direction(direction), origin(origin) {};
	Vec3 direction;
	Vec3 origin;
	std::pair<double, double> distance(Vec3 sphere);
	Vec3 intersectPlane(Vec3 n);
};
