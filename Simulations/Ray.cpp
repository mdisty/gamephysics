#include "Ray.h"

std::pair<double, double> Ray::distance(Vec3 sphere)
{
	Vec3 s = sphere - origin;
	double shadow = max(dot(s, direction), 0.);
	//Shadow is the distance from our origin until the end of the shadow of our next sphere
	double distance = norm(shadow * direction - sphere + origin);
	//distance from ray to sphere center
	return std::make_pair(distance, shadow);
}

Vec3 Ray::intersectPlane(Vec3 n)
{
	double t = dot(n, -origin) / dot(n, direction); // -origin since it should be z-axis of the plane (in our case it is 0)	
	return origin + t * direction;
}
