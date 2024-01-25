#include "SpringSystem.h"

// MassPoint class

MassPoint::MassPoint(Vec3 position, Vec3 veloctiy, std::array<size_t, 2> gridPosition, bool isFixed) :
	position{ position },
	veloctiy{ veloctiy },
	gridPosition{ gridPosition },
	isFixed{ isFixed } {}

// Spring class

Spring::Spring(int32_t masspoint1, int32_t masspoint2, float length) : 
	masspoint1{masspoint1}, 
	masspoint2{ masspoint2 }, 
	length{ length } {}

// SpringSystem class 

SpringSystem::SpringSystem(float mass, float stiffness, float damping, Vec3 startMassPoint, float temperatur) : 
	mass_{mass}, 
	stiffness_{stiffness}, 
	damping_{damping},
	diffusion_()
{
	addMassPoint(startMassPoint, Vec3(0.0f), {2, 2}, temperatur, false);
}

void SpringSystem::setMass(float mass)
{
	mass_ = mass;
}

void SpringSystem::setStiffness(float stiffness)
{
	stiffness_ = stiffness;
}

void SpringSystem::setDamping(float damping)
{
	damping_ = damping;
}

int SpringSystem::insertSpringMassPoint(int massPointIndex, Vec3 newMassPointPos, float initialLength, float temperatur)
{
	MassPoint other{ massPoints_.at(massPointIndex) };
	int32_t currentGridSize{ diffusion_.getDiffusionGrid().getHeight() };
	
	// Calculate Grid Position TODO: Put in seperate function

	Vec3 up{ 0.0f, 1.0f, 0.0f };
	Vec3 newPosRelative = newMassPointPos - other.position;

	double theta = std::acos(dot(up, newPosRelative) / (norm(up) * norm(newPosRelative)));
	theta = newPosRelative.x < 0.0 ? -theta : theta;

	size_t x = other.gridPosition.at(0);
	size_t y = other.gridPosition.at(1);

	if (theta <= 0.785398 && theta >= -0.785398) { // Oben
		y += 1;
	} 
	else if (theta >= 0.785398 && theta <= 2.35619) { // Rechts
		x += 1;
	} 
	else if (theta >= 2.35619 || theta <= -2.35619) { // Unten
		y -= 1;
	}  
	else if (theta <= -0.785398 && theta >= -2.35619) { // Links
		x -= 1;
	}

	bool newNeighbourPosition = diffusion_.getDiffusionGrid().insertNeighbour(x, y, static_cast<double>(temperatur));
	
	int32_t newGridSize{ diffusion_.getDiffusionGrid().getHeight() };

	if (currentGridSize < newGridSize) {
		for (auto& point : massPoints_) {
			point.gridPosition.at(0) += (newGridSize - currentGridSize) / 2;
			point.gridPosition.at(1) += (newGridSize - currentGridSize) / 2;
		}
		x += (newGridSize - currentGridSize) / 2;
		y += (newGridSize - currentGridSize) / 2;
	}

	if (newNeighbourPosition) {
		// Add springs with neighbours
		int result = addMassPoint(newMassPointPos, Vec3(0.0f), {x, y}, temperatur, false);

		addMissingSpringsToMassPoint(result, initialLength);

		return result;
	}

	std::cerr << "ERROR: SpringMassPoint could not be inserted!" << std::endl;
	return -1; // TODO: Better error handling!
}

void SpringSystem::calculateMidpointStep(float timeStep, float alpha)
{
	diffusion_.diffuseTemperatureExplicit(timeStep, alpha);

	vector<Vec3> newPositions;
	vector<Vec3> newVelocities;
	vector<MassPoint> oldMassPoints(massPoints_.size());

	for (size_t i = 0; i < massPoints_.size(); ++i) {
		MassPoint currentPoint = massPoints_.at(i);
		oldMassPoints.at(i) = currentPoint;

		if (currentPoint.isFixed) {
			newVelocities.emplace_back(currentPoint.veloctiy);
			newPositions.emplace_back(currentPoint.position);
			continue;
		}

		vector<Spring> pSprings;

		// Calculate midpoint velocity
		for (Spring s : springs_) {
			if (s.masspoint1 == i) {
				pSprings.emplace_back(s);
			}
			else if (s.masspoint2 == i) {
				pSprings.emplace_back(s);
			}
		}

		Vec3 force;

		for (Spring s : pSprings) {
			force += calculateForce(s, i);
		}

		// EXTERNAL FORCES CALCULATION
		/*force += m_externalForce;
		if (i == selectedMassPoint) {
			force += individualExternalForce;
		}*/

		Vec3 acceleration = force / mass_;
		Vec3 pVelocityMidstep = currentPoint.veloctiy + 0.5 * timeStep * acceleration;

		// Calculate midpoint position
		Vec3 pPositionMidstep = currentPoint.position + 0.5 * timeStep * currentPoint.veloctiy;

		newVelocities.emplace_back(pVelocityMidstep);
		newPositions.emplace_back(pPositionMidstep);
	}

	// Set midstep values
	for (size_t i = 0; i < massPoints_.size(); ++i) {
		massPoints_.at(i).veloctiy = newVelocities.at(i);
		massPoints_.at(i).position = newPositions.at(i);
	}

	newVelocities.clear();
	newPositions.clear();

	for (size_t i = 0; i < massPoints_.size(); ++i) {
		MassPoint currentPoint = massPoints_.at(i);

		if (currentPoint.isFixed) {
			newVelocities.emplace_back(currentPoint.veloctiy);
			newPositions.emplace_back(currentPoint.position);
			continue;
		}

		vector<Spring> pSprings;

		// Calculate new velocity
		for (Spring s : springs_) {
			if (s.masspoint1 == i) {
				pSprings.emplace_back(s);
			}
			else if (s.masspoint2 == i) {
				pSprings.emplace_back(s);
			}
		}

		Vec3 force;

		for (Spring s : pSprings) {
			force += calculateForce(s, i);
		}

		// EXTERNAL FORCES CALCULATION
		/*force += m_externalForce;
		if (i == selectedMassPoint) {
			force += individualExternalForce;
		}*/

		Vec3 accelerationMidstep = (force - damping_ * oldMassPoints.at(i).veloctiy) / mass_;
		Vec3 pVelocityNext = oldMassPoints.at(i).veloctiy + timeStep * accelerationMidstep;

		// Calculate new position
		Vec3 pPositionNext = oldMassPoints.at(i).position + timeStep * currentPoint.veloctiy;

		newVelocities.emplace_back(pVelocityNext);
		newPositions.emplace_back(pPositionNext);
	}

	// Set new values
	for (size_t i = 0; i < massPoints_.size(); ++i) {
		massPoints_.at(i).position = newPositions.at(i);
		massPoints_.at(i).veloctiy = newVelocities.at(i);
	}
}

Vec3 SpringSystem::calculateForce(Spring s, int pointIndex)
{
	Vec3 d;
	if (s.masspoint1 == pointIndex) {
		d = massPoints_.at(s.masspoint1).position - massPoints_.at(s.masspoint2).position;
	}
	else {
		d = massPoints_.at(s.masspoint2).position - massPoints_.at(s.masspoint1).position;
	}

	float length = normalize(d);

	double temperaturP1 = diffusion_.getDiffusionGrid().getValue(massPoints_.at(s.masspoint1).gridPosition.at(0), massPoints_.at(s.masspoint1).gridPosition.at(1));
	double temperaturP2 = diffusion_.getDiffusionGrid().getValue(massPoints_.at(s.masspoint2).gridPosition.at(0), massPoints_.at(s.masspoint2).gridPosition.at(1));

	double average = (temperaturP1 + temperaturP2) / 2.0;

	return -stiffness_ * (length - (s.length + average)) * d;
}

void SpringSystem::drawSprings(DrawingUtilitiesClass* DUC) const
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	for (int i = 0; i < massPoints_.size(); ++i) {
		DUC->setUpLighting(Vec3(),
			0.4 * Vec3(1, 1, 1),
			100, Vec3(1.5 * randCol(eng),
				0.6 * randCol(eng), 1.5 * randCol(eng)));
		DUC->drawSphere(massPoints_.at(i).position, 0.1f);
	}

	for (int i = 0; i < springs_.size(); ++i) {
		DUC->beginLine();
		DUC->drawLine(massPoints_.at(springs_.at(i).masspoint1).position,
			Vec3(1.5 * randCol(eng), 0.6 * randCol(eng), 1.5 * randCol(eng)),
			massPoints_.at(springs_.at(i).masspoint2).position,
			Vec3(1.5 * randCol(eng), 0.6 * randCol(eng), 1.5 * randCol(eng)));
		DUC->endLine();
	}
}

int SpringSystem::addMassPoint(Vec3 position, Vec3 velocity, std::array<size_t, 2> gridPosition, float temperatur, bool isFixed)
{
	MassPoint massPoint( position, velocity, gridPosition, isFixed );
	massPoints_.emplace_back(massPoint);
	diffusion_.getDiffusionGrid().setValue(gridPosition.at(0), gridPosition.at(1), temperatur);
	return massPoints_.size() - 1;
}

void SpringSystem::addSpring(int massPoint1, int massPoint2, float length)
{
	Spring spring{ massPoint1, massPoint2, length };
	springs_.emplace_back(spring);
}

void SpringSystem::addMissingSpringsToMassPoint(int massPointIndex, float springLength)
{
	MassPoint massPoint = massPoints_.at(massPointIndex);
	size_t x = massPoint.gridPosition.at(0);
	size_t y = massPoint.gridPosition.at(1);

	std::cerr << "DEBUG: MassPoint Position: x: " << x << " y: " << y << std::endl;

	std::array<size_t, 2> slot1{ x + 1, y };
	std::array<size_t, 2> slot2{ x, y + 1 };
	std::array<size_t, 2> slot3{ x - 1, y };
	std::array<size_t, 2> slot4{ x, y - 1 };

	auto mask = diffusion_.getDiffusionGrid().getMask();

	try {
		if (mask.at(slot1.at(0)).at(slot1.at(1))) {
			addSpring(massPointIndex, findMassPointByGridPosition(slot1.at(0), slot1.at(1)), springLength);
		} 
		if (mask.at(slot2.at(0)).at(slot2.at(1))) {
			addSpring(massPointIndex, findMassPointByGridPosition(slot2.at(0), slot2.at(1)), springLength);
		} 
		if (mask.at(slot3.at(0)).at(slot3.at(1))) {
			addSpring(massPointIndex, findMassPointByGridPosition(slot3.at(0), slot3.at(1)), springLength);
		} 
		if (mask.at(slot4.at(0)).at(slot4.at(1))) {
			addSpring(massPointIndex, findMassPointByGridPosition(slot4.at(0), slot4.at(1)), springLength);
		}
	}
	catch (std::exception e) {
		std::cerr << e.what() << std::endl;
	}

	//std::cerr << "WARNING: No new spring was added!" << std::endl;
}

int SpringSystem::findMassPointByGridPosition(size_t x, size_t y)
{
	for (int i = 0; i < massPoints_.size(); ++i) {
		if (massPoints_.at(i).gridPosition.at(0) == x && massPoints_.at(i).gridPosition.at(1) == y) {
			return i;
		}
	}
	throw std::runtime_error("ERROR: MassPoint could not be found!");
}
