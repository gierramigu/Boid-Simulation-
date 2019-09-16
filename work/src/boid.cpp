
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
//#include "scene.hpp"
//#include "scene.cpp"
#include "cgra/cgra_mesh.hpp"

#include <string>
#include <vector>
#include <iostream>


using namespace glm;
using namespace std;


vec3 Boid::color() const {
	return vec3(0, 1, 0);
}


void Boid::calculateForces(Scene *scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Calculate the forces affecting the boid and update the
	// acceleration (assuming mass = 1).
	// Do NOT update velocity or position in this function.
	// Core : 
	//  - Cohesion
	//  - Alignment
	//  - Avoidance
	//  - Soft Bound (optional)
	// Completion : 
	//  - Cohesion and Alignment with only boids in the same flock
	//  - Predator Avoidance (boids only)
	//  - Predator Chase (predator only)
	// Challenge : 
	//  - Obstacle avoidance
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...

		/*Cohesion*/
		//Finds the average location of nearby boids and manipulates the steering force to move in that direction
		//Similar to alignment but instead of the position it would be the velocity
		glm::vec3 sumCohesion = glm::vec3(0, 0, 0);
		float neighbourDist = scene->neighbourSize; //field of neighbour radius
		int countCohesion = 0;
		//Boid b = scene->boids().at(i);

		//iterate through the neighbours
		for (int j = 0; j < scene->boids().size(); j++) {
			Boid b2 = scene->boids().at(j);
			float d = distance((m_position), (b2.m_position));
			//cout << d << endl;
			if ((d>0) && (d < neighbourDist)) {
				sumCohesion += scene->boids().at(j).m_position; //ones in the neighbourhood
				countCohesion++;
			}
		}
		//If there are boids close enough then calculate the forces
		if (countCohesion > 0) {
			glm::vec3 centroid = sumCohesion / (float)countCohesion; 
			//creates a force that goes from the boid's current position to the centroid 
			sumCohesion = scene->m_cohesion * (centroid - m_position); 
		}
	
		/*Alignment*/
		//For every nearby boid in the system, calculate the average velocity
		glm::vec3 sumAlignment = glm::vec3(0, 0, 0);
		int countAlignment = 0;
		for (int j = 0; j < scene ->boids().size(); j++) {
			float d = distance(m_position, (scene->boids().at(j).m_position));
			if ((d > 0) && (d < neighbourDist)) {
				sumAlignment += scene -> boids().at(j).m_velocity;
				countAlignment++;
			}
		}
		if (countAlignment > 0) {
			//divide sum by the number of close boids (average velocity)
			glm::vec3 centroid = sumAlignment / (float)countAlignment; 
			sumAlignment = (scene->m_alignment*(centroid - m_velocity));
		}	

		/*Avoidance*/
		glm::vec3 steerAvoidance = glm::vec3(0, 0, 0);
		int countAvoidance = 0;

		//for each boid in the neighbourhood
		for (int j = 0; j < scene->boids().size(); j++) {
			Boid b2 = scene->boids().at(j);
			float d = distance((m_position), (scene->boids().at(j).m_position));

			//If its a fellow boid and its too close, move away from its current position
			if ((d > 0) && (d < neighbourDist)) {
				float val = 1 / glm::length(m_position-scene->boids().at(j).m_position);
				steerAvoidance += (glm::normalize(m_position - scene->boids().at(j).m_position)*val);
			}
		}
		steerAvoidance *= scene->m_avoidance;

		//Adding it all up with the acceleration
		m_acceleration = (sumCohesion + sumAlignment + steerAvoidance);
}
 

void Boid::update(float timestep, Scene *scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Integrate the velocity of the boid using the timestep.
	// Update the position of the boid using the new velocity.
	// Take into account the bounds of the scene which may
	// require you to change the velocity (if bouncing) or
	// change the position (if wrapping).
	//-------------------------------------------------------------

	// YOUR CODE GOES HERE
	// ...

	//Limiting the max and min speed 
	glm::vec3 velocity = m_velocity + (m_acceleration*0.01f);
	float len = glm::length(velocity);
	if (len > scene->maxSpeed) {
		velocity = (scene->maxSpeed / len)* velocity;
	}

	float lenMin = glm::length(velocity);
	if (lenMin < scene->minSpeed) {
		velocity = (scene->minSpeed / lenMin)* velocity;
	}

	m_velocity = velocity;
	//m_position = m_position + m_velocity*0.01f;
	m_position = m_position + m_velocity * timestep;
	
	//Wrapping
	if (m_position.x < -scene->bound().x) m_position.x = scene->bound().x;
	if (m_position.x > scene->bound().x) m_position.x = -scene->bound().x;
	if (m_position.y < -scene->bound().y) m_position.y = scene->bound().y;
	if (m_position.y > scene->bound().y) m_position.y = -scene->bound().y;
	if (m_position.z < -scene->bound().z) m_position.z = scene->bound().z;
	if (m_position.z > scene->bound().z) m_position.z = -scene->bound().z;

}

//Calculating the distance between two vectors 
float Boid::distance(glm::vec3 a, glm::vec3 b) {
	glm::vec3 d(b - a);
	return length(d);
}
