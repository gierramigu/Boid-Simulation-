
#pragma once

// glm
#include <glm/glm.hpp>

// project
#include "scene.hpp"


class Boid {
private:

	glm::vec3 m_position;
	glm::vec3 m_velocity;
	glm::vec3 m_acceleration;

	//other flocks
	glm::vec3 o_position;
	glm::vec3 o_velocity;
	glm::vec3 o_acceleration;
	

public:
	Boid(glm::vec3 pos, glm::vec3 dir) : m_position(pos), m_velocity(dir) { }

	glm::vec3 position() const { return m_position; }
	glm::vec3 velocity() const { return m_velocity; }
	glm::vec3 acceleration() const { return m_acceleration; }

	glm::vec3 color() const;

	void calculateForces(Scene *scene);
	void update(float timestep, Scene *scene);

	float distance(glm::vec3 a, glm::vec3 b);
};