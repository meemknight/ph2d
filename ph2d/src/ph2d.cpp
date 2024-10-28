//////////////////////////////////////////////////
//ph2d.h				0.0.1
//Copyright(c) 2024 Luta Vlad - Low Level Game Dev
// please credit me if you want, 
// and keep this in the code
//
//////////////////////////////////////////////////



#include <ph2d/ph2d.h>
#include <iostream>

constexpr static float MAX_VELOCITY = 10'000;
constexpr static float MAX_ACCELERATION = 10'000;
constexpr static float MAX_AIR_DRAG = 100;

void normalizeSafe(glm::vec2 &v)
{
	float l = glm::length(v);

	if (l <= 0.00000001)
	{
		v = {1,0};
	}
	else
	{
		v /= l;
	}
}

namespace ph2d
{



	bool AABBvsAABB(AABB a, AABB b, float delta)
	{
		glm::vec2 aMax = a.max() + glm::vec2(delta, delta);
		glm::vec2 bMax = b.max() + glm::vec2(delta, delta);

		glm::vec2 aMin = a.min() - glm::vec2(delta, delta);
		glm::vec2 bMin = b.min() - glm::vec2(delta, delta);

		if (aMax.x < bMin.x || aMin.x > bMax.x) return false;
		if (aMax.y < bMin.y || aMin.y > bMax.y) return false;

		return true;
	}

	bool AABBvsPoint(AABB a, glm::vec2 b, float delta)
	{
		glm::vec2 aMin = a.min() - glm::vec2(delta, delta);
		glm::vec2 aMax = a.max() + glm::vec2(delta, delta);

		if (
			aMin.x < b.x && aMax.x > b.x &&
			aMin.y < b.y && aMax.y > b.y)
		{
			return true;
		}

		return false;
	}


	bool CirclevsCircle(Circle a, Circle b, float delta)
	{
		float r = a.r + b.r + delta;
		r *= r;
		return r > ((a.center.x - b.center.x) * (a.center.x - b.center.x)
			+ (a.center.y - b.center.y) * (a.center.y - b.center.y));
	}

	void applyDrag(MotionState &motionState)
	{
		glm::vec2 dragForce = 0.1f * -motionState.velocity * glm::abs(motionState.velocity) / 2.f;
		float length = glm::length(dragForce);
		if (length)
		{
			if (length > MAX_AIR_DRAG)
			{
				dragForce /= length;
				dragForce *= MAX_AIR_DRAG;
			}
		
			motionState.acceleration += dragForce;
		}
	}

	void updateForces(MotionState &motionState, float mass, float deltaTime)
	{

		motionState.acceleration = glm::clamp(motionState.acceleration, 
			glm::vec2(-MAX_ACCELERATION), glm::vec2(MAX_ACCELERATION));

		//Symplectic Euler
		motionState.velocity += motionState.acceleration * deltaTime * 0.5f;
		motionState.velocity = glm::clamp(motionState.velocity, glm::vec2(-MAX_VELOCITY), glm::vec2(MAX_VELOCITY));

		motionState.pos += motionState.velocity * deltaTime;

		motionState.velocity += motionState.acceleration * deltaTime * 0.5f;
		motionState.velocity = glm::clamp(motionState.velocity, glm::vec2(-MAX_VELOCITY), glm::vec2(MAX_VELOCITY));

		if (std::fabs(motionState.velocity.x) < 0.00001) { motionState.velocity.x = 0; }
		if (std::fabs(motionState.velocity.y) < 0.00001) { motionState.velocity.y = 0; }

		motionState.acceleration = {};
	}


	Collider createBoxCollider(glm::vec2 size)
	{
		Collider c;
		c.type = ColliderBox;

		c.collider.box.size = size;

		return c;
	}

	Collider createCircleCollider(float r)
	{
		Collider c;
		c.type = ColliderCircle;

		c.collider.circle.radius = r;

		return c;
	}

};

bool overlap(ph2d::Body &a, ph2d::Body &b)
{

	if (a.collider.type == ph2d::ColliderCircle &&
		b.collider.type == ph2d::ColliderCircle
		)
	{

		return ph2d::CirclevsCircle(
			glm::vec3(a.motionState.pos, a.collider.collider.circle.radius),
			glm::vec3(b.motionState.pos, b.collider.collider.circle.radius));
	}


	//todo
	return 0;
}


void ph2d::PhysicsEngine::runSimulation(float deltaTime)
{

	auto impulseResolution = [&](auto &A, auto &B, glm::vec2 normal, 
		float velAlongNormal)
	{


		//calculate elasticity
		float e = std::min(A.elasticity, B.elasticity);

		float massInverseA = 1.f / A.mass;
		float massInverseB = 1.f / B.mass;

		if (A.mass == 0 || A.mass == INFINITY) { massInverseA = 0; }
		if (B.mass == 0 || B.mass == INFINITY) { massInverseB = 0; }

		// Calculate impulse scalar
		float j = -(1.f + e) * velAlongNormal;
		j /= massInverseA + massInverseB;

		// Apply impulse
		glm::vec2 impulse = j * normal;
		A.motionState.velocity -= massInverseA * impulse;
		B.motionState.velocity += massInverseB * impulse;
	};

	size_t bodiesSize = bodies.size();
	for (int i = 0; i < bodiesSize; i++)
	{

		applyDrag(bodies[i].motionState);

		updateForces(bodies[i].motionState, 0, deltaTime);

		//detect colisions
		for (int j = 0; j < bodiesSize; j++)
		{

			if (i == j) { break; }

			auto &A = bodies[i];
			auto &B = bodies[j];

			if (A.collider.type == ph2d::ColliderCircle &&
				B.collider.type == ph2d::ColliderCircle
				)
			{

				if (ph2d::CirclevsCircle(
					glm::vec3(A.motionState.pos, A.collider.collider.circle.radius),
					glm::vec3(B.motionState.pos, B.collider.collider.circle.radius)))
				{

					glm::vec2 normal = B.motionState.pos - A.motionState.pos;
					normalizeSafe(normal);

					glm::vec2 relativeVelocity = B.motionState.velocity -
						A.motionState.velocity;

					float velAlongNormal = glm::dot(relativeVelocity, normal);

					// Do not resolve if velocities are separating
					if (velAlongNormal > 0)
					{
						//todo?
						//return;
					}
					else
					{
						impulseResolution(A, B, normal, velAlongNormal);
					}


					////circle overlapping here
					//float overlap = A.collider.collider.circle.radius + B.collider.collider.circle.radius
					//	- glm::distance(A.motionState.pos, B.motionState.pos);
					//
					//glm::vec2 diffVector = (A.motionState.pos - B.motionState.pos);
					//float distLen = glm::length(diffVector);
					//if (distLen == 0)
					//{
					//	diffVector = {1,0};
					//}
					//else
					//{
					//	diffVector /= distLen;
					//}
					//
					//A.motionState.pos += overlap * (diffVector) * 0.5f;
					//B.motionState.pos -= overlap * (diffVector) * 0.5f;

				}
			}
			else if (A.collider.type == ph2d::ColliderBox &&
				B.collider.type == ph2d::ColliderBox)
			{

				if (ph2d::AABBvsAABB(
					A.getAABB(), B.getAABB()))
				{

					glm::vec2 relativeVelocity = B.motionState.velocity -
						A.motionState.velocity;
					glm::vec2 relativeVelocityNormalized = relativeVelocity;
					normalizeSafe(relativeVelocityNormalized);

					glm::vec2 normal = {1,0};

					float up = glm::dot(relativeVelocityNormalized, {0,1});
					float down = glm::dot(relativeVelocityNormalized, {0,-1});
					float left = glm::dot(relativeVelocityNormalized, {-1,0});
					float right = glm::dot(relativeVelocityNormalized, {1,0});

					float current = 0;
					if (up > down)
					{
						current = up;
						normal = {0, 1};
					}
					else
					{
						current = down;
						normal = {0, -1};
					}

					if (left > current)
					{
						current = left;
						normal = {-1,0};
					}

					if (right > current)
					{
						current = right;
						normal = {1,0};
					}

					float velAlongNormal = glm::dot(relativeVelocity, normal);

					impulseResolution(A, B, normal, velAlongNormal);
				}



			}


		}

		bodies[i].motionState.lastPos = bodies[i].motionState.pos;
	}



}


void ph2d::PhysicsEngine::addBody(glm::vec2 centerPos, Collider collider, float mass)
{

	Body body;
	body.motionState.setPos(centerPos);
	body.collider = collider;
	body.mass = mass;

	bodies.push_back(body);


}


ph2d::AABB ph2d::Body::getAABB()
{

	switch (collider.type)
	{

	case ColliderCircle:
	{
		glm::vec4 rez;
		rez.x = motionState.pos.x;
		rez.y = motionState.pos.y;

		rez.x -= collider.collider.circle.radius;
		rez.y -= collider.collider.circle.radius;

		rez.z = collider.collider.circle.radius * 2.f;
		rez.w = collider.collider.circle.radius * 2.f;

		return rez;
	};
	break;

	case ColliderBox:
	{
		glm::vec4 rez;
		rez.x = motionState.pos.x;
		rez.y = motionState.pos.y;

		rez.x -= collider.collider.box.size.x / 2.f;
		rez.y -= collider.collider.box.size.y / 2.f;

		rez.z = collider.collider.box.size.x;
		rez.w = collider.collider.box.size.y;

		return rez;
	}
	break;

	}

	return {};

}

