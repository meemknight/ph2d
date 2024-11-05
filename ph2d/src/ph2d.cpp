//////////////////////////////////////////////////
//ph2d.h				0.0.1
//Copyright(c) 2024 Luta Vlad - Low Level Game Dev
// please credit me if you want, 
// and keep this in the code
//
//////////////////////////////////////////////////


//credits:
//https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-basics-and-impulse-resolution--gamedev-6331

//todo layers
//todo apply force,
//todo optimize collisions using some octrees maybe or something.


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

	//The second is the circle
	bool AABBvsCircle(AABB abox, AABB bbox, float &penetration,
		glm::vec2 &normal)
	{

		// Vector from A to B
		glm::vec2 n = bbox.center() - abox.center();

		// Closest point on A to center of B
		glm::vec2 closest = n;

		// Calculate half extents along each axis
		float x_extent = (abox.max().x - abox.min().x) / 2;
		float y_extent = (abox.max().y - abox.min().y) / 2;

		// Clamp point to edges of the AABB
		closest.x = glm::clamp(closest.x, - x_extent, x_extent);
		closest.y = glm::clamp(closest.y, -y_extent, y_extent);

		bool inside = false;
		// Circle is inside the AABB, so we need to clamp the circle's center
		// to the closest edge
		if (n == closest)
		{

			inside = true;

			// Find closest axis
			if (abs(n.x) > abs(n.y))
			{

				// Clamp to closest extent
				if (closest.x > 0)
				{
					closest.x = x_extent;
				}
				else
				{
					closest.x = -x_extent;
				}
			}
			// y axis is shorter
			else
			{
				// Clamp to closest extent
				if (closest.y > 0)
				{
					closest.y = y_extent;
				}
				else
				{
					closest.y = -y_extent;
				};
			}

		}

		glm::vec2 normal2 = n - closest;
		float d = glm::dot(normal2, normal2);
		float r = bbox.size.x/2.f;

		// Early out of the radius is shorter than distance to closest point and
		// Circle not inside the AABB
		if (d > r * r && !inside)
		{
			return false;
		}


		// Avoided sqrt until we needed
		d = sqrt(d);

		// Collision normal needs to be flipped to point outside if circle was
		// inside the AABB
		if (inside)
		{
			normal = -n;
			penetration = r - d;
		}
		else
		{
			normal = n;
			penetration = r - d;
		}

		normalizeSafe(normal);

		return true;
	}


	bool AABBvsAABB(AABB abox, AABB bbox, float &penetration,
		glm::vec2 &normal)
	{
		// Vector from A to B
		glm::vec2 n = bbox.center() - abox.center();

		// Calculate half extents along x axis for each object
		float a_extent = (abox.max().x - abox.min().x) / 2;
		float b_extent = (bbox.max().x - bbox.min().x) / 2;

		auto aMax = abox.max();
		auto aMin = abox.min();
		auto bMax = bbox.max();
		auto bMin = bbox.min();

		// Calculate overlap on x axis
		float x_overlap = a_extent + b_extent - abs(n.x);

		// SAT test on x axis
		if (x_overlap > 0)
		{

			// Calculate half extents along x axis for each object
			float a_extent = (abox.max().y - abox.min().y) / 2.f;
			float b_extent = (bbox.max().y - bbox.min().y) / 2.f;

			// Calculate overlap on y axis
			float y_overlap = a_extent + b_extent - abs(n.y);

			// SAT test on y axis
			if (y_overlap > 0)
			{

				// Find out which axis is axis of least penetration
				if (x_overlap < y_overlap)
				{

					// Point towards B knowing that n points from A to B
					if (n.x < 0)
					{
						normal = glm::vec2(-1, 0);
					}
					else
					{
						normal = glm::vec2(1, 0);
					}

					penetration = x_overlap;
					return true;
				}
				else
				{

					// Point toward B knowing that n points from A to B
					if (n.y < 0)
					{
						normal = glm::vec2(0, -1);
					}
					else
					{
						normal = glm::vec2(0, 1);
					}

					penetration = y_overlap;
					return true;
				}

			}

		}

		return false;
	}


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


	bool CirclevsCircle(Circle a, Circle b,
		float &penetration,
		glm::vec2 &normal
		)
	{
		float r = a.r + b.r;
		float rSquared = r * r;
		float distanceSquared = ((a.center.x - b.center.x) * (a.center.x - b.center.x)
			+ (a.center.y - b.center.y) * (a.center.y - b.center.y));

		bool rez = rSquared > distanceSquared;

		if(rez)
		{
			normal = b.center - a.center;
			normalizeSafe(normal);
			penetration = r - sqrt(distanceSquared);
		}

		return rez;
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

float PythagoreanSolve(float fA, float fB)
{
	return std::sqrt(fA * fA + fB * fB);
}

bool overlap(ph2d::Body &a, ph2d::Body &b)
{

	

	//todo
	return 0;
}


void ph2d::PhysicsEngine::runSimulation(float deltaTime)
{

	int counter = 0;
	if (setFixedTimeStamp <= 0)
	{
		_fixedTimeAccumulated = 0;
		counter = 1;
	}
	else
	{
		_fixedTimeAccumulated += deltaTime;

		_fixedTimeAccumulated = std::min(_fixedTimeAccumulated, maxAccumulated);
		

		while (_fixedTimeAccumulated > setFixedTimeStamp)
		{
			_fixedTimeAccumulated -= setFixedTimeStamp;
			counter++;
		}

		deltaTime = setFixedTimeStamp;
	}

	for (int currentIteration = 0; currentIteration < counter; currentIteration++)
	{
		auto positionalCorrection = [&](Body &A, Body &B, glm::vec2 n,
			float penetrationDepth, float aInverseMass, float bInverseMass)
		{

			const float percent = 0.25; // usually 20% to 80%
			const float slop = 0.01; // usually 0.01 to 0.1 

			glm::vec2 correction = (glm::max(penetrationDepth - slop, 0.0f) / (aInverseMass + bInverseMass)) * percent * n;

			A.motionState.pos -= aInverseMass * correction;
			B.motionState.pos += bInverseMass * correction;
		};

		auto applyFriction = [&](Body &A, Body &B, glm::vec2 tangent, glm::vec2 rv,
			float aInverseMass, float bInverseMass, float j)
		{
			// Solve for magnitude to apply along the friction vector
			float jt = -glm::dot(rv, tangent);
			jt = jt / (aInverseMass + bInverseMass);

			// PythagoreanSolve = A^2 + B^2 = C^2, solving for C given A and B
			// Use to approximate mu given friction coefficients of each body
			float mu = PythagoreanSolve(A.staticFriction, B.staticFriction);

			// Clamp magnitude of friction and create impulse vector
			//(Coulomb's Law) Ff<=Fn
			glm::vec2 frictionImpulse = {};
			if (abs(jt) < j * mu)
			{
				frictionImpulse = jt * tangent;
			}
			else
			{
				float dynamicFriction = PythagoreanSolve(A.dynamicFriction, B.dynamicFriction);
				frictionImpulse = -j * tangent * dynamicFriction;
			}

			// Apply
			A.motionState.velocity -= (aInverseMass)*frictionImpulse;
			B.motionState.velocity += (bInverseMass)*frictionImpulse;

		};

		auto impulseResolution = [&](Body &A, Body &B, glm::vec2 normal,
			float velAlongNormal, float penetrationDepth)
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

			positionalCorrection(A, B, normal, penetrationDepth, massInverseA, massInverseB);

			{

				// Re-calculate relative velocity after normal impulse
				// is applied (impulse from first article, this code comes
				// directly thereafter in the same resolve function)

				glm::vec2 rv = B.motionState.velocity - A.motionState.velocity;

				// Solve for the tangent vector
				glm::vec2 tangent = rv - glm::dot(rv, normal) * normal;

				normalizeSafe(tangent);
				
				applyFriction(A, B, tangent, rv, massInverseA, massInverseB, j);
			}


		};

		size_t bodiesSize = bodies.size();
		for (int i = 0; i < bodiesSize; i++)
		{

			//applyDrag(bodies[i].motionState);

			//detect colisions
			//for(int _ = 0; _ < 4; _++)
			for (int j = 0; j < bodiesSize; j++)
			{

				if (i == j) { continue; }

				auto &A = bodies[i];
				auto &B = bodies[j];

				if (A.collider.type == ph2d::ColliderCircle &&
					B.collider.type == ph2d::ColliderCircle
					)
				{

					glm::vec2 normal = {};
					float penetration = 0;

					if (ph2d::CirclevsCircle(
						glm::vec3(A.motionState.pos, A.collider.collider.circle.radius),
						glm::vec3(B.motionState.pos, B.collider.collider.circle.radius), 
						penetration, normal))
					{
						glm::vec2 relativeVelocity = B.motionState.velocity -
							A.motionState.velocity;
						float velAlongNormal = glm::dot(relativeVelocity, normal);

						// Do not resolve if velocities are separating
						if (velAlongNormal > 0)
						{

						}
						else
						{
							impulseResolution(A, B, normal, velAlongNormal, penetration);
						}
					}
				}
				else if (A.collider.type == ph2d::ColliderBox &&
					B.collider.type == ph2d::ColliderBox)
				{

					auto abox = A.getAABB();
					auto bbox = B.getAABB();

					glm::vec2 normal = {};
					float penetration = 0;

					if (ph2d::AABBvsAABB(
						abox, bbox, penetration, normal))
					{

						glm::vec2 relativeVelocity = B.motionState.velocity -
							A.motionState.velocity;
						float velAlongNormal = glm::dot(relativeVelocity, normal);
						
						// Do not resolve if velocities are separating
						if (velAlongNormal > 0)
						{
						
						}
						else
						{
							impulseResolution(A, B, normal, velAlongNormal, penetration);
						}
					}

				}
				else if (A.collider.type == ph2d::ColliderBox &&
					B.collider.type == ph2d::ColliderCircle)
				{

					auto abox = A.getAABB();
					auto bbox = B.getAABB();

					glm::vec2 normal = {};
					float penetration = 0;

					if (ph2d::AABBvsCircle(
						abox, bbox, penetration, normal))
					{
						glm::vec2 relativeVelocity = B.motionState.velocity -
							A.motionState.velocity;
						float velAlongNormal = glm::dot(relativeVelocity, normal);

						// Do not resolve if velocities are separating
						if (velAlongNormal > 0)
						{

						}
						else
						{
							impulseResolution(A, B, normal, velAlongNormal, penetration);
						}
					}
				}


			}

			updateForces(bodies[i].motionState, 0, deltaTime);
			bodies[i].motionState.lastPos = bodies[i].motionState.pos;
		}

	};

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

