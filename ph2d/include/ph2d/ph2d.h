//////////////////////////////////////////////////
//ph2d.h				0.0.1
//Copyright(c) 2024 Luta Vlad - Low Level Game Dev
// please credit me if you want, 
// and keep this in the code
//
//	dependences: glm
//
//////////////////////////////////////////////////

//Library conventions

//for positions and size we use vec4 or AABB. But the idea is, whenever I use a vec4,
// that means position x, position y; size x, size y.
// you can easily convert from a vec4 to an aabb and back

#undef min
#undef max

#include <glm/glm.hpp>
#include <vector>

namespace ph2d
{

	struct AABB
	{
		AABB() {};

		AABB(glm::vec4 a): pos(a.x, a.y), size(a.z, a.w) {};

		glm::vec2 pos = {};
		glm::vec2 size = {};

		glm::vec4 asVec4() { return glm::vec4{pos, size}; }

		glm::vec2 min() { return pos; }
		glm::vec2 max() { return pos + size; }
		glm::vec2 center() { return pos + size / 2.f; }

		void getMinMaxPointsRotated(glm::vec2 &outMin, glm::vec2 &outMax, float r);

		void rotateAroundCenter(float r);

		float down() { return pos.y + size.y; }
		float top() { return pos.y; }
		float left() { return pos.x; }
		float right() { return pos.x + size.x; }
	};

	struct Circle
	{
		Circle() {};

		Circle(glm::vec3 a): center(a.x, a.y), r(a.z) {};

		glm::vec2 center = {};
		float r = 0;

		glm::vec2 getTopLeftCorner() { return center += glm::vec2(-r, r); }

		AABB getAABB() { return glm::vec4(getTopLeftCorner(), r*2, r*2); }
	};

	bool OBBvsOBB(AABB a, float ar, AABB b, float br, float &penetration, glm::vec2 &normal);

	bool AABBvsAABB(AABB a, AABB b, float delta = 0);

	bool AABBvsPoint(AABB a, glm::vec2 b, float delta = 0);

	bool OBBvsPoint(AABB a, float rotation, glm::vec2 b, float delta = 0);

	bool CircleVsPoint(glm::vec2 pos, float r, glm::vec2 p, float delta = 0);

	glm::vec2 rotateAroundCenter(glm::vec2 in, float r);

	glm::vec2 rotateAroundPoint(glm::vec2 in, glm::vec2 centerReff, float r);

	struct MotionState
	{
		glm::vec2 pos = {};
		glm::vec2 lastPos = {};
		glm::vec2 velocity = {};
		glm::vec2 acceleration = {};

		float rotation = 0; // radians
		float angularVelocity = 0;
		float torque = 0;
		float mass = 1;
		float momentOfInertia = 1;

		void setPos(glm::vec2 p) { pos = p; lastPos = p; }

		void applyImpulseObjectPosition(glm::vec2 impulse, glm::vec2 contactVector);
		
		void applyImpulseWorldPosition(glm::vec2 impulse, glm::vec2 contactVectorWorldPos);

	};

	struct Collider
	{
		union
		{
			struct
			{
				float radius;
			}circle;

			struct
			{
				glm::vec2 size;
			}box;
		} collider;

		unsigned char type = 0;

		float computeMass();

		float computeMomentOfInertia(float mass);
	};

	enum ColliderType
	{
		ColliderNone = 0,
		ColliderCircle,
		ColliderBox,
	};

	Collider createBoxCollider(glm::vec2 size);

	Collider createCircleCollider(float r);

	struct Body
	{
		//we use center position
		MotionState motionState;

		Collider collider = {};

		float elasticity = 0.2;
		float staticFriction = 0.5;
		float dynamicFriction = 0.5;

		AABB getAABB();

		bool intersectPoint(glm::vec2 p, float delta = 0);
	};

	struct PhysicsEngine
	{
		std::vector<Body> bodies;

		//this will make the simulation run at fixed intervals of time, making it deterministic.
		// it is measured in secconds, so if you set it to 0.016, it will be updated 60 times a seccond, 
		// if you set it to 0.032, it will be updated 30 times a seccond, and a smaller number will get a lot more updates!
		//you can set it to 0 if you want it to just update with the deltaTime;
		float setFixedTimeStamp = 0.008;
		float maxAccumulated = 0.32;

		float _fixedTimeAccumulated = 0;
		void runSimulation(float deltaTime);

		void addBody(glm::vec2 centerPos, Collider collider);
	};

	glm::mat2 rotationMatrix(float angle);

	void integrateForces(MotionState &motionState, float mass, float deltaTime);
};


