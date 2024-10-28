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

	bool AABBvsAABB(AABB a, AABB b, float delta = 0);

	bool AABBvsPoint(AABB a, glm::vec2 b, float delta = 0);

	bool CirclevsCircle(Circle a, Circle b, float delta = 0);

	struct MotionState
	{
		glm::vec2 pos = {};
		glm::vec2 lastPos = {};
		glm::vec2 velocity = {};
		glm::vec2 acceleration = {};

		void setPos(glm::vec2 p) { pos = p; lastPos = p; }
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

		float elasticity = 1;
		float mass = 1;

		AABB getAABB();
	};

	struct PhysicsEngine
	{
		std::vector<Body> bodies;

		void runSimulation(float deltaTime);

		void addBody(glm::vec2 centerPos, Collider collider, float mass = 1);
	};

	void updateForces(MotionState &motionState, float mass, float deltaTime);
};


