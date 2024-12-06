#define GLM_ENABLE_EXPERIMENTAL
#include "gameLayer.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include "platformInput.h"
#include "imgui.h"
#include <iostream>
#include <sstream>
#include "imfilebrowser.h"
#include <gl2d/gl2d.h>
#include <platformTools.h>

#include <ph2d/ph2d.h>


// Function to generate a pastel color based on an input number
glm::vec3 generatePastelColor(int inputNumber)
{
	srand(inputNumber);

	// Seed random number generator based on input
	//auto baseColor = glm::vec3(0,0,0);
	//glm::vec3 baseColor = glm::ballRand<float>(1.0f);  // Random direction on a sphere
	glm::vec3 baseColor(rand() % 100 / 100.f, rand() % 100 / 100.f, rand() % 100 / 100.f);
	baseColor = glm::abs(baseColor);            // Ensure positive values

	// Adjust saturation to pastel range by mixing with white
	float pastelFactor = 0.7f;
	glm::vec3 pastelColor = glm::mix(baseColor, glm::vec3(1.0f), pastelFactor);

	// Map input number into [0,1] range
	float modifier = (inputNumber % 1000) / 1000.0f;
	pastelColor += modifier * 0.1f;

	return glm::clamp(pastelColor, 0.0f, 1.0f);
}

gl2d::Renderer2D renderer;
gl2d::Texture wood;

ph2d::PhysicsEngine physicsEngine;

float floorPos = 1050;

bool initGame()
{
	//initializing stuff for the renderer
	gl2d::init();
	renderer.create();

	physicsEngine.simulationphysicsSettings.gravity = glm::vec2(0, 9.81) * 100.f;

	for (int i = 0; i < 10; i++)
	{
		//if (i == 1) { mass = 0; }

		if(1)
		{
			float w = rand() % 100 + 50;
			float h = rand() % 100 + 50;

			physicsEngine.addBody({rand() % 800 + 100, rand() % 800 + 100},
				ph2d::createBoxCollider({w, h}));
			//physicsEngine.bodies.back().motionState.momentOfInertia = 0;
		}

		for(int j = 0; j <1; j++)
		{
			float r = rand() % 35 + 10;
		
			auto b = physicsEngine.addBody({rand() % 800 + 100, rand() % 800 + 100},
				ph2d::createCircleCollider({r}));
			//physicsEngine.bodies[b].motionState.momentOfInertia = 0;

		}
	}

	wood.loadFromFile(RESOURCES_PATH "wood.png");

	//physicsEngine.addBody({500, 1100}, 
	//	ph2d::createBoxCollider({1100, 10}));

	//physicsEngine.addBody({500, 500}, ph2d::createBoxCollider({300, 300}));
	
	//physicsEngine.addBody({700, 700}, ph2d::createBoxCollider({300, 300}));


	
	//physicsEngine.addBody({600, 600}, ph2d::createBoxCollider({350, 100}));
	//physicsEngine.bodies[1].motionState.rotation = glm::radians(30.f);
	
	//physicsEngine.addBody({500, 500}, ph2d::createCircleCollider({55}));
	//physicsEngine.addBody({800, 100}, ph2d::createCircleCollider({55}));
	//physicsEngine.addBody({900, 500}, ph2d::createCircleCollider({25}));
	//physicsEngine.addBody({550, 700}, ph2d::createCircleCollider({25}));

	//std::cout << ph2d::vectorToRotation({0,1}) << "\n";
	//std::cout << ph2d::vectorToRotation({-1,1}) << "\n";
	//std::cout << ph2d::vectorToRotation({-1,0}) << "\n";
	//std::cout << ph2d::vectorToRotation({0,-1}) << "\n";
	//std::cout << ph2d::vectorToRotation({1,0}) << "\n";
	//
	//std::cout << "\n";
	//std::cout << ph2d::rotationToVector(ph2d::vectorToRotation({0,1}) ).x << " " << ph2d::rotationToVector(ph2d::vectorToRotation({0,1}) ).y  << "\n";
	//std::cout << ph2d::rotationToVector(ph2d::vectorToRotation({-1,1})).x << " " << ph2d::rotationToVector(ph2d::vectorToRotation({-1,1})).y << "\n";
	//std::cout << ph2d::rotationToVector(ph2d::vectorToRotation({-1,0})).x << " " << ph2d::rotationToVector(ph2d::vectorToRotation({-1,0})).y << "\n";
	//std::cout << ph2d::rotationToVector(ph2d::vectorToRotation({0,-1})).x << " " << ph2d::rotationToVector(ph2d::vectorToRotation({0,-1})).y << "\n";
	//std::cout << ph2d::rotationToVector(ph2d::vectorToRotation({1,0}) ).x << " " << ph2d::rotationToVector(ph2d::vectorToRotation({1,0}) ).y  << "\n";

	//physicsEngine.addHalfSpaceStaticObject({0, floorPos}, {0, 1});
	{
		auto b = physicsEngine.addBody({500, floorPos - 30}, ph2d::createBoxCollider({1000, 50}));
		physicsEngine.bodies[b].motionState.mass = 0;
		physicsEngine.bodies[b].motionState.momentOfInertia = 0;
		physicsEngine.bodies[b].motionState.rotation = glm::radians(0.f);
	}
	return true;
}


bool gameLogic(float deltaTime)
{

#pragma region init stuff
	int w = 0; int h = 0;
	w = platform::getFrameBufferSizeX(); //window w
	h = platform::getFrameBufferSizeY(); //window h

	glViewport(0, 0, w, h);
	glClearColor(0.2, 0.22, 0.23, 1);
	glClear(GL_COLOR_BUFFER_BIT); //clear screen

	renderer.updateWindowMetrics(w, h);
#pragma endregion


	//ph2d::AABB box1 = glm::vec4{300,300, 300,300};
	//
	//ph2d::AABB box2 = glm::vec4(glm::vec2(platform::getRelMousePosition()) - glm::vec2(50, 50), 100, 100);
	//
	//renderer.renderRectangleOutline(box1.asVec4(), Colors_White);
	//
	//auto color = Colors_White;
	//
	//if (ph2d::AABBvsAABB(box1, box2))
	//{
	//	color = Colors_Red;
	//}
	//
	//renderer.renderRectangleOutline(box2.asVec4(), color);


	//ph2d::Circle a = glm::vec3{450,450, 150};
	//ph2d::Circle b = glm::vec3(glm::vec2(platform::getRelMousePosition()), 50);
	//renderer.renderCircleOutline(a.center, a.r, Colors_White, 2, 32);
	//
	//auto color = Colors_White;
	//if (ph2d::CirclevsCircle(a, b))
	//{
	//	color = Colors_Red;
	//}
	//renderer.renderCircleOutline(b.center, b.r, color, 2, 32);
	static int simulationSpeed = 1;
	float rightPos = 1050;

	//physicsEngine.bodies[0].motionState.rotation = glm::radians(-30.f);


	ImGui::PushStyleColor(ImGuiCol_WindowBg, {0.3,0.3,0.3,0.8});
	ImGui::Begin("Settings");

	ImGui::DragInt("Speed", &simulationSpeed);

	//ImGui::SliderAngle("ANgle", &physicsEngine.bodies[1].motionState.rotation);

	ImGui::End();
	ImGui::PopStyleColor();

	static int selected = -1;

	//mouse
	//physicsEngine.bodies[0].motionState.setPos(platform::getRelMousePosition());

	static glm::vec2 pressedPosition = {};

	if (platform::isLMousePressed())
	{
		selected = -1;

		for (auto &b : physicsEngine.bodies)
		{
			if (OBBvsPoint(b.second.getAABB(),
				b.second.motionState.rotation,
				platform::getRelMousePosition()))
			{
				selected = b.first;
				pressedPosition = platform::getRelMousePosition();
			}
		}
	}

	if (selected >= 0)
	{
		renderer.renderLine(pressedPosition, platform::getRelMousePosition(), Colors_Blue, 4);
	}

	if (platform::isLMouseReleased() && selected >= 0)
	{

		glm::vec2 force = pressedPosition - glm::vec2(platform::getRelMousePosition());

		//physicsEngine.bodies[selected].motionState.velocity += force;
		force *= physicsEngine.bodies[selected].motionState.mass;

		physicsEngine.bodies[selected].motionState.applyImpulseWorldPosition(force, 
			physicsEngine.bodies[selected].motionState.pos,
			physicsEngine.bodies[selected].flags);

		//physicsEngine.bodies[selected].motionState.angularVelocity = 10;

		selected = -1;
		pressedPosition = {};
	}
	

	for (int i = 0; i < simulationSpeed; i++)
	{

		//gravity
		//for (int i=0; i<physicsEngine.bodies.size(); i++)
		//{
		//	if(physicsEngine.bodies[i].motionState.mass != 0 && physicsEngine.bodies[i].motionState.mass != INFINITY)
		//		physicsEngine.bodies[i].motionState.acceleration += glm::vec2(0, 9.81) * 100.f;
		//}

		physicsEngine.runSimulation(deltaTime);

		for (auto &i : physicsEngine.bodies)
		{
			auto &b = i.second;

			auto bottom = b.getAABB().max().y;
			auto left = b.getAABB().min().x;
			auto right = b.getAABB().max().x;
			auto top = b.getAABB().min().y;

			if (bottom > floorPos)
			{
				float diff = bottom - floorPos;
				b.motionState.pos.y -= diff;
				b.motionState.lastPos = b.motionState.pos;
			
				b.motionState.velocity.y *= -0.2;
			}
			
			if (left < 0)
			{
				b.motionState.pos.x -= left;
				b.motionState.lastPos = b.motionState.pos;

				b.motionState.velocity.x *= -0.2;
			}

			if (right > rightPos)
			{
				b.motionState.pos.x -= right - rightPos;
				b.motionState.lastPos = b.motionState.pos;

				b.motionState.velocity.x *= -0.2;
			}

			if (top < 0)
			{
				b.motionState.pos.y -= top;
				b.motionState.lastPos = b.motionState.pos;
				
				b.motionState.velocity.y *= -0.2;
			}
		}
	}

	//std::cout << physicsEngine.bodies[0].getAABB().max().x << "\n";

	//renderer.renderRectangleOutline({300 - 25, 100 - 25, 50, 50}, Colors_Red);
	//renderer.renderRectangleOutline({600 - 25, 200 - 25, 50, 50}, Colors_Red);

	float p = 0;
	glm::vec2 n = {};
	bool penetrated = 0;
	glm::vec2 contactPoint = {};

	glm::vec2 tangengA = {};
	glm::vec2 tangengB = {};

	if (ph2d::BodyvsBody(physicsEngine.bodies[0],
		physicsEngine.bodies[1],
		p, n, contactPoint, tangengA, tangengB ))
	{
		penetrated = true;
	}

	//if (ph2d::OBBvsOBB(physicsEngine.bodies[0].getAABB(),
	//	physicsEngine.bodies[0].motionState.rotation,
	//	physicsEngine.bodies[1].getAABB(),
	//	physicsEngine.bodies[1].motionState.rotation,
	//	p, n, contactPoint))
	//{
	//	penetrated = true;
	//}

	//auto a = physicsEngine.bodies[0].getAABB();
	//auto b = physicsEngine.bodies[1].getAABB();
	//ph2d::Circle a1 = glm::vec3{a.center(),a.size.x / 2};
	//ph2d::Circle b1 = glm::vec3{b.center(),b.size.x / 2};
	//if (ph2d::CirclevsCircle(a1, b1, p, n, contactPoint))
	//{
	//	penetrated = true;
	//}


	//auto a = physicsEngine.bodies[0].getAABB();
	//auto b = physicsEngine.bodies[1];
	//ph2d::LineEquation lineEquation;
	//lineEquation.createFromRotationAndPoint(b.motionState.rotation,
	//	b.motionState.pos);
	//if (ph2d::HalfSpaceVSCircle(lineEquation, a, p, n, contactPoint))
	//{
	//	penetrated = true;
	//}

	//glm::vec2 cornersA[4] = {};
	//glm::vec2 cornersB[4] = {};
	//physicsEngine.bodies[0].getAABB().getCornersRotated(cornersA, physicsEngine.bodies[0].motionState.rotation);
	//physicsEngine.bodies[1].getAABB().getCornersRotated(cornersB, physicsEngine.bodies[1].motionState.rotation);
	//
	//float rez = ph2d::calculatePenetrationAlongOneAxe(cornersA, 4, cornersB, 4, {1,0});
	//if (rez > 0)
	//{
	//	penetrated = true;
	//}

	ImGui::Begin("Settings");
	ImGui::Text("Penetration: %f", p);
	ImGui::End();


	for (int i=0; i< physicsEngine.bodies.size(); i++)
	{
		auto &b = physicsEngine.bodies[i];

		auto color = glm::vec4(generatePastelColor(i), 1);

		if (i == selected)
		{
			color = Colors_Blue;
		}
		
		if (b.intersectPoint(platform::getRelMousePosition()))
		{
			color = Colors_Turqoise;
		}

		if (penetrated)
		{
			color = Colors_Red;
		}

		if (b.collider.type == ph2d::ColliderCircle)
		{
			renderer.renderCircleOutline(b.motionState.pos,
				b.collider.collider.circle.radius, color, 4, 32);

			glm::vec2 vector = {1,0};
			vector = ph2d::rotateAroundCenter(vector, b.motionState.rotation);
			renderer.renderLine(b.motionState.pos, b.motionState.pos + vector *
				b.collider.collider.circle.radius, color, 4);

		}
		else if (b.collider.type == ph2d::ColliderBox)
		{
			float rotation = glm::degrees(b.motionState.rotation);

			//renderer.renderRectangle(b.getAABB().asVec4(), wood, Colors_White, {}, rotation);

			renderer.renderRectangleOutline(b.getAABB().asVec4(), color, 4, {}, rotation);
		}
		else if (b.collider.type == ph2d::ColliderHalfSpace)
		{

			ph2d::LineEquation lineEquation;
			lineEquation.createFromRotationAndPoint(b.motionState.rotation,
				b.motionState.pos);

			glm::vec2 lineEquationStart = lineEquation.getClosestPointToOrigin();
			lineEquationStart -= lineEquation.getLineVector() * 1000.f;
			renderer.renderLine(lineEquationStart, lineEquationStart + lineEquation.getLineVector() * 2000.f, Colors_Red, 4);


		}
	}

	//renderer.renderRectangle({-100, floorPos, 100000, 20});

	if (penetrated)
	{
		renderer.renderLine(contactPoint,
			contactPoint + n * 100.f, Colors_Green, 4);

		renderer.renderRectangle({contactPoint - glm::vec2(2,2), 4,4}, Colors_White);
	}


	//glm::vec2 lineEquationStart = lineEquation.getClosestPointToOrigin();
	//lineEquationStart -= lineEquation.getLineVector() * 1000.f;
	//renderer.renderLine(lineEquationStart, lineEquationStart + lineEquation.getLineVector() * 2000.f, Colors_Red);


	//ph2d::LineEquation lineEquation;
	//lineEquation.createFromNormalAndPoint({0,1}, {0, floorPos});

	//float pl = 0;
	//pl = lineEquation.computeEquation(platform::getRelMousePosition());
	//ImGui::Begin("Settings");
	//ImGui::Text("Penetration line: %f", pl);
	//ImGui::End();


	//glm::vec2 p2 = platform::getRelMousePosition();
	//p2 = ph2d::rotateAroundCenter(p2, glm::radians(45.f));
	//renderer.renderRectangle({p2, 10, 10}, Colors_Red);

	renderer.flush();

	

	return true;
#pragma endregion

}

//This function might not be be called if the program is forced closed
void closeGame()
{


}
