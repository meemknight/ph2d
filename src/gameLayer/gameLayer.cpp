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


gl2d::Renderer2D renderer;

ph2d::PhysicsEngine physicsEngine;

bool initGame()
{
	//initializing stuff for the renderer
	gl2d::init();
	renderer.create();

	for (int i = 0; i < 0; i++)
	{
		//if (i == 1) { mass = 0; }

		if(1)
		{
			float w = rand() % 100 + 20;
			float h = rand() % 100 + 20;

			physicsEngine.addBody({rand() % 800 + 100, rand() % 800 + 100},
				ph2d::createBoxCollider({w, h}));
		}

		if(1)
		{
			float r = rand() % 60 + 10;

			physicsEngine.addBody({rand() % 800 + 100, rand() % 800 + 100},
				ph2d::createCircleCollider({r}));
		}
	}

	//physicsEngine.addBody({500, 1100}, 
	//	ph2d::createBoxCollider({1100, 10}));

	physicsEngine.addBody({500, 500}, ph2d::createBoxCollider({200, 200}));


	physicsEngine.addBody({700, 500}, ph2d::createBoxCollider({400, 100}));
	physicsEngine.bodies[1].motionState.rotation = glm::radians(30.f);

	//physicsEngine.addBody({600, 200}, ph2d::createCircleCollider({25}));
	//physicsEngine.addBody({800, 100}, ph2d::createCircleCollider({25}));
	//physicsEngine.addBody({900, 500}, ph2d::createCircleCollider({25}));
	//physicsEngine.addBody({550, 700}, ph2d::createCircleCollider({25}));

	return true;
}


//IMPORTANT NOTICE, IF YOU WANT TO SHIP THE GAME TO ANOTHER PC READ THE README.MD IN THE GITHUB
//https://github.com/meemknight/cmakeSetup
//OR THE INSTRUCTION IN THE CMAKE FILE.
//YOU HAVE TO CHANGE A FLAG IN THE CMAKE SO THAT RESOURCES_PATH POINTS TO RELATIVE PATHS
//BECAUSE OF SOME CMAKE PROGBLMS, RESOURCES_PATH IS SET TO BE ABSOLUTE DURING PRODUCTION FOR MAKING IT EASIER.

bool gameLogic(float deltaTime)
{
#pragma region init stuff
	int w = 0; int h = 0;
	w = platform::getFrameBufferSizeX(); //window w
	h = platform::getFrameBufferSizeY(); //window h

	glViewport(0, 0, w, h);
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
	float floorPos = 1050;
	float rightPos = 1050;

	//physicsEngine.bodies[0].motionState.rotation = glm::radians(-30.f);


	ImGui::PushStyleColor(ImGuiCol_WindowBg, {0.3,0.3,0.3,0.8});
	ImGui::Begin("Settings");

	ImGui::DragInt("Speed", &simulationSpeed);

	ImGui::SliderAngle("ANgle", &physicsEngine.bodies[0].motionState.rotation);

	ImGui::End();
	ImGui::PopStyleColor();

	static int selected = -1;

	physicsEngine.bodies[0].motionState.setPos(platform::getRelMousePosition());

	static glm::vec2 pressedPosition = {};

	if (platform::isLMousePressed())
	{
		selected = -1;

		for (int i = 0; i < physicsEngine.bodies.size(); i++)
		{
			if (OBBvsPoint(physicsEngine.bodies[i].getAABB(),
				physicsEngine.bodies[i].motionState.rotation,
				platform::getRelMousePosition()))
			{
				selected = i;
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

		physicsEngine.bodies[selected].motionState.applyImpulseWorldPosition(force, pressedPosition);

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

		for (auto &b : physicsEngine.bodies)
		{
			auto bottom = b.getAABB().max().y;
			auto left = b.getAABB().min().x;
			auto right = b.getAABB().max().x;
			auto top = b.getAABB().min().y;

			if (bottom > floorPos)
			{
				float diff = bottom - floorPos;
				b.motionState.pos.y -= diff;
				b.motionState.lastPos = b.motionState.pos;

				b.motionState.velocity.y *= -0.9;
			}
			
			if (left < 0)
			{
				b.motionState.pos.x -= left;
				b.motionState.lastPos = b.motionState.pos;

				b.motionState.velocity.x *= -0.9;
			}

			if (right > rightPos)
			{
				b.motionState.pos.x -= right - rightPos;
				b.motionState.lastPos = b.motionState.pos;

				b.motionState.velocity.x *= -0.9;
			}

			if (top < 0)
			{
				b.motionState.pos.y -= top;
				b.motionState.lastPos = b.motionState.pos;
				
				b.motionState.velocity.y *= -0.9;
			}
		}
	}

	//std::cout << physicsEngine.bodies[0].getAABB().max().x << "\n";

	physicsEngine.bodies[0].motionState.pos.x;

	//renderer.renderRectangleOutline({300 - 25, 100 - 25, 50, 50}, Colors_Red);
	//renderer.renderRectangleOutline({600 - 25, 200 - 25, 50, 50}, Colors_Red);

	float p = 0;
	glm::vec2 n = {};
	bool penetrated = 0;

	if (OBBvsOBB(physicsEngine.bodies[0].getAABB(),
		physicsEngine.bodies[0].motionState.rotation,
		physicsEngine.bodies[1].getAABB(),
		physicsEngine.bodies[1].motionState.rotation,
		p, n))
	{
		penetrated = true;
	}

	for (int i=0; i< physicsEngine.bodies.size(); i++)
	{
		auto &b = physicsEngine.bodies[i];

		auto color = Colors_White;

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
				b.collider.collider.circle.radius, color, 2, 32);
		}
		else if (b.collider.type == ph2d::ColliderBox)
		{
			float rotation = glm::degrees(b.motionState.rotation);

			renderer.renderRectangleOutline(b.getAABB().asVec4(), color, 2, {}, rotation);
		}
	}



	renderer.renderRectangle({-100, floorPos, 100000, 20});

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
