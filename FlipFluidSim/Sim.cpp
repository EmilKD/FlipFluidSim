#include<iostream>
#include<chrono>
#include"../Oasis/Source/Headers/RenderSystem.h"
#include"../FlipFluidSim/src/Headers/Fluid.h"

int screenSize[2] = { 1080, 720 };
float worldSize[2] = {0.4, 0.3};

render_system::RenderSystem RS(screenSize);

int main()
{
	Fluid fluidSim(worldSize[0], worldSize[1]);
	//fluidSim.AddObstacle(new CircularObj(0.f, 0.f, 0.04f, fluidSim));

	const int& numOfCells = fluidSim.cells.size() * 2;

	double frameTime{ 0 };
	vec2 cursorPos{ vec2(0.f) }, cursorPrevPos{ vec2(0.f) };

	auto frameTimer = new std::chrono::high_resolution_clock();

	while (RS.CheckWindowClosureStatus())
	{
		auto frameTimerStart = frameTimer->now();

		cursorPrevPos = cursorPos;
		cursorPos = RS.GetCursorPos();
		const vec2& cursorVel = (cursorPos - cursorPrevPos) / (float)frameTime;

		//fluidSim.UpdateObstacle(0, cursorPos.x/screenSize[0] * worldSize[0], (1 - cursorPos.y/screenSize[1])*worldSize[1]);
		fluidSim.AddVelocity(
			cursorPos.x / screenSize[0] * worldSize[0], // x
			(1 - cursorPos.y / screenSize[1]) * worldSize[1], // y 
			cursorVel.x / screenSize[0] * worldSize[0], // u
			-(cursorVel.y / screenSize[1]) * worldSize[1]); // v

		fluidSim.Simulate(frameTime);
		RS.RenderPointCloud(fluidSim.GetPosBuffer(), fluidSim.GetColorBuffer(), numOfCells, RS.pointShader, 15);
		RS.UpdateWindow();
		
		auto frameTimerEnd = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double> frameDuration = frameTimerEnd - frameTimerStart;
		frameTime = frameDuration.count(); 
	}

	//RS.Terminate();
	return 0;
}