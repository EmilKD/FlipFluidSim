#pragma once

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>
#include<vector>
#include<array>
#include<execution>

#define vec3 glm::vec3
#define vec2 glm::vec2

using std::vector, std::array;

struct cell
{
	vec3 pos{0.0f, 0.0f, 0.0f};

	int id, s{ 1 };

	float u{ 0.0f };
	float v{ 0.0f};
	float newU{ 0.0f };
	float newV{ 0.0f };
	float m{ 0.0f };
	float newM{ 0.0f };

	double p{ 0.0f };

	int id_right{ 0 }, id_left{ 0 }, id_up{ 0 }, id_down{ 0 };
};

struct CircularObj;

class Fluid
{
	
public:
	Fluid(const float& Size_x, const float& Size_y);	
	//void render(GraphicalObj* gobj, const float &renderScale_x, const float &renderScale_y);
	void project(double dt);
	void advectVelocity(double dt);
	void project_parallel(double dt);
	void advectVelocity_parallel(double dt);
	void advectSmoke(double dt);
	void simulate(double dt);
	void extrapolate();
	void AddObstacle(CircularObj* obj);
	void UpdateObstacle(int id, const float& newPosX, const float& newPosY);
	void UpdateCellColor(int idx, const vec3& color);
	void GenPosBuffer();
	void GenColorBuffer();
	vec2 sampleVelocity(const vec2 &samplePos);
	float sampleDensity(const vec2& samplePos);
	void AddVelocity(const float& posX, const float& posY, const float& u, const float& v);

	vector<float> GetPosBuffer() { return PositionBuffer; };
	vector<float> GetColorBuffer() { return ColorBuffer; };

	float xC(float worldx) {
		return 2 * (worldx / worldSize_x - 0.5);
	}

	float yC(float worldy) {
		return 2 * (worldy / worldSize_y - 0.5);
	}

public:
	static constexpr float gridSize = 0.001; // meters
	const int gridCount_x;
	const int gridCount_y;
	vector<cell> cells;
	float InletVel[2]{ .2f, 0.f };

private:
	const float worldSize_x, worldSize_y;
	vector<cell*> cellPtrs;
	vector<CircularObj*> Obstacles{};
	vector<uint32_t> IterHeight, IterWidth, IterIndices, IterSubSteps;

	// Creating the grid graphical object
	const int ArraySize{ gridCount_x * gridCount_y };
	vector<float> PositionBuffer, ColorBuffer;

	int substeps{ 20 }, inputIdx{-1};
	double ndt{ 0 }, d;
	float de{1.0}, cp;
	float density{ 1000.0f };
	float overRelaxation{ 1.9f };
	float simulationTime{ 0.0f };
	float avgV{ 0.0f }, avgU{ 0.0f };
	float inputU{ 0.f }, inputV{ 0.f }, inputM{ 0.f };
};

struct CircularObj
{
	float x{ 0.0f }, y{ 0.0f }, radius{ 0.05f }, u{ 0.f }, v{ 0.f }, gridSize{ 0.01 };
	int gridCountX{ 0 }, gridCountY{ 0 };
	vector<int> cellIds{};
	vector<uint32_t> IterSphere;
	Fluid* grid{ nullptr };

	CircularObj(const float x, const float y, const float radius, Fluid& grid)
		: x{ x }, y{ y }, radius{ radius }, gridSize{ grid.gridSize }, gridCountX{ grid.gridCount_x }, gridCountY{ grid.gridCount_y }, grid{&grid}
	{
		for (int i = int((x - radius) / gridSize); i < int((x + radius) / gridSize); i++)
		{
			for (int j = int((y - radius) / gridSize); j < int((y + radius) / gridSize); j++)
			{
				const int& idx = i + j * gridCountX;
				if (std::sqrt(std::pow(grid.cells[idx].pos.x - x, 2) + std::pow(grid.cells[idx].pos.y - y, 2)) <= radius)
				{
					cellIds.push_back(idx);
					//IterSphere.push_back(idx);
				}
			}
		}
	}

	void Update(const float x, const float y, const float radius)
	{
		this->radius = radius; this->x = x; this->y = y;
		for (int& cid : cellIds)
			grid->cells[cid].s = 1;
		cellIds.clear();
		
		for (int i = int((x - radius) / gridSize); i < int((x + radius) / gridSize); i++)
		{
			for (int j = int((y - radius) / gridSize); j < int((y + radius) / gridSize); j++)
			{
				const int& idx = std::clamp(i + j * gridCountX, 0, (int)grid->cells.size());
				if (std::sqrt(std::pow(grid->cells[idx].pos.x - x, 2) + std::pow(grid->cells[idx].pos.y - y, 2)) <= radius)
				{
					cellIds.push_back(idx);
					grid->cells[idx].s = 0;
				}
			}
		}
	}

};