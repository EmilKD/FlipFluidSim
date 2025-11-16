#include<iostream>
#include<random>
#include<functional>
#include"../src/Headers/Fluid.h"
#include<math.h>
#include<algorithm>

bool randomBool() {
	static auto gen = std::bind(std::uniform_int_distribution<>(0, 1), std::default_random_engine());
	return gen();
}

Fluid::Fluid(const float& Size_x, const float& Size_y) :
	gridCount_x{ static_cast<int>(Size_x / gridSize) },
	gridCount_y{ static_cast<int>(Size_y / gridSize) },
	worldSize_x{ Size_x }, worldSize_y{ Size_y }
{
	cell initialCell;
	initialCell.m = 0.001f;
	initialCell.u = 0;
	initialCell.v = 0;
	initialCell.s = 1;

	cells.resize(gridCount_x * gridCount_y, initialCell);

	for (int i{ 0 }; i < gridCount_x; i++)
	{
		for (int j{ 0 }; j < gridCount_y; j++)
		{
			int idx = i + j * gridCount_x;
			cell& thisCell = cells[idx];
			thisCell.pos = vec3(
				worldSize_x * (i + 0.5f) / gridCount_x,
				worldSize_y * (j + 0.5f) / gridCount_y,
				0.0f
			);

			// Boundary conditions
			// Default inward neighbors
			int left = (i > 0) ? idx - 1 : idx;
			int right = (i < gridCount_x - 1) ? idx + 1 : idx;
			int down = (j > 0) ? idx - gridCount_x : idx;
			int up = (j < gridCount_y - 1) ? idx + gridCount_x : idx;

			// LEFT = INLET
			if (i == 0) {
				/*thisCell.s = 1;
				thisCell.u = InletVel[0];
				thisCell.v = InletVel[1];
				if (j < (gridCount_y / 2 + 20) && j >(gridCount_y / 2 - 20))
					thisCell.m = 1;*/
				//left = idx;         
				//right = idx + 1;   
			}

			// RIGHT = OUTLET (OPEN)
			if (i == gridCount_x - 1) {
				//right = idx;        
				//eft = idx - 1;
			}

			if (i == 0 || j == 0 || i == gridCount_x - 1 || j == gridCount_y - 1)
				thisCell.s = 0;

			thisCell.id_left = left;
			thisCell.id_right = right;
			thisCell.id_up = up;
			thisCell.id_down = down;
		}
	}

	GenPosBuffer();
	GenColorBuffer();

	IterWidth.resize(gridCount_x);
	for (size_t i = 0; i < gridCount_x; i++)
		IterWidth.at(i) = i;

	IterHeight.resize(gridCount_y);
	for (size_t i = 0; i < gridCount_y; i++)
		IterHeight.at(i) = i;

	IterSubSteps.resize(substeps);
	for (size_t i = 0; i < substeps; i++)
		IterSubSteps.at(i) = i;

	const int spacing = 4;
	IterIndices.resize((gridCount_x) * (gridCount_y));
	for (size_t s = 0; s < spacing; s++)
		for (size_t p = 0; p < spacing; p++)
			for (size_t j = s; j < gridCount_y; j += spacing)
				for (size_t i = p; i < gridCount_x; i += spacing)
					IterIndices[i + j * (gridCount_x)] = i + j * gridCount_x;
				
	if ((gridCount_x) * (gridCount_y) != IterIndices.size())
	{
		printf("Iterator size: %i\n", IterIndices.size());
		printf("correct size: %i\n", (gridCount_x - 2) * (gridCount_y - 2));
		throw std::invalid_argument("iterator construction failed");
	}
}

void Fluid::GenPosBuffer() 
{
	PositionBuffer.resize(ArraySize * 2);

	for (size_t i = 0; i < ArraySize; i++)
	{
		PositionBuffer[i * 2] = xC(cells[i].pos.x);
		PositionBuffer[i * 2 + 1] = yC(cells[i].pos.y);
	}	
}

void Fluid::GenColorBuffer()
{
	ColorBuffer.resize(ArraySize * 3);

	for (size_t i = 0; i < ArraySize; i++)
	{
		const cell& thisCell = cells[i];
		if (thisCell.s)
		{
			if (thisCell.m != 0)
			{
				ColorBuffer[i * 3] = thisCell.m;
				ColorBuffer[i * 3 + 1] = thisCell.m;
				ColorBuffer[i * 3 + 2] = thisCell.m;
			}
		}
		else
		{
			ColorBuffer[i * 3] = 1.0f;
			ColorBuffer[i * 3 + 1] = 0.749f;
			ColorBuffer[i * 3 + 2] = 0.f;
		}
	}
}

void Fluid::UpdateCellColor(int idx, const vec3& color)
{
	const cell& thisCell = cells[idx];
	if (thisCell.s)
	{

		ColorBuffer[idx * 3] = color.x;
		ColorBuffer[idx * 3 + 1] = color.y;
		ColorBuffer[idx * 3 + 2] = color.z;
		
	}
}

void Fluid::AddObstacle(CircularObj* obj)
{
	this->Obstacles.push_back(obj);
	UpdateObstacle(0, 0.15, 0);
}

void Fluid::UpdateObstacle(int id, const float& newPosX, const float& newPosY)
{
	CircularObj* obstacle = Obstacles.at(id);
	obstacle->Update(newPosX, newPosY, obstacle->radius);
	/*for (cell* c : obstacle->cells)
	{
		c->s = 0;
		c->u = sin((c->pos.x - obstacle->x) / obstacle->radius) * obstacle->u;
		c->v = cos((c->pos.y - obstacle->y) / obstacle->radius) * obstacle->v;
		c->m = 0;
	}*/
}

void Fluid::ProjectParallel(double dt) {
	cp = this->density * gridSize / dt;

	std::for_each(std::execution::par_unseq, IterSubSteps.begin(), IterSubSteps.end(), [this](uint8_t n) 
	{
		std::for_each(std::execution::par_unseq, IterIndices.begin(), IterIndices.end(), [this](uint32_t i)
		{
			cell& thisCell = cells[i];

			//if (thisCell.s == 0)
				//continue;

			const double& d = (thisCell.u - cells[thisCell.id_right].u + thisCell.v - cells[thisCell.id_up].v) * overRelaxation;
			const int& s = cells[thisCell.id_down].s + cells[thisCell.id_up].s + cells[thisCell.id_left].s + cells[thisCell.id_right].s;

			if (s == 0) // Obstacle velocity should be added
				return;

			thisCell.p += d / s * cp;
			thisCell.u -= d / s * cells[thisCell.id_left].s;
			cells[thisCell.id_right].u += d / s * cells[thisCell.id_right].s;
			thisCell.v -= d / s * cells[thisCell.id_down].s;
			cells[thisCell.id_up].v += d / s * cells[thisCell.id_up].s;
		});
		if (n < substeps - 1)
			return;
		else
		{
			for (size_t i{ 0 }; i < gridCount_x; i++)
			{
				for (size_t j{ 0 }; j < gridCount_y; j++)
				{
					cell& thisCell = cells[i + j * gridCount_x];
					thisCell.p /= substeps;
				}
			}
		}
	});
}

void Fluid::Project(double dt)
{
	cp = this->density * gridSize / dt;
	for (cell& c : cells)
		c.p = 0;

	for (size_t n{0}; n < substeps; n++)
	{
		for (size_t i{ 0 }; i < gridCount_x; i++)
		{
			for (size_t j{ 0 }; j < gridCount_y; j++)
			{
				cell& thisCell = cells[i + j * gridCount_x];

				//if (thisCell.s == 0)
					//continue;

				const double& d = (thisCell.u - cells[thisCell.id_right].u + thisCell.v - cells[thisCell.id_up].v) * overRelaxation;
				const int& s = cells[thisCell.id_down].s + cells[thisCell.id_up].s + cells[thisCell.id_left].s + cells[thisCell.id_right].s;

				if (s == 0) // Obstacle velocity should be added
					continue;

				thisCell.p += d / s * cp;
				thisCell.u -= d / s * cells[thisCell.id_left].s;
				cells[thisCell.id_right].u += d / s * cells[thisCell.id_right].s;
				thisCell.v -= d / s * cells[thisCell.id_down].s;
				cells[thisCell.id_up].v += d / s * cells[thisCell.id_up].s;
			}
		}
		if (n < substeps - 1)
			continue;
		else
		{
			for (size_t i{ 0 }; i < gridCount_x; i++)
			{
				for (size_t j{ 0 }; j < gridCount_y; j++)
				{
					cell& thisCell = cells[i + j * gridCount_x];
					thisCell.p /= substeps;
				}
			}
		}
	}
	//printf("pressure at depth %0.5f: %0.2f\n", cells[20].pos.y, cells[gridCount_x + 20].p); // pressure sensor at the bottom of the container
}


void Fluid::AdvectVelocityParallel(double dt)
{
	std::for_each(std::execution::par_unseq, IterHeight.begin(), IterHeight.end(), [this, &dt](uint32_t j)
	{
		std::for_each(std::execution::par_unseq, IterWidth.begin(), IterWidth.end(), [this, &dt, &j](uint32_t i)
		{
			cell& thisCell = cells[i + j * gridCount_x];
			if (thisCell.s == 0)  // Skip boundary cells
			{
				UpdateCellColor(i + j * gridCount_x, vec3(thisCell.m));
				return;
			}
			
			//float avgU{ 0.0f };

			const float& avgV = (thisCell.v + cells[thisCell.id_left].v + cells[thisCell.id_up].v + cells[thisCell.id_up - 1].v) / 4.f;
			const vec2& samplePos{ 
				std::fmin(std::fmax(thisCell.pos.x - thisCell.u * dt, 0.f), worldSize_x), 
				std::fmin(std::fmax(thisCell.pos.y - avgV * dt, 0.f), worldSize_y) };

			const float& sampledVelX = SampleVelocity(samplePos, 0);
			const float& sampledVelY = SampleVelocity(samplePos, 1);
			thisCell.m = SampleDensity(samplePos);
			thisCell.u = sampledVelX;
			thisCell.v = sampledVelY;
			
			UpdateCellColor(i + j * gridCount_x, vec3(thisCell.m));
		});
	});
}

void Fluid::AdvectVelocity(double dt)
{
	std::for_each(std::execution::par_unseq, IterHeight.begin(), IterHeight.end(), [this, &dt](uint32_t j)
	{
		std::for_each(std::execution::par_unseq, IterWidth.begin(), IterWidth.end(), [this, &dt, &j](uint32_t i)
		{
			cell* const thisCell = &cells[i + j * gridCount_x];
			if (thisCell->s == 1 && cells[thisCell->id_left].s == 1 && i < gridCount_x - 2)
			{
				avgV = (thisCell->v + cells[thisCell->id_left].v + cells[thisCell->id_up].v + cells[i - 1 + (j + 1) * gridCount_x].v) / 4.0f;

				const vec2& samplePos{ thisCell->pos.x - gridSize / 2.0f - dt * thisCell->u, thisCell->pos.y - dt * avgV };
				thisCell->newU = SampleVelocity(samplePos, 0);
			}
			if (thisCell->s == 1 && cells[thisCell->id_down].s == 1 && j < gridCount_y - 2)
			{
				avgU = (thisCell->u + cells[thisCell->id_right].u + cells[thisCell->id_down].u + cells[i + 1 + (j - 1) * gridCount_x].u) / 4.0f;

				const vec2& samplePos{ thisCell->pos.x - dt * avgU, thisCell->pos.y - gridSize / 2.0f - dt * thisCell->v };
				thisCell->newV = SampleVelocity(samplePos, 1);
			}
			if (thisCell->s == 1) {
				avgU = (thisCell->u + cells[thisCell->id_right].u) / 2.0f;
				avgV = (thisCell->v + cells[thisCell->id_up].v) / 2.0f;

				const vec2& samplePos{ thisCell->pos.x - avgU * dt, thisCell->pos.y - avgV * dt };
				thisCell->newM = SampleDensity(samplePos);
			}
		});
	});

	std::for_each(std::execution::par_unseq, IterHeight.begin(), IterHeight.end(), [this](uint32_t j)
	{
		std::for_each(std::execution::par_unseq, IterWidth.begin(), IterWidth.end(), [this, &j](uint32_t i)
		{
			const int& idx = i + j * gridCount_x;
			cell& thisCell = cells[idx];
					
			if (idx == inputIdx)
			{
				thisCell.u = thisCell.newU + inputU;
				thisCell.v = thisCell.newV + inputV;
				thisCell.m = thisCell.newM + inputM;
				//printf("m = %f\n", thisCell.m);
			}
			else
			{
				thisCell.u = thisCell.newU;
				thisCell.v = thisCell.newV;
				thisCell.m = thisCell.newM;
			}
					
			//UpdateCellColor(idx, vec3(thisCell.u, thisCell.v, 0)); // velocity field
			UpdateCellColor(idx, vec3(thisCell.m)); // smoke field
		});
	});
}

float Fluid::SampleVelocity(const vec2 &samplePos, int type)
{
	const int sampleCelli = std::clamp(int(samplePos.x / worldSize_x * gridCount_x), 0, gridCount_x - 1);
	const int sampleCellj = std::clamp(int(samplePos.y / worldSize_y * gridCount_y), 0, gridCount_y - 1);

	const int& thisIdx = sampleCelli + sampleCellj * gridCount_x;
	const cell* sampleCell = &cells[thisIdx];

	int id0, id1, id2, id3;
	
	if (type == 0)
	{
		// sampling u

		if (sampleCell->s == 0)
			return sampleCell->u;

		if (sampleCellj == gridCount_y - 1)
			return cells[sampleCell->id_down].u;

		if (samplePos.y > cells[sampleCelli + sampleCellj * gridCount_x].pos.y)
		{
			id0 = thisIdx;
			id1 = sampleCelli + 1 + sampleCellj * gridCount_x;
			id2 = sampleCelli + (sampleCellj + 1) * gridCount_x;
			id3 = sampleCelli + 1 + (sampleCellj + 1) * gridCount_x;
		}
		else
		{
			id0 = sampleCelli + (sampleCellj - 1) * gridCount_x;
			id1 = sampleCelli + 1 + (sampleCellj - 1) * gridCount_x;
			id2 = thisIdx;
			id3 = sampleCelli + 1 + sampleCellj * gridCount_x;
		}
		const cell& c0 = cells[id0];
		const cell& c1 = cells[id1];
		const cell& c2 = cells[id2];
		const cell& c3 = cells[id3];

		const float& x_u = samplePos.x - (c0.pos.x - gridSize / 2.0f);
		const float& y_u = samplePos.y - c0.pos.y;

		const float& w00_u = 1 - x_u / gridSize;
		const float& w01_u = x_u / gridSize;
		const float& w10_u = 1 - y_u / gridSize;
		const float& w11_u = y_u / gridSize;

		return w00_u * w10_u * c0.u + w01_u * w10_u * c1.u + w00_u * w11_u * c2.u + w01_u * w11_u * c3.u;
	}
	else if (type == 1)
	{
		// sampling v

		if (sampleCell->s == 0)
			return sampleCell->v;

		if (sampleCellj == gridCount_y - 1)
			return cells[sampleCell->id_down].v;

		if (samplePos.x < sampleCell->pos.x)
		{
			id0 = sampleCelli - 1 + sampleCellj * gridCount_x;
			id1 = thisIdx;
			id2 = sampleCelli - 1 + (sampleCellj + 1) * gridCount_x;
			id3 = sampleCelli + (sampleCellj + 1) * gridCount_x;
		}
		else
		{
			id0 = thisIdx;
			id1 = sampleCelli + 1 + sampleCellj * gridCount_x;
			id2 = sampleCelli + (sampleCellj + 1) * gridCount_x;
			id3 = sampleCelli + 1 + (sampleCellj + 1) * gridCount_x;
		}

		const cell& c0 = cells[id0];
		const cell& c1 = cells[id1];
		const cell& c2 = cells[id2];
		const cell& c3 = cells[id3];

		const float x = samplePos.x - c0.pos.x;
		const float y = samplePos.y - (c0.pos.y - gridSize / 2.0f);

		const float w00 = 1 - x / gridSize;
		const float w01 = x / gridSize;
		const float w10 = 1 - y / gridSize;
		const float w11 = y / gridSize;

		return w00 * w10 * c0.v + w01 * w10 * c1.v + w00 * w11 * c2.v + w01 * w11 * c3.v;
	}
}

float Fluid::SampleDensity(const vec2& samplePos)
{
	const int sampleCelli = std::clamp(int(samplePos.x / worldSize_x * gridCount_x), 0, gridCount_x - 1);
	const int sampleCellj = std::clamp(int(samplePos.y / worldSize_y * gridCount_y), 0, gridCount_y - 1);

	const int& thisIdx = sampleCelli + sampleCellj * gridCount_x;
	cell* sampleCell = &cells[thisIdx];

	if (sampleCell->s == 0)
	{
		return 0;
	}

	int id0 = sampleCelli - 1 + (sampleCellj - 1) * gridCount_x;
	int id1 = sampleCelli - 1 + sampleCellj * gridCount_x;
	int id2 = sampleCelli - 1 + (sampleCellj + 1) * gridCount_x;
	int id3 = sampleCelli + (sampleCellj + 1) * gridCount_x;

	if (samplePos.x <= sampleCell->pos.x && samplePos.y >= sampleCell->pos.y)
	{
		id0 = sampleCelli - 1 + sampleCellj * gridCount_x;
		id1 = thisIdx;
		id2 = sampleCelli - 1 + (sampleCellj + 1) * gridCount_x;
		id3 = sampleCelli + (sampleCellj + 1) * gridCount_x;
	}
	else if (samplePos.x <= sampleCell->pos.x && samplePos.y < sampleCell->pos.y)
	{
		id0 = sampleCelli - 1 + (sampleCellj - 1) * gridCount_x;
		id1 = sampleCelli + (sampleCellj - 1) * gridCount_x;
		id2 = sampleCelli - 1 + sampleCellj * gridCount_x;
		id3 = thisIdx;
	}
	else if (samplePos.x > sampleCell->pos.x && samplePos.y >= sampleCell->pos.y)
	{
		id0 = thisIdx;
		id1 = sampleCelli + 1 + sampleCellj * gridCount_x;
		id2 = sampleCelli + (sampleCellj + 1) * gridCount_x;
		id3 = sampleCelli + 1 + (sampleCellj + 1) * gridCount_x;
	}
	else if (samplePos.x > sampleCell->pos.x && samplePos.y < sampleCell->pos.y)
	{
		id0 = sampleCelli + (sampleCellj - 1) * gridCount_x;
		id1 = sampleCelli + 1 + (sampleCellj - 1) * gridCount_x;
		id2 = thisIdx;
		id3 = sampleCelli + 1 + sampleCellj * gridCount_x;
	}
	else if (samplePos.x == sampleCell->pos.x && samplePos.y == sampleCell->pos.y)
	{
		return sampleCell->m;
	}

	const cell& c0 = cells[id0];
	const cell& c1 = cells[id1];
	const cell& c2 = cells[id2];
	const cell& c3 = cells[id3];

	const float& x = samplePos.x - c0.pos.x;
	const float& y = samplePos.y - c0.pos.y;

	const float& w00 = 1 - x / gridSize;
	const float& w01 = x / gridSize;
	const float& w10 = 1 - y / gridSize;
	const float& w11 = y / gridSize;

	return w00 * w10 * c0.m + w01 * w10 * c1.m + w00 * w11 * c2.m + w01 * w11 * c3.m;
}

void Fluid::AdvectSmoke(double dt) {
	
	vec2 samplePos{ vec2(0.0f, 0.0f) };

	std::for_each(std::execution::par_unseq, IterHeight.begin(), IterHeight.end(), [this, &dt](uint32_t j)
	{
		std::for_each(std::execution::par_unseq, IterWidth.begin(), IterWidth.end(), [this, &dt, &j](uint32_t i)
			{
				cell* const thisCell = &cells[i + j * gridCount_x];
				if (thisCell->s == 0)
					return;
				const float& avgU = (thisCell->u + cells[thisCell->id_right].u) / 2.0f;
				const float& avgV = (thisCell->v + cells[thisCell->id_right].v) / 2.0f;

				const float& samplePosX = thisCell->pos.x - avgU * dt;
				const float& samplePosY = thisCell->pos.y - avgV * dt;

				const int& sampleCelli = std::clamp(int(samplePosX / worldSize_x * gridCount_x), 0, gridCount_x - 1);
				const int& sampleCellj = std::clamp(int(samplePosY / worldSize_y * gridCount_y), 0, gridCount_y - 1);

				thisCell->newM = cells[sampleCelli + sampleCellj * gridCount_x].m;
				
			});
	});
	std::for_each(std::execution::par_unseq, IterHeight.begin(), IterHeight.end(), [this, &dt](uint32_t j)
	{
		std::for_each(std::execution::par_unseq, IterWidth.begin(), IterWidth.end(), [this, &dt, &j](uint32_t i)
		{
			cell& thisCell = cells[i + j * gridCount_x];
			if (thisCell.s != 0)
			{
				thisCell.m = thisCell.newM;
			}
		});
	});
}

void Fluid::Extrapolate() 
{
	for (int i = 0; i <= gridCount_x - 1; i++)
	{
		for (int j = 0; j <= gridCount_y - 1; j++)
		{
			cell* const thisCell = &cells[i + j * gridCount_x];

			if (j==0 || j == gridCount_y - 1)
			{
				thisCell->v = 0;
				thisCell->u = 0;
				continue;
			}
			if (i==0)
			{
				thisCell->u = InletVel[0];
				thisCell->v = InletVel[1];
				continue;
			}
			else if (i == gridCount_x - 1)
			{
				const cell& L = cells[(i - 1) + j * gridCount_x];
				thisCell->u = L.u;
				thisCell->v = L.v;
				thisCell->p = L.p;
			}
		}
	}
}

void Fluid::Simulate(double dt) {
	ndt = dt / substeps;

	/*std::for_each(std::execution::par_unseq, IterIndices.begin(), IterIndices.end(), [this, &dt](uint32_t i)
	{
		cell& thisCell = cells[i];
		if (thisCell.s != 0)
		{
			thisCell.v += -9.83 * dt;
		}
	});*/
	
	ProjectParallel(ndt);
	//Extrapolate();
	AdvectVelocity(dt);
	//AdvectSmoke(dt);
	simulationTime += dt;
}

void Fluid::AddVelocity(const float& posX, const float& posY, const float& u, const float& v)
{
	static float amp{10};

	inputIdx = std::clamp((int)(posX / gridSize), 1, gridCount_x - 2) + std::clamp((int)(posY / gridSize), 1, gridCount_y - 2) * gridCount_x;

	cell& thisCell = cells[inputIdx];
	inputU = u * amp;
	inputV = v * amp;
	inputM = std::abs(u) + std::abs(v) * amp*100;
}