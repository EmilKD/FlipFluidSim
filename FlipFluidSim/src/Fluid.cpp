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
	gridCountX{ static_cast<int>(Size_x / gridSize) },
	gridCountY{ static_cast<int>(Size_y / gridSize) },
	worldSizeX{ Size_x }, worldSizeY{ Size_y }
{
	cell initialCell;
	initialCell.m = 0.001f;
	initialCell.u = 0;
	initialCell.v = 0;
	initialCell.s = 1;

	cells.resize(gridCountX * gridCountY, initialCell);

	for (int i{ 0 }; i < gridCountX; i++)
	{
		for (int j{ 0 }; j < gridCountY; j++)
		{
			int idx = i + j * gridCountX;
			cell& thisCell = cells[idx];
			thisCell.pos = vec3(
				worldSizeX * (i + 0.5f) / gridCountX,
				worldSizeY * (j + 0.5f) / gridCountY,
				0.0f
			);

			// Boundary conditions
			// Default inward neighbors
			int left = (i > 0) ? idx - 1 : idx;
			int right = (i < gridCountX - 1) ? idx + 1 : idx;
			int down = (j > 0) ? idx - gridCountX : idx;
			int up = (j < gridCountY - 1) ? idx + gridCountX : idx;

			// LEFT = INLET
			if (i == 0) {
				/*thisCell.s = 1;
				thisCell.u = InletVel[0];
				thisCell.v = InletVel[1];
				if (j < (gridCountY / 2 + 20) && j >(gridCountY / 2 - 20))
					thisCell.m = 1;*/
				//left = idx;         
				//right = idx + 1;   
			}

			// RIGHT = OUTLET (OPEN)
			if (i == gridCountX - 1) {
				//right = idx;        
				//eft = idx - 1;
			}

			if (i == 0 || j == 0 || i == gridCountX - 1 || j == gridCountY - 1)
				thisCell.s = 0;

			thisCell.id_left = left;
			thisCell.id_right = right;
			thisCell.id_up = up;
			thisCell.id_down = down;
		}
	}

	GenPosBuffer();
	GenColorBuffer();

	IterWidth.resize(gridCountX);
	for (size_t i = 0; i < gridCountX; i++)
		IterWidth.at(i) = i;

	IterHeight.resize(gridCountY);
	for (size_t i = 0; i < gridCountY; i++)
		IterHeight.at(i) = i;

	IterSubSteps.resize(substeps);
	for (size_t i = 0; i < substeps; i++)
		IterSubSteps.at(i) = i;

	const int spacing = 1;
	for (size_t s = 0; s < spacing; s++)
		for (size_t p = 0; p < spacing; p++)
			for (size_t j = s; j < gridCountY; j += spacing)
				for (size_t i = p; i < gridCountX; i += spacing)
					IterIndices.push_back(i + j * (gridCountX));
				
	if ((gridCountX) * (gridCountY) != IterIndices.size())
	{
		printf("Iterator size: %i\n", IterIndices.size());
		printf("correct size: %i\n", (gridCountX - 2) * (gridCountY - 2));
		throw std::invalid_argument("iterator construction failed");
	}

	// Tiling
	const int tileSize = 16;
	const int tileCountX = (gridCountX + tileSize - 1) / tileSize;
	const int tileCountY = (gridCountY + tileSize - 1) / tileSize;
	
	tileIndices.resize(tileCountX * tileCountY);

	for (size_t j = 0; j < tileCountY; j++)
	{
		for (size_t i = 0; i < tileCountX; i++)
		{
			for (size_t y = 0; y < tileSize; y++)
			{
				const int gy = j * tileSize + y;
				if (gy >= gridCountY) break;

				for (size_t x = 0; x < tileSize; x++)
				{
					const int gx = i * tileSize + x;
					if (gx >= gridCountX) break;

					tileIndices[i + j * tileCountX].push_back(x + i * tileSize + (y + j * tileSize) * gridCountX);
				}
			}
		}
	}

	IterTiles.resize(tileCountX * tileCountY);
	for (size_t i = 0; i < tileCountX * tileCountY; i++)
		IterTiles.at(i) = i;
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
	const float& cp = this->density * gridSize / dt;

	for (size_t n{ 0 }; n < substeps; n++)
	{
		std::for_each(std::execution::par_unseq, IterIndices.begin(), IterIndices.end(), [this, &cp](uint32_t i)
		{
			cell& thisCell = cells[i];

			if (thisCell.s == 0) {
				thisCell.p = 0;
				thisCell.u = 0;
				thisCell.v = 0;
				return;
			}

			const double& d = (thisCell.u - cells[thisCell.id_right].u + thisCell.v - cells[thisCell.id_up].v) * overRelaxation;
			const int& s = cells[thisCell.id_down].s + cells[thisCell.id_up].s + cells[thisCell.id_left].s + cells[thisCell.id_right].s;

			thisCell.p += d / s * cp;
			thisCell.u -= d / s * cells[thisCell.id_left].s;
			cells[thisCell.id_right].u += d / s * cells[thisCell.id_right].s;
			thisCell.v -= d / s * cells[thisCell.id_down].s;
			cells[thisCell.id_up].v += d / s * cells[thisCell.id_up].s;
		});
	}
	std::for_each(std::execution::par_unseq, IterIndices.begin(), IterIndices.end(), [this](uint32_t idx)
	{
		cell& thisCell = cells[idx];
		thisCell.p /= substeps;
	});
}

void Fluid::Project(double dt)
{
	cp = this->density * gridSize / dt;
	for (cell& c : cells)
		c.p = 0;

	for (size_t n{0}; n < substeps; n++)
	{
		for (size_t i{ 0 }; i < gridCountX; i++)
		{
			for (size_t j{ 0 }; j < gridCountY; j++)
			{
				cell& thisCell = cells[i + j * gridCountX];

				if (thisCell.s == 0) {
					thisCell.p = 0;
					thisCell.u = 0;
					thisCell.v = 0;
					continue;
				}

				const double& d = (thisCell.u - cells[thisCell.id_right].u + thisCell.v - cells[thisCell.id_up].v) * overRelaxation;
				const int& s = cells[thisCell.id_down].s + cells[thisCell.id_up].s + cells[thisCell.id_left].s + cells[thisCell.id_right].s;

				thisCell.p += d / s * cp;
				thisCell.u -= d / s * cells[thisCell.id_left].s;
				cells[thisCell.id_right].u += d / s * cells[thisCell.id_right].s;
				thisCell.v -= d / s * cells[thisCell.id_down].s;
				cells[thisCell.id_up].v += d / s * cells[thisCell.id_up].s;
			}
		}
	}
	for (size_t i{ 0 }; i < gridCountX; i++)
	{
		for (size_t j{ 0 }; j < gridCountY; j++)
		{
			cell& thisCell = cells[i + j * gridCountX];
			thisCell.p /= substeps;
		}
	}
	//printf("pressure at depth %0.5f: %0.2f\n", cells[20].pos.y, cells[gridCountX + 20].p); // pressure sensor at the bottom of the container
}


void Fluid::AdvectVelocityParallel(double dt)
{
	std::for_each(std::execution::par_unseq, IterIndices.begin(), IterIndices.end(), [this, &dt](uint32_t idx)
		{
			cell& const thisCell = cells[idx];
			if (thisCell.s == 1 && cells[thisCell.id_left].s == 1 && cells[thisCell.id_right].s == 1 && cells[thisCell.id_down].s == 1 && cells[thisCell.id_up].s == 1)
			{
				const float& avgV = (thisCell.v + cells[thisCell.id_left].v + cells[thisCell.id_up].v + cells[cells[thisCell.id_left].id_up].v) / 4.0f;

				const vec2& samplePosU{ thisCell.pos.x - gridSize / 2.0f - dt * thisCell.u, thisCell.pos.y - dt * avgV };
				thisCell.newU = Sample(samplePosU, 0);

				const float& avgU = (thisCell.u + cells[thisCell.id_right].u + cells[thisCell.id_down].u + cells[cells[thisCell.id_right].id_down].u) / 4.0f;

				const vec2& samplePosV{ thisCell.pos.x - dt * avgU, thisCell.pos.y - gridSize / 2.0f - dt * thisCell.v };
				thisCell.newV = Sample(samplePosV, 1);
			}

			const float& avgU = (thisCell.u + cells[thisCell.id_right].u) / 2.0f;
			const float& avgV = (thisCell.v + cells[thisCell.id_up].v) / 2.0f;

			const vec2& samplePos{ thisCell.pos.x - avgU * dt, thisCell.pos.y - avgV * dt };
			thisCell.newM = Sample(samplePos, 2);
		});

	std::for_each(std::execution::par_unseq, IterIndices.begin(), IterIndices.end(), [this, &dt](uint32_t idx)
	{
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
		UpdateCellColor(idx, vec3(0, thisCell.m * 0.6 + glm::abs(thisCell.u * thisCell.v) * 500 * 0.4, thisCell.m * 0.3 + glm::abs(thisCell.u * thisCell.v) * 300 * 0.7) ); // smoke field
	});
}

void Fluid::AdvectVelocity(double dt)
{
	for (int i = 0; i <= gridCountX - 1; i++)
	{
		for (int j = 0; j <= gridCountY - 1; j++)
		{
			const int& idx = i + j * gridCountX;
			cell& const thisCell = cells[idx];

			if (thisCell.s == 1 && cells[thisCell.id_left].s == 1 && cells[thisCell.id_right].s == 1 && cells[thisCell.id_down].s == 1 && cells[thisCell.id_up].s == 1)
			{
				const float& avgV = (thisCell.v + cells[thisCell.id_left].v + cells[thisCell.id_up].v + cells[cells[thisCell.id_left].id_up].v) / 4.0f;

				const vec2& samplePosU{ thisCell.pos.x - gridSize / 2.0f - dt * thisCell.u, thisCell.pos.y - dt * avgV };
				thisCell.newU = Sample(samplePosU, 0);

				const float& avgU = (thisCell.u + cells[thisCell.id_right].u + cells[thisCell.id_down].u + cells[cells[thisCell.id_right].id_down].u) / 4.0f;

				const vec2& samplePosV{ thisCell.pos.x - dt * avgU, thisCell.pos.y - gridSize / 2.0f - dt * thisCell.v };
				thisCell.newV = Sample(samplePosV, 1);
			}

			const float& avgU = (thisCell.u + cells[thisCell.id_right].u) / 2.0f;
			const float& avgV = (thisCell.v + cells[thisCell.id_up].v) / 2.0f;

			const vec2& samplePos{ thisCell.pos.x - avgU * dt, thisCell.pos.y - avgV * dt };
			thisCell.newM = Sample(samplePos, 2);
		}
	}

	for (int i = 0; i <= gridCountX - 1; i++)
	{
		for (int j = 0; j <= gridCountY - 1; j++)
		{
			const int& idx = i + j * gridCountX;
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
		}
	}
}

float Fluid::Sample(const vec2 &samplePos, int type)
{
	const int sampleCelli = std::clamp(int(samplePos.x / worldSizeX * gridCountX), 0, gridCountX - 1);
	const int sampleCellj = std::clamp(int(samplePos.y / worldSizeY * gridCountY), 0, gridCountY - 1);

	const int& thisIdx = sampleCelli + sampleCellj * gridCountX;
	const cell* sampleCell = &cells[thisIdx];

	int id0, id1, id2, id3;
	
	if (type == 0)
	{
		// sampling u

		if (sampleCell->s == 0)
			return sampleCell->u;

		if (sampleCellj == gridCountY - 1)
			return cells[sampleCell->id_down].u;

		if (samplePos.y > cells[sampleCelli + sampleCellj * gridCountX].pos.y)
		{
			id0 = thisIdx;
			id1 = sampleCelli + 1 + sampleCellj * gridCountX;
			id2 = sampleCelli + (sampleCellj + 1) * gridCountX;
			id3 = sampleCelli + 1 + (sampleCellj + 1) * gridCountX;
		}
		else
		{
			id0 = sampleCelli + (sampleCellj - 1) * gridCountX;
			id1 = sampleCelli + 1 + (sampleCellj - 1) * gridCountX;
			id2 = thisIdx;
			id3 = sampleCelli + 1 + sampleCellj * gridCountX;
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

		if (sampleCellj == gridCountY - 1)
			return cells[sampleCell->id_down].v;

		if (samplePos.x < sampleCell->pos.x)
		{
			id0 = sampleCelli - 1 + sampleCellj * gridCountX;
			id1 = thisIdx;
			id2 = sampleCelli - 1 + (sampleCellj + 1) * gridCountX;
			id3 = sampleCelli + (sampleCellj + 1) * gridCountX;
		}
		else
		{
			id0 = thisIdx;
			id1 = sampleCelli + 1 + sampleCellj * gridCountX;
			id2 = sampleCelli + (sampleCellj + 1) * gridCountX;
			id3 = sampleCelli + 1 + (sampleCellj + 1) * gridCountX;
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
	else if (type == 2)
	{
		if (sampleCell->s == 0)
		{
			return 0;
		}

		int id0 = sampleCelli - 1 + (sampleCellj - 1) * gridCountX;
		int id1 = sampleCelli - 1 + sampleCellj * gridCountX;
		int id2 = sampleCelli - 1 + (sampleCellj + 1) * gridCountX;
		int id3 = sampleCelli + (sampleCellj + 1) * gridCountX;

		if (samplePos.x <= sampleCell->pos.x && samplePos.y >= sampleCell->pos.y)
		{
			id0 = sampleCelli - 1 + sampleCellj * gridCountX;
			id1 = thisIdx;
			id2 = sampleCelli - 1 + (sampleCellj + 1) * gridCountX;
			id3 = sampleCelli + (sampleCellj + 1) * gridCountX;
		}
		else if (samplePos.x <= sampleCell->pos.x && samplePos.y < sampleCell->pos.y)
		{
			id0 = sampleCelli - 1 + (sampleCellj - 1) * gridCountX;
			id1 = sampleCelli + (sampleCellj - 1) * gridCountX;
			id2 = sampleCelli - 1 + sampleCellj * gridCountX;
			id3 = thisIdx;
		}
		else if (samplePos.x > sampleCell->pos.x && samplePos.y >= sampleCell->pos.y)
		{
			id0 = thisIdx;
			id1 = sampleCelli + 1 + sampleCellj * gridCountX;
			id2 = sampleCelli + (sampleCellj + 1) * gridCountX;
			id3 = sampleCelli + 1 + (sampleCellj + 1) * gridCountX;
		}
		else if (samplePos.x > sampleCell->pos.x && samplePos.y < sampleCell->pos.y)
		{
			id0 = sampleCelli + (sampleCellj - 1) * gridCountX;
			id1 = sampleCelli + 1 + (sampleCellj - 1) * gridCountX;
			id2 = thisIdx;
			id3 = sampleCelli + 1 + sampleCellj * gridCountX;
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
}

void Fluid::Extrapolate() 
{
	for (int i = 0; i <= gridCountX - 1; i++)
	{
		for (int j = 0; j <= gridCountY - 1; j++)
		{
			cell* const thisCell = &cells[i + j * gridCountX];

			if (j==0 || j == gridCountY - 1)
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
			else if (i == gridCountX - 1)
			{
				const cell& L = cells[(i - 1) + j * gridCountX];
				thisCell->u = L.u;
				thisCell->v = L.v;
				thisCell->p = L.p;
			}
		}
	}
}

void Fluid::Simulate(double dt) {
	ndt = dt / substeps;

	std::for_each(std::execution::par_unseq, IterIndices.begin(), IterIndices.end(), [this, &dt](uint32_t i)
	{
		cell& thisCell = cells[i];
		if (thisCell.s == 0)
		{
			thisCell.v += -9.83 * dt;
		}
	});
	
	ProjectParallel(ndt);
	//Extrapolate();
	AdvectVelocityParallel(dt);
	//AdvectSmoke(dt);
	simulationTime += dt;
}

void Fluid::AddVelocity(const float& posX, const float& posY, const float& u, const float& v)
{
	static float amp{10};

	inputIdx = std::clamp((int)(posX / gridSize), 1, gridCountX - 2) + std::clamp((int)(posY / gridSize), 1, gridCountY - 2) * gridCountX;

	cell& thisCell = cells[inputIdx];
	inputU = u * amp;
	inputV = v * amp;
	inputM = std::abs(u) + std::abs(v) * amp*100;
}