/*
	Copyright 2011-2018 Daniel S. Buckstein

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

/*
	animal3D SDK: Minimal 3D Animation Framework
	By Daniel S. Buckstein
	
	a3_PhysicsWorld.c/.cpp
	Physics world function implementations.
*/


#include "a3_PhysicsWorld.h"

// include utilities on an as-needed basis
#include "animal3D/a3utility/a3_Timer.h"


// external
#include <stdio.h>
#include <string.h>


//-----------------------------------------------------------------------------

// internal utility for initializing and terminating physics world
void a3physicsInitialize_internal(a3_PhysicsWorld *world)
{
	// e.g. reset all particles and/or rigid bodies
	memset(world->particle, 0, sizeof(world->particle));
	world->particlesActive = 18;
	world->t = 0.0;


	// fixed positions
	a3real3Set(world->fixedGround->position.v, 
		a3realZero, a3realZero, a3realZero);
	a3real3Set(world->fixedSpringAnchor->position.v, 
		-10.0f, -10.0f, 10.0f);
	a3real3Set(world->fixedSpringAnchorOther->position.v, 
		-10.0f, a3realZero, 10.0f);
	a3real3Set(world->fixedRampStationary->position.v, 
		a3realZero, 10.0f, 3.0f);
	a3real3Set(world->fixedRampTilting->position.v, 
		15.0f, 10.0f, 3.0f);
	a3real3Set(world->fixedRampRotating->position.v, 
		-15.0f, 10.0f, 3.0f);

	// moving
	a3real3Set(world->particleVaccumBowly->position.v, 
		a3realZero, -16.0f, 5.0f);
	a3real3Set(world->particleNormalBowly->position.v, 
		4.0f, -16.0f, 5.0f);
	a3real3Set(world->particleVaccumFeathery->position.v, 
		a3realZero, -13.0f, 5.0f);
	a3real3Set(world->particleNormalFeathery->position.v, 
		4.0f, -13.0f, 5.0f);
	a3real3Set(world->particleDraggy->position.v, 
		4.0f, a3realZero, a3realZero);
	a3real3Set(world->particleBouncy->position.v, 
		4.0f, 4.0f, 10.0f);
	a3real3Set(world->particleLessBouncy->position.v, 
		8.0f, 4.0f, 10.0f);

	world->particleSpringy->position = world->fixedSpringAnchor->position;
	world->particleSpringy->position.z -= a3realTwo;
	world->particleOtherSpringy->position = world->fixedSpringAnchorOther->position;
	world->particleOtherSpringy->position.z -= a3realTwo;
	world->particleRampy->position = world->fixedRampStationary->position;
	world->particleRampy->position.z += a3realOne;
	world->particleTilty->position = world->fixedRampTilting->position;
	world->particleTilty->position.z += a3realOne;
	world->particleRotaty->position = world->fixedRampRotating->position;
	world->particleRotaty->position.z += a3realOne;


	// surface normals
	world->groundNormal = a3zVec3;
	world->rampStationaryNormal.z = 0.8f;
	world->rampStationaryNormal.y = -0.6f;
	world->rampRotatingNormal = world->rampStationaryNormal;
	world->rampTiltingNormal = world->rampStationaryNormal;

	
	// other values
	world->dragCoeffBowlingBall = 0.47f;
	world->dragCoeffFeather = 2.0f;
	world->dragCoeffSpring = .95f;

	// let's say all objects are made of steel for now, easier
	// we can use constants, otherwise this would be calculated
	// fun link: http://www.roymech.co.uk/Useful_Tables/Tribology/co_of_frict.htm#method
	world->frictionCoeffStaticGround = 0.74f;
	world->frictionCoeffKineticGround = 0.42f;
	world->frictionCoeffStaticRamp = 0.74f;
	world->frictionCoeffKineticRamp = 0.42f;

	world->areaBowlingBall = a3realPi * 0.01f;	// pi r^2, r = 0.1
	world->areaFeather = 0.01f;	// r^2, r = 0.1
	world->areaSpringy = a3realPi * .01f;

	world->airDensity = 1.2f;
	world->fixedLength = 1.0f;
	world->springCoeff = 2.0f;
	// ****TO-DO: 
	//	- other initializations
	a3particleSetMass(world->particleVaccumBowly, (a3real)4.0f);
	a3particleSetMass(world->particleNormalBowly, (a3real)4.0f);
	a3particleSetMass(world->particleVaccumFeathery, (a3real)0.001f);
	a3particleSetMass(world->particleNormalFeathery, (a3real)0.001f);
	a3particleSetMass(world->particleSpringy, (a3real)1.0f);

	for (unsigned int i = 0; i < world->particlesActive; i++)
	{
		a3quaternionCreateIdentity(world->particle[i].rotation.v);
		a3quaternionCreateIdentity(world->particle[i].velocity_a.v);
		a3quaternionCreateIdentity(world->particle[i].acceleration_a.v);
	}
}

void a3physicsTerminate_internal(a3_PhysicsWorld *world)
{
	// any term tasks here
}


//-----------------------------------------------------------------------------

// physics simulation
long a3physicsThread(a3_PhysicsWorld *world)
{
	// physics simulation timer
	a3_Timer physicsTimer[1] = { 0 };

	// second counter for physics (debugging)
	unsigned int currSecond = 0, prevSecond = 0;

	// create world
	a3physicsInitialize_internal(world);

	// start timer
	// rate should be set before beginning thread
	a3timerSet(physicsTimer, world->rate);
	a3timerStart(physicsTimer);

	// if lock is negative, terminate
	while (world->lock >= 0)
	{
		if (a3timerUpdate(physicsTimer))
		{
			// update timer ticked, do the thing
			a3physicsUpdate(world, physicsTimer->previousTick);

			// debug display time in seconds
			currSecond = (unsigned int)(physicsTimer->totalTime);
			if (currSecond > prevSecond)
			{
				prevSecond = currSecond;
				printf("\n physics time: %.4lf;  ticks: %llu \n     ups avg: %.4lf;  dt avg: %.4lf",
					physicsTimer->totalTime,
					physicsTimer->ticks,
					(double)physicsTimer->ticks / physicsTimer->totalTime,
					physicsTimer->totalTime / (double)physicsTimer->ticks
				);
			}
		}
	}

	// terminate world
	a3physicsTerminate_internal(world);
	return 0;
}

void a3physicsUpdate(a3_PhysicsWorld *world, double dt)
{
	// copy of state to edit before writing to world
	a3_PhysicsWorldState state[1] = { 0 };

	// time as real
	const a3real t_r = (a3real)(world->t);
	const a3real dt_r = (a3real)(dt);


	const a3real t_trigParam = a3trigValid_sinr(t_r);


	unsigned int i;


	// ****TO-DO: 
	//	- apply forces
	//	- convert forces to acceleration
	{
		a3vec3 tmp;
		a3forceGravity(tmp.v, a3zVec3.v, world->particleVaccumBowly->mass);
		world->particleVaccumBowly->force = tmp;

		a3forceGravity(tmp.v, a3zVec3.v, world->particleNormalBowly->mass);
		world->particleNormalBowly->force = tmp;
		a3real3Add(world->particleNormalBowly->force.v, 
			a3forceDrag(tmp.v, world->particleNormalBowly->velocity.v, a3zeroVec3.v, world->airDensity, world->areaBowlingBall, world->dragCoeffBowlingBall));

		a3forceGravity(tmp.v, a3zVec3.v, world->particleNormalFeathery->mass);
		world->particleNormalFeathery->force = tmp;
		a3real3Add(world->particleNormalFeathery->force.v,
			a3forceDrag(tmp.v, world->particleNormalFeathery->velocity.v, a3zeroVec3.v, world->airDensity, world->areaFeather, world->dragCoeffFeather));

		a3forceGravity(tmp.v, a3zVec3.v, world->particleVaccumFeathery->mass);
		world->particleVaccumFeathery->force = tmp;

		a3forceGravity(tmp.v, a3zVec3.v, world->particleSpringy->mass);
		a3real3Add(world->particleSpringy->force.v,
			a3forceDrag(tmp.v, world->particleSpringy->velocity.v, a3zeroVec3.v, world->airDensity, world->areaSpringy, world->dragCoeffSpring));

		a3forceSpring(tmp.v, world->particleSpringy->position.v, world->fixedSpringAnchor->position.v, world->fixedLength, world->springCoeff);
		a3real3Add(world->particleSpringy->force.v, tmp.v);

		a3forceDampingLinear(tmp.v, world->particleSpringy->velocity.v, 0.2f);
		a3real3Add(world->particleSpringy->force.v, tmp.v);
	}

	// ****TO-DO: 
	//	- integrate using choice algorithm
	for (i = 0; i < world->particlesActive; ++i)
	{
		//integrate
		a3particleIntegrateEulerExplicit(world->particle + i, dt_r);

		//convert force
		a3real3ProductS(world->particle[i].acceleration.v, world->particle[i].force.v, world->particle[i].massInverse);
		
		//reset force
		a3particleResetForce(world->particle + i);
	}

	// tilt planes
	world->rampTiltingNormal.y = a3sinr(t_trigParam) * 0.6f;
	a3real3Normalize(world->rampTiltingNormal.v);
	world->rampRotatingNormal.y = a3sinr(t_trigParam) * 0.6f;
	world->rampRotatingNormal.x = a3cosr(t_trigParam) * 0.6f;
	a3real3Normalize(world->rampRotatingNormal.v);


	// accumulate time
	world->t += dt;


	// write to state
	for (i = 0; i < world->particlesActive; ++i)
	{
		state->position[i].xyz = world->particle[i].position;
	}

	for (i = 0; i < world->particlesActive; ++i)
	{
		state->rotation[i] = world->particle[i].rotation;
	}

	// write operation is locked
	if (a3physicsLockWorld(world) > 0)
	{
		// copy state to world
		*world->state = *state;
		a3physicsUnlockWorld(world);
	}
}


// get thread ID
#ifdef _WIN32
#include <Windows.h>
int threadID()
{
	return GetCurrentThreadId();
}
#else
#include <sys/types.h>
int threadID()
{
	return gettid();
}
#endif	// _WIN32

// mutex
extern inline int a3physicsLockWorld(a3_PhysicsWorld *world)
{
	// wait for lock to be released, then set it
	while (world->lock > 0);
	if (world->lock == 0)
	{
		world->lock = threadID();
		return world->lock;
	}
	return -1;
}

extern inline int a3physicsUnlockWorld(a3_PhysicsWorld *world)
{
	const int ret = world->lock;
	if (ret == threadID())
	{
		world->lock = 0;
		return ret;
	}
	return -1;
}


//-----------------------------------------------------------------------------

// initialize world state
int a3physicsWorldStateReset(a3_PhysicsWorldState *worldState)
{
	unsigned int i;
	if (worldState)
	{
		for (i = 0; i < physicsMaxCount_rigidbody; ++i)
			worldState->position[i] = a3wVec4;
		return i;
	}
	return -1;
}


//-----------------------------------------------------------------------------
