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
	
	a3_PhysicsWorld.h
	Interface for physics world.
*/

#ifndef __ANIMAL3D_PHYSICSWORLD_H
#define __ANIMAL3D_PHYSICSWORLD_H


//-----------------------------------------------------------------------------
// physics includes

#include "a3_RigidBody.h"


//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#else	// !__cplusplus
	typedef struct a3_PhysicsWorld					a3_PhysicsWorld;
	typedef struct a3_PhysicsWorldState				a3_PhysicsWorldState;
#endif	// __cplusplus


//-----------------------------------------------------------------------------

	// counters
	enum a3_PhysicsWorldMaxCount
	{
		physicsMaxCount_rigidbody = 32,
	};


//-----------------------------------------------------------------------------

	// state of a physics world: things that can be used for graphics ONLY
	//	- position and rotation... why not scale? RIGID bodies don't scale
	struct a3_PhysicsWorldState
	{
		a3vec4 position[physicsMaxCount_rigidbody];
		a3vec4 rotation[physicsMaxCount_rigidbody];
	};


//-----------------------------------------------------------------------------

	// persistent physics world data structure
	struct a3_PhysicsWorld
	{
		//---------------------------------------------------------------------
		// SIMPLE MUTEX LOCK
		int lock;


		//---------------------------------------------------------------------
		// timer rate
		double rate;


		//---------------------------------------------------------------------
		// the state to store all of the things that need to go to graphics
		a3_PhysicsWorldState state[1];


		//---------------------------------------------------------------------
		// general variables pertinent to the state

		// e.g. set of rigid bodies
		
		// e.g. set of particles

		union {
			a3_RigidBody rigidbody[physicsMaxCount_rigidbody];
			struct {
				a3_RigidBody
					fixedGround[1],
					fixedSpringAnchor[1],
					fixedSpringAnchorOther[1],
					fixedRampStationary[1],
					fixedRampTilting[1],
					fixedRampRotating[1],

					particleVaccumBowly[1],
					particleNormalBowly[1],
					particleVaccumFeathery[1],
					particleNormalFeathery[1],
					particleDraggy[1],
					particleBouncy[1],
					particleLessBouncy[1],
					particleSpringy[1],
					particleOtherSpringy[1],
					particleRampy[1],
					particleTilty[1],
					particleRotaty[1];
			};
		};
		unsigned int particlesActive;
		double t;

		// not particle-based: 
		// surface normals
		a3vec3 groundNormal, rampStationaryNormal, rampTiltingNormal, rampRotatingNormal;

		// drag coefficients for falling and sliding objects
		a3real dragCoeffBowlingBall, dragCoeffFeather, dragCoeffSpring;
		a3real fixedLength;
		// friction coefficients for surfaces
		a3real frictionCoeffStaticGround, frictionCoeffKineticGround, frictionCoeffStaticRamp, frictionCoeffKineticRamp;

		// cross-sectional areas for falling objects
		a3real areaBowlingBall, areaFeather, areaSpringy;

		a3real springCoeff;

		// restitution for bouncy objects
		a3real restitutionBouncy, restitutionLessBouncy;

		// air density (for drag)
		a3real airDensity;


		//---------------------------------------------------------------------
	};

	
//-----------------------------------------------------------------------------

	// threaded simulation
	long a3physicsThread(a3_PhysicsWorld *world);
	void a3physicsUpdate(a3_PhysicsWorld *world, double dt);

	// mutex handling
	inline int a3physicsLockWorld(a3_PhysicsWorld *world);
	inline int a3physicsUnlockWorld(a3_PhysicsWorld *world);


//-----------------------------------------------------------------------------

	// world utilities
	int a3physicsWorldStateReset(a3_PhysicsWorldState *worldState);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// !__ANIMAL3D_PHYSICSWORLD_H