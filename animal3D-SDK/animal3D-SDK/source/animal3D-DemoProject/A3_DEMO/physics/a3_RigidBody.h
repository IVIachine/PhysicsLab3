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

a3_RigidBody.h
Rigid body for physics simulation.
*/

/*
	Tyler Chermely 0967813
	EGP-425-01
	Lab 3
	3/3/2018

	I certify that this work is
	entirely my own. The assessor of this project may reproduce this project
	and provide copies to other academic staff, and/or communicate a copy of
	this project to a plagiarism-checking service, which may retain a copy of the
	project on its database.
*/

#ifndef __ANIMAL3D_RIGIDBODY_H
#define __ANIMAL3D_RIGIDBODY_H


//-----------------------------------------------------------------------------

// math library
#include "animal3D/a3math/A3DM.h"
#include "a3_Quaternion.h"

//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#else	// !__cplusplus
typedef struct a3_RigidBody		a3_RigidBody;
typedef struct a3_Particle		a3_Particle;
#endif	// __cplusplus


//-----------------------------------------------------------------------------

// particle for motion
struct a3_Particle
{
	a3vec3 position;		// x
	a3vec3 velocity;		// v = x' = dx/dt
	a3vec3 acceleration;	// a = v' = x" = dv/dt = d2x/dt2

	a3vec3 force;
	a3vec4 torque;
	a3real mass, massInverse;

	// ****TO-DO: 
	//	- add rotation and its derivatives
	a3vec4 rotation; //q
	a3vec4 velocity_a; //w(omega) dq/dt = (w)q/2
	a3vec4 acceleration_a; //alpha dw/dt = a
};


// rb
//	struct a3_RigidBody
//	{
//		a3vec3 position;
//	};


//-----------------------------------------------------------------------------

// named Euler methods (described below)
inline void a3particleIntegrateEulerExplicit(a3_Particle *p, const a3real dt);
inline void a3particleIntegrateEulerSemiImplicit(a3_Particle *p, const a3real dt);
inline void a3particleIntegrateEulerKinematic(a3_Particle *p, const a3real dt);


//-----------------------------------------------------------------------------

// set mass
inline int a3particleSetMass(a3_Particle *p, const a3real mass);

// reset force
inline int a3particleResetForce(a3_Particle *p);

// reset torque
inline int a3particleResetTorque(a3_Particle *p);

// check if particle is moving
inline int a3particleIsMoving(const a3_Particle *p);

// check if particle is rotating
inline int a3ParticleIsRotating(const a3_Particle *p);

// calculate critical damping for a particle
inline a3real a3calcCriticalDamping(const a3real mass, const a3real springCoeff);


// calculate force of gravity in freefall (using earth gravity)
inline a3real3r a3forceGravity(a3real3p f_out, const a3real3p unitUpward, const a3real mass);

// calculate normal force against plane
inline a3real3r a3forceNormal(a3real3p f_out, const a3real3p f_gravity, const a3real3p unitNormal);

// calculate sliding force on a plane
inline a3real3r a3forceSliding(a3real3p f_out, const a3real3p f_gravity, const a3real3p f_normal);

// calculate static friction (if particle is still)
inline a3real3r a3forceFrictionStatic(a3real3p f_out, const a3real3p f_normal, const a3real3p f_opposing, const a3real frictionCoeffStatic);

// calculate kinetic friction (if particle is moving)
inline a3real3r a3forceFrictionKinetic(a3real3p f_out, const a3real3p f_normal, const a3real3p particleVelocity, const a3real frictionCoeffKinetic);

// calculate drag force through fluid
inline a3real3r a3forceDrag(a3real3p f_out, const a3real3p particleVelocity, const a3real3p fluidVelocity, const a3real fluidDensity, const a3real objectArea, const a3real objectDragCoeff);

// calculate spring force using Hooke's law
inline a3real3r a3forceSpring(a3real3p f_out, const a3real3p particlePosition, const a3real3p anchorPosition, const a3real restingLength, const a3real springCoeff);

// calculate general damping force
inline a3real3r a3forceDampingLinear(a3real3p f_out, const a3real3p particleVelocity, const a3real dampingCoeff);


//-----------------------------------------------------------------------------

//	integrate first order
inline void a3particleIntegrateFirstOrder(a3_Particle *p, const a3real dt);

//	integrate second order
inline void a3particleIntegrateSecondOrder(a3_Particle *p, const a3real dt);

//	integrate displacement
inline void a3particleIntegrateDisplacement(a3_Particle *p, const a3real dt);

//	integrate Verlet
inline void a3particleIntegrateVerlet(a3_Particle *p, const a3real dt);

//	integrate Adams-Bashforth (2nd step)
inline void a3particleIntegrateAdamsBashforth2(a3_Particle *p, const a3real dt);

//	integrate Runge-Kutta (RK4)
inline void a3particleIntegrateRungeKutta4(a3_Particle *p, const a3real dt);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// !__ANIMAL3D_RIGIDBODY_H