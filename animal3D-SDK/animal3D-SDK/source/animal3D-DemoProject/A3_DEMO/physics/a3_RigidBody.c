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
	
	a3_RigidBody.c/.cpp
	Implementation of rigid body.
*/

#include "a3_RigidBody.h"
#include <stdio.h>

//-----------------------------------------------------------------------------
// integration functions

// named Euler methods (described below)
extern inline void a3particleIntegrateEulerExplicit(a3_Particle *p, const a3real dt)
{
	a3vec3 d;
	a3vec4 r;
	//	x(t+dt) = x(t) + f(t)dt
	//					 f(t) = dx/dt = v(t)
	//	x(t+dt) = x(t) + v(t)dt
	a3real3Add(p->position.v, a3real3ProductS(d.v, p->velocity.v, dt));

	//	v(t+dt) = v(t) + g(t)dt
	//					 g(t) = dv/dt = a(t)
	//	v(t+dt) = v(t) + a(t)dt
	a3real3Add(p->velocity.v, a3real3ProductS(d.v, p->acceleration.v, dt));

	// ****TO-DO: 
	//	- integrate rotation
	//	- integrate angular velocity
	
	a3real4Add(p->rotation.v, a3real4ProductS(r.v, a3quaternionConcat(r.v, p->velocity_a.v, p->rotation.v), dt * a3realHalf));
	a3real4Add(p->velocity_a.v, a3real4ProductS(r.v, p->acceleration_a.v, dt));

	a3real4Normalize(p->rotation.v);
	a3real4Normalize(p->velocity_a.v);
}

extern inline void a3particleIntegrateEulerSemiImplicit(a3_Particle *p, const a3real dt)
{
	a3vec3 d;

	//	v(t+dt) = v(t) + a(t)dt
	a3real3Add(p->velocity.v, a3real3ProductS(d.v, p->acceleration.v, dt));

	//	x(t+dt) = x(t) + v(t+dt)dt
	a3real3Add(p->position.v, a3real3ProductS(d.v, p->velocity.v, dt));

	// ****TO-DO: 
	//	- integrate angular velocity
	//	- integrate rotation
}

extern inline void a3particleIntegrateEulerKinematic(a3_Particle *p, const a3real dt)
{
	a3vec3 d;

	//	x(t+dt) = x(t) + v(t)dt + a(t)dt2 / 2
	a3real3Add(p->position.v, a3real3ProductS(d.v, a3real3Sum(d.v, p->velocity.v, a3real3ProductS(d.v, p->acceleration.v, a3realHalf * dt)), dt));

	//	v(t+dt) = v(t) + a(t)dt
	a3real3Add(p->velocity.v, a3real3ProductS(d.v, p->acceleration.v, dt));

	// ****TO-DO: 
	//	- integrate rotation using kinematic formula
	//	- integrate angular velocity

	//q(t+dt) = q(t) + q'(t)dt + q" INCOMPLETE
	//q" = dq/dt = wq/2 -> d(dq/dt)/dt = d2q/dt2 = d(wq/2)dt = (aq + wqw/2)/2
	//q(t+dt) = q(t) + w(t)q(t)dt/2 + (aq/2 + w(t)^2 q(t)/4)dt^2 / 2
}


//-----------------------------------------------------------------------------

extern inline int a3particleSetMass(a3_Particle *p, const a3real mass)
{
	if (p)
	{
		// ****TO-DO: 
		//	- correctly set mass: 
		//		- valid or contingency for invalid
		if (mass > a3realZero)
		{
			p->mass = mass;
			p->massInverse = a3recip(mass);
			return 1;
		}
		else
		{
			p->mass = p->massInverse = a3realZero;
			return 0;
		}

	}
	return -1;
}


// reset force
extern inline int a3particleResetForce(a3_Particle *p)
{
	if (p)
	{
		// ****TO-DO: 
		//	- set force to zero vector
		a3real3Set(p->force.v, a3realZero, a3realZero, a3realZero);
		return 1;
	}
	return -1;
}

// check if particle is moving
extern inline int a3particleIsMoving(const a3_Particle *p)
{
	if (p)
	{
		// ****TO-DO: 
		//	- determine if particle is moving (speed is not zero)
		const a3real v = a3real3LengthSquared(p->velocity.v);
		return a3isNotNearZero(v);
	}
	return -1;
}

// calculate critical damping for a particle
extern inline a3real a3calcCriticalDamping(const a3real mass, const a3real springCoeff)
{
	// a damped oscillator follows quadratic formula, 
	//	discriminant is used to determine damping: 
	// c2 - 4mk > 0		-> overdamped
	// c2 - 4mk = 0		-> critically damped
	// c2 - 4mk < 0		-> underdamped
	// e.g. http://hyperphysics.phy-astr.gsu.edu/hbase/oscda.html
	return (mass + mass)*springCoeff;
}


// calculate force of gravity in freefall
extern inline a3real3r a3forceGravity(a3real3p f_out, const a3real3p unitUpward, const a3real mass)
{
	// standard acceleration due to gravity: 
	//	g = 9.80665 m/s2 "down"

	// ****TO-DO:
	//	- store gravity vector
	//	- convert acceleration to force: F = ma
	//		g = a -> F = mg
	const a3real f_g = (a3real)(-9.80665) * mass;
	f_out = a3real3ProductS(f_out, unitUpward, f_g);

	return f_out;
}

// calculate normal force against plane
extern inline a3real3r a3forceNormal(a3real3p f_out, const a3real3p f_gravity, const a3real3p unitNormal)
{
	// ****TO-DO: 
	//	- projection of gravity vector onto surface vector
	//	- result would be negative, so just multiply by negative
	const a3real f_n = -a3real3Dot(f_gravity, unitNormal);
	a3real3ProductS(f_out, unitNormal, f_n);
	return f_out;
}

// calculate sliding force on a plane
extern inline a3real3r a3forceSliding(a3real3p f_out, const a3real3p f_gravity, const a3real3p f_normal)
{
	// 2D example: http://www.physicsclassroom.com/class/vectors/Lesson-3/Inclined-Planes
	// need to convert this to 3D: 
	//	- force of gravity:		Fg = mg
	//	- normal force:			Fn = Fg cos(a)
	//	- final:				F = Fg + Fn

	// ****TO-DO: 
	//	- calculate sliding force
	a3real3Sum(f_out, f_gravity, f_normal);
	return f_out;
}

// calculate static friction (if object is still)
extern inline a3real3r a3forceFrictionStatic(a3real3p f_out, const a3real3p f_normal, const a3real3p f_opposing, const a3real frictionCoeffStatic)
{
	// ****TO-DO: 
	//	- calculate maximum force: k * Fn
	//	- if opposing force is less than max, apply negative opposing
	//	- if opposing force exceeds max, apply maximum in opposite direction
	//	- start with squared forces for fast compare just in case
	const a3real maxForce = a3real3LengthSquared(f_normal) * frictionCoeffStatic * frictionCoeffStatic;
	const a3real opposing = a3real3LengthSquared(f_opposing);
	a3real3GetNegative(f_out, f_opposing);

	if (opposing > maxForce)
	{
		const a3real factor = (a3real)(a3sqrt((a3f64)maxForce) / a3sqrt((a3f64)opposing));
		a3real3MulS(f_out, factor);
	}

	return f_out;
}

// calculate kinetic friction (if particle is moving)
extern inline a3real3r a3forceFrictionKinetic(a3real3p f_out, const a3real3p f_normal, const a3real3p particleVelocity, const a3real frictionCoeffKinetic)
{
	// ****TO-DO: 
	//	- calculate force: k * Fn
	//	- apply in opposite direction of travel
	const a3real f_f = a3real3Length(f_normal) * frictionCoeffKinetic;
	const a3real reverse = -a3real3Length(particleVelocity);
	a3real3ProductS(f_out, particleVelocity, f_f / reverse);

	return f_out;
}

// calculate drag force through fluid
extern inline a3real3r a3forceDrag(a3real3p f_out, const a3real3p particleVelocity, const a3real3p fluidVelocity, const a3real fluidDensity, const a3real objectArea, const a3real objectDragCoeff)
{
	// ****TO-DO: 
	//	- implement drag equation: F = (p u2 A C) / 2
	//		- p is the fluid density
	//		- u is the flow velocity relative to object
	//		- A is the cross-sectional area of the object
	//		- C is the object's drag coefficient
	a3real f_d = a3realHalf * fluidDensity * objectArea * objectDragCoeff;
	a3real3Diff(f_out, fluidVelocity, particleVelocity);
	a3real3MulS(f_out, f_d * a3real3Length(f_out));
	return f_out;
}

// calculate spring force using Hooke's law
extern inline a3real3r a3forceSpring(a3real3p f_out, const a3real3p particlePosition, const a3real3p anchorPosition, const a3real restingLength, const a3real springCoeff)
{
	// ****TO-DO: 
	//	- calculate displacement (vector from anchor to position)
	//	- calculate tension using Hooke's law: F = -k(L - L0) = k(l0 - L)
	//	- apply tension in direction of anchor
	a3real length, tension;
	a3real3Diff(f_out, particlePosition, anchorPosition);
	length = a3real3Length(f_out);
	tension = springCoeff * (restingLength - length);
	a3real3MulS(f_out, tension / length);

	return f_out;
}

// calculate general damping force
extern inline a3real3r a3forceDampingLinear(a3real3p f_out, const a3real3p particleVelocity, const a3real dampingCoeff)
{
	// ****TO-DO: 
	//	- implement simple linear damper: F = -cv
	a3real3ProductS(f_out, particleVelocity, -dampingCoeff);
	return f_out;
}


//-----------------------------------------------------------------------------

//	integrate first order
extern inline void a3particleIntegrateFirstOrder(a3_Particle *p, const a3real dt)
{
	// delta value: dx for position, dv for velocity.
//	a3vec3 dx;
//	a3vec3 dv;

	// ****TO-DO: update variables using values from current state

}

//	integrate second order
extern inline void a3particleIntegrateSecondOrder(a3_Particle *p, const a3real dt)
{
//	a3vec3 dx;
//	a3vec3 dv;

	// ****TO-DO: update variables using values over multiple states

}

//	integrate displacement
extern inline void a3particleIntegrateDisplacement(a3_Particle *p, const a3real dt)
{
//	a3vec3 dx;
//	a3vec3 dv;

	// ****TO-DO: update position using average of current and next velocity

}

//	integrate Verlet
extern inline void a3particleIntegrateVerlet(a3_Particle *p, const a3real dt)
{
	// ****TO-DO: 
	// Verlet integration: 
	//	x(t+dt) = 2 x(t) - x(t-dt) + a(t)dt^2
	// velocity Verlet integration: 
	//	use displacement formula for BOTH position and velocity
}

//	integrate Adams-Bashforth (2nd step)
extern inline void a3particleIntegrateAdamsBashforth2(a3_Particle *p, const a3real dt)
{
	// ****TO-DO: 
	// Adams-Bashforth iteration: 
	//	x(t+dt) = x(t) + (3 v(t) - v(t-dt))dt/2
}

//	integrate Runge-Kutta (RK4)
extern inline void a3particleIntegrateRungeKutta4(a3_Particle *p, const a3real dt)
{
	// ****TO-DO: 
	// RK4: "the" Runge-Kutta method: 
	//	x'(t) = f(t, x(t))
	//		k0 = f(t, x(t))		<- literally Euler's method (first-order)
	//		k1 = f(t + dt/2, x(t) + k0 dt/2)
	//		k2 = f(t + dt/2, x(t) + k1 dt/2)
	//		k3 = f(t + dt, x(t) + k2 dt)
	//	x(t+dt) = x(t) + (k0 + 2 k1 + 2 k2 + k3)dt/6
}


//-----------------------------------------------------------------------------
