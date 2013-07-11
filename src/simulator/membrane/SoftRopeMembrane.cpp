/*
 * @(#) SoftRopeMembrane.cpp   1.0   Dec 02, 2012
 *
 * Jurg Germann (jurg.germann@epfl.ch)
 *
 * Copyright © 2011-2013 Manuel Stockli, Andrea Maesani, Jurg Germann
 *
 * This file is part of the Soft Cell Simulator (SCS) software.
 *
 * Soft Cell Simulator (SCS) is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Soft Cell Simulator (SCS) is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Soft Cell Simulator (SCS).  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#define deg2rad 0.0174532925199432957f
#define rad2deg 57.295779513082320876f

#include "Box2D/Box2D.h"

#include "membrane/SoftRopeMembrane.h"

#include "SoftElementDef.h"
#include "SoftElement.h"

namespace scs {

SoftRopeMembrane::SoftRopeMembrane(b2World* world,
		const SoftElementDef& softElementDefinition,
		const SoftMembraneDef& definition, unsigned int id) :
		SoftElement(world, softElementDefinition, id), definition_(definition) {

	float radius = this->definition_.getRadius();
	unsigned int nbrBodies = SoftElement::getDefinition().getNbrBodies();

	// Initial surface is the circular surface, needed for pressure calculations
	initSurface_ = radius * radius * 180.0f * deg2rad * 1.05;

	// Initialise the size of the bodies, the boxes have to be big enough for a physical joint
	// factor 9 and 2.5 are experimental
	bodyLength_ = 9 * radius / nbrBodies;
	bodyWidth_ = 2.5 * radius / nbrBodies;
	bodyWidth_ = (180.0f * deg2rad * radius / nbrBodies) * 1;
	// Create the bodies and their fixtures

	// Define the bodies,
	b2CircleShape shape;
	shape.m_radius = bodyWidth_;

	b2FixtureDef fd;
	fd.shape = &shape;
	fd.density = SoftElement::getDefinition().getDensity();
	fd.restitution = SoftElement::getDefinition().getRestitution();
	fd.friction = SoftElement::getDefinition().getFriction();

	// Definition for rope joint between bodies
	b2RopeJointDef ropeJoint;
	ropeJoint.type = e_ropeJoint;
	ropeJoint.localAnchorA.Set(0.0f, 0.0f);
	ropeJoint.localAnchorB.Set(0.0f, 0.0f);
	ropeJoint.maxLength = 1.5f;

	// Position of the Element
	const float32 x = SoftElement::getDefinition().getInitPosition().x;
	const float32 y = SoftElement::getDefinition().getInitPosition().y;
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(x, y + radius);
	bodies_.push_back(world->CreateBody(&bd));
	bodies_[0]->CreateFixture(&fd);

	// Create the nbrBodies Bodies in a circular shape
	// Leave i as an integer
	for (int i = 1; i < (int) nbrBodies; ++i) {

		//bd.position.Set(x+i,y+i);
		bd.position.Set(
				x + radius * sin((i - 0.0f) / nbrBodies * 360.0f * deg2rad),
				y + radius * cos((i - 0.0f) / nbrBodies * 360.0f * deg2rad));
		bd.angle = (-i - 0.0f) / (float) nbrBodies * 360.0f * deg2rad;
		bodies_.push_back(world->CreateBody(&bd));

		bodies_[i]->CreateFixture(&fd);

		//ropeJoint.Initialize(bodies_[i - 1], bodies_[i], bodies_[i - 1]->GetWorldCenter());
		ropeJoint.bodyA = bodies_[i - 1];
		ropeJoint.bodyB = bodies_[i];

		this->joints_.push_back(world->CreateJoint(&ropeJoint));

		if (i == (nbrBodies - 1)) {
			ropeJoint.bodyA = bodies_[i];
			ropeJoint.bodyB = bodies_[0];
			this->joints_.push_back(world->CreateJoint(&ropeJoint));
		}

	}

}

SoftRopeMembrane::~SoftRopeMembrane() {

	for (unsigned int i = 0; i < this->joints_.size(); ++i) {
		this->getWorld()->DestroyJoint(this->joints_[i]);
	}

	for (unsigned int i = 0; i < this->bodies_.size(); ++i) {
		this->getWorld()->DestroyBody(this->bodies_[i]);
	}
}

SoftMembraneDef& SoftRopeMembrane::getDefinition() {
	return definition_;
}

// claculate the momentary surface and update the "initSurface_"
void SoftRopeMembrane::updateSurface() {

	unsigned int nbrBodies = SoftElement::getDefinition().getNbrBodies();

	// get vector with center of gravity of the bodies and their positions
	std::vector<b2Vec2> Positions;
	for (unsigned int i = 0; i < nbrBodies; i++) {
		Positions.push_back(bodies_[i]->GetWorldCenter());
	}
	// add first element as last to enable cross product
	Positions.push_back(Positions[0]);

	// get surface of the total Element
	float32 surface = 0.0f;
	for (unsigned int i = 0; i < Positions.size() - 1; i += 2) {
		surface += Positions[i + 1].x * (Positions[i + 2].y - Positions[i].y)
				+ Positions[i + 1].y * (Positions[i].x - Positions[i + 2].x);
	}

}

float SoftRopeMembrane::getSurface() {
	return this->initSurface_;
}

void SoftRopeMembrane::setSurface(float surface) {
	this->initSurface_ = surface;
}

void SoftRopeMembrane::computeDynamics(float globalFriction) {

	unsigned int nbrBodies = SoftElement::getDefinition().getNbrBodies();

	// get vector with center of gravity of the bodies and their positions
	std::vector<b2Vec2> Positions;
	// get the average linear velocity of all the bodies
	std::vector<b2Vec2> Velocities;
	b2Vec2 avg_vel(0.0f, 0.0f);
	for (unsigned int i = 0; i < nbrBodies; i++) {
		Positions.push_back(bodies_[i]->GetWorldCenter());
		b2Vec2 vel = bodies_[i]->GetLinearVelocity();
		Velocities.push_back(vel);
		avg_vel += vel;
	}
	// average sum of the position to get element center
	// add first element as last to enable cross product
	Positions.push_back(Positions[0]);
	// average sum of linear velocity
	avg_vel *= 1.0f / (float) nbrBodies;

	// get surface of the total Element
	float32 surface = 0.0f;
	for (unsigned int i = 0; i < Positions.size() - 1; i += 2) {
		surface += Positions[i + 1].x * (Positions[i + 2].y - Positions[i].y)
				+ Positions[i + 1].y * (Positions[i].x - Positions[i + 2].x);
	}
	surface = abs(surface / 2);

	//calculate force on each body of the Soft Element
	float32 force = 0.0f;
	if (surface > 0) {
		// Internal pressure
		force = this->definition_.getSoftness() * (initSurface_ / surface - 1);
	}

	// calculate the resulting force and moment if the forces were applied on each element theoretically; allows to prohibit
	// a resulting force and moment acting on the element due to internal pressure
	b2Vec2 resultForce(0.0f, 0.0f);
	for (unsigned int i = 0; i < nbrBodies; i++) {
		// add some friction force for the relative velocity of each element
		b2Vec2 f = bodies_[i]->GetWorldVector(
				(b2Vec2(0.0f,
						force - SoftElement::getDefinition().getFriction())));
		resultForce += f;
	}
	resultForce *= 1.0f / (float) nbrBodies; // use this force and moment later to correct the force on each element;

	for (unsigned int i = 0; i < nbrBodies; i++) {
		// add some friction force for the relative velocity of each element
		b2Vec2 rel_vel = bodies_[i]->GetLocalVector(Velocities[i] - avg_vel);
		b2Vec2 friction = this->definition_.getDamping() * rel_vel;

		// add global friction if computed
		b2Vec2 globFriction = globalFriction * avg_vel;

		b2Vec2 f = bodies_[i]->GetWorldVector(b2Vec2(0.0f, force) - friction);
		b2Vec2 p = bodies_[i]->GetWorldPoint(b2Vec2(0.0f, 0.0f));
		bodies_[i]->ApplyForce(f - globFriction - resultForce, p);
	}
}

std::vector<b2Body*>& SoftRopeMembrane::getBodies() {
	return this->bodies_;
}

float SoftRopeMembrane::getBodyLength() const {
	return this->bodyLength_;
}

float SoftRopeMembrane::getBodyWidth() const {
	return this->bodyWidth_;
}

b2Vec2 SoftRopeMembrane::getCenter() const {

	std::vector<b2Vec2> Positions;
	b2Vec2 center(0.0f, 0.0f);
	for (unsigned int i = 0; i < this->bodies_.size(); i++) {
		b2Vec2 pos = bodies_[i]->GetWorldCenter();
		Positions.push_back(pos);
		center += pos;
	}
	// average sum of the position to get element center
	center *= 1.0f / this->bodies_.size();
	return center;
}

void SoftRopeMembrane::mergeMembrane(std::string mergeEdge,
		SoftRopeMembrane* other, float elementsToConnect) {

	std::vector<b2Body*>& bodiesA = this->bodies_;
	std::vector<b2Body*>& bodiesB = other->getBodies();

	int nBodiesA = bodiesA.size();
	int nBodiesB = bodiesB.size();

	unsigned int nBodiesToUse = 0;
	if (nBodiesA * elementsToConnect < nBodiesB * elementsToConnect) {
		nBodiesToUse = nBodiesA * elementsToConnect;
	} else {
		nBodiesToUse = nBodiesB * elementsToConnect;
	}

	// Get the bodies to connect (bodies are numbered starting from the top, clockwise)
	// A will be connected in clockwise order, B in anti-clockwise order
	int mergeIxA = 0;
	int mergeIxB = 0;
	if (mergeEdge.compare("t") == 0 || mergeEdge.compare("tr") == 0) {
		mergeIxA = 0;
		mergeIxB = 2;
	} else if (mergeEdge.compare("r") == 0 || mergeEdge.compare("br") == 0) {
		mergeIxA = 1;
		mergeIxB = 3;
	} else if (mergeEdge.compare("b") == 0 || mergeEdge.compare("bl") == 0) {
		mergeIxA = 2;
		mergeIxB = 0;
	} else if (mergeEdge.compare("l") == 0 || mergeEdge.compare("tl") == 0) {
		mergeIxA = 3;
		mergeIxB = 1;
	}

	bool shifted = false;
	if (mergeEdge.compare("tr") == 0 || mergeEdge.compare("br") == 0
			|| mergeEdge.compare("br") == 0 || mergeEdge.compare("bl") == 0) {
		shifted = true;
	}

	int startBodyA = (mergeIxA * nBodiesA / 4) - nBodiesToUse / 2;
	if (shifted) {
		startBodyA += nBodiesA / 8;
	}
	if (startBodyA < 0) {
		startBodyA = nBodiesA + startBodyA;
	}

	int startBodyB = (mergeIxB * nBodiesB / 4) + nBodiesToUse / 2;
	if (shifted) {
		startBodyB += nBodiesB / 8;
	}
	if (startBodyB < 0) {
		startBodyB = nBodiesB + startBodyB;
	}
	for (unsigned int i = 0; i < nBodiesToUse; ++i) {

		b2Body* bodyA = bodiesA[(startBodyA + i) % nBodiesA];

		int ixB = startBodyB - i;
		if (ixB < 0) {
			ixB = nBodiesB + ixB;
		}
		b2Body* bodyB = bodiesB[ixB % nBodiesB];

		b2DistanceJointDef jointDef;
		jointDef.Initialize(bodyA, bodyB, bodyA->GetWorldCenter(),
				bodyB->GetWorldCenter());
		jointDef.length = 0;
		jointDef.collideConnected = false;
		this->getWorld()->CreateJoint(&jointDef);

	}

}

}
