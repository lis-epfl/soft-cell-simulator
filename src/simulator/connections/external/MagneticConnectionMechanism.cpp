/*
 * @(#) MagneticConnectionMechanism.cpp   1.0   Dec 14, 2011
 *
 * Manuel Stockli (manuel.stockli@epfl.ch)
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * Copyright © 2011-2013 Laboratory of Intelligent Systems, EPFL
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
#include <cmath>

#include "connections/external/ConnectionMechanismDef.h"
#include "connections/external/MagneticConnectionMechanism.h"

#include "SoftElementDef.h"
#include "SoftElement.h"

#include <iostream>
using namespace std;
#include <stdlib.h>

using std::cout;
using std::endl;

#define deg2rad 0.0174532925199432957f
#define rad2deg 57.295779513082320876f

namespace scs {

struct SensorUserData {
	// designates the state in which the connection mechanism is
	int32 state;
	// designates the number of the element where the connection mechanism is
	int32 nbrElement;
	// number of contacts (in case there are more than one at a time)
	int32 nbrContacts;
};

MagneticConnectionMechanism::MagneticConnectionMechanism(b2World* world,
		const ConnectionMechanismDef& definition) :
		ConnectionMechanism(world, definition) {

	// initialising the variables

}

MagneticConnectionMechanism::~MagneticConnectionMechanism() {

}

void MagneticConnectionMechanism::bindToSoftElement(SoftElement* softElement,
		unsigned int bodyIndex) {

	// copy the Id of the soft element
	elementId_ = softElement->getId();

	// get length and width of the soft elements body
	length_ = softElement->getBodyLength();
	width_ = 1.1 * softElement->getBodyWidth(); // make a bit bigger to avoid vibrations during connection

			// define fixture definition for the connection point with a slightly larger box to avoid vibrations
	b2FixtureDef fdConnect;
	fdConnect.density = softElement->getDefinition().getDensity();
	fdConnect.restitution = softElement->getDefinition().getRestitution();
	fdConnect.friction = softElement->getDefinition().getFriction();
	b2PolygonShape shapeConnect;
	shapeConnect.SetAsBox(width_ * 1, width_ * 1);

	fdConnect.shape = &shapeConnect;

	// Fixture Definition for the influence region of the connection bodies
	b2FixtureDef ConnectFixDef;
	b2CircleShape circleShape;
	circleShape.m_radius = this->getDefinition().getInteractionRadius();
	ConnectFixDef.shape = &circleShape;
	ConnectFixDef.isSensor = true;
	ConnectFixDef.filter.categoryBits = SENSOR;
	ConnectFixDef.filter.maskBits = SENSOR; // only sense other connection bodies

	// Definition for revolute joint between body and connection point
	// Actually rigid joint, since lower and higher limit are zero
	b2RevoluteJointDef jdConnect;
	jdConnect.collideConnected = false;
	jdConnect.localAnchorA.Set(0, 0);
	jdConnect.localAnchorB.Set(0, 0);

	// define the sensor body
	b2BodyDef bd;
	bd.type = b2_dynamicBody;

	std::vector<b2Body*>& bodies = softElement->getBodies();

	// get the body of the element which is going to be a connection point, only executed if bodyIndex is valid
	if (bodyIndex < bodies.size()) {

		connectedBody_ = bodies[bodyIndex];

		// Create fixture for the indexed body as connection point fixture
		connectedBody_->CreateFixture(&fdConnect);

		// Create new connection point as body with sensor fixture
		bd.position.Set(connectedBody_->GetPosition().x,
				connectedBody_->GetPosition().y);
		bd.angle = connectedBody_->GetAngle();
		connectPoint_ = this->getWorld()->CreateBody(&bd);
		connectPoint_->CreateFixture(&ConnectFixDef);

		// add blocked rotational joint
		jdConnect.bodyA = connectPoint_;
		jdConnect.bodyB = connectedBody_;
		this->getWorld()->CreateJoint(&jdConnect);

		connectPoint_->SetUserData(this);

	}

}

void MagneticConnectionMechanism::applyForces(ConnectionMechanism* otherPoint) {

	// get positions of the connection points
	b2Vec2 firstPos = this->connectPoint_->GetPosition();
	b2Vec2 secondPos = otherPoint->getConnectPoint()->GetPosition();

	// connection force as magnetic force between two magnets
	b2Vec2 diff = firstPos - secondPos;

	// get the angle of this vector
	float phi = asin(diff.y / diff.Length());

	float diffdash = diff.Length() - 2 * 0.1885;

	b2Vec2 diff2;
	diff2.x = diffdash * cos(phi);
	diff2.y = diffdash * sin(phi);

	float diffLength = std::max(diff.Length(), this->length_);
	// we dont want the force to be too big to avoid vibrations
	b2Vec2 connectForce = this->getDefinition().getAmplitude()
			* pow(diffLength, -3) * diff;

	// apply forces
	this->connectPoint_->ApplyForce(-connectForce, firstPos);
	otherPoint->getConnectPoint()->ApplyForce(connectForce, secondPos);

}

float MagneticConnectionMechanism::getLength() const {
	return this->length_;
}

float MagneticConnectionMechanism::getWidth() const {
	return this->width_;
}

b2Body* MagneticConnectionMechanism::getConnectedBody() {
	return this->connectedBody_;
}

b2Body* MagneticConnectionMechanism::getConnectPoint() {
	return this->connectPoint_;
}

int MagneticConnectionMechanism::getElementId() const {
	return this->elementId_;
}

}
