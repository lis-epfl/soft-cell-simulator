/*
 * @(#) LinearActuatorInternalConnection.cpp   1.0   Dec 14, 2011
 *
 * Manuel Stockli (manuel.stockli@epfl.ch)
 * Andrea Maesani (andrea.maesani@epfl.ch)
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
#include <vector>
#include <iostream>
#include "connections/internal/LinearActuatorInternalConnection.h"
#include "SoftElement.h"

namespace scs {

LinearActuatorInternalConnection::LinearActuatorInternalConnection(
		b2World* world, float lengthFactor, float frequency, float damping,
		float minLengthFactor, unsigned int actuationSteps) :
		InternalConnection(world), lengthFactor_(lengthFactor), frequency_(
				frequency), damping_(damping), minLengthFactor_(
				minLengthFactor), actuationSteps_(actuationSteps), connectionJoint_(
				NULL) {
}

LinearActuatorInternalConnection::~LinearActuatorInternalConnection() {
	if (this->connectionJoint_ != NULL) {
		this->getWorld()->DestroyJoint(this->connectionJoint_);
	}
}

void LinearActuatorInternalConnection::bindConnection(SoftElement* softElement,
		unsigned int bodyIndexA, unsigned int bodyIndexB) {

	// initialise the connectedElement_ to remember which element the internal connection is bound to
	softElement_ = softElement;

	std::vector<b2Body*>& bodies = softElement->getBodies();

	// Definition for distance joint with spring effect
	b2DistanceJointDef jd;
	jd.collideConnected = true;

	jd.frequencyHz = frequency_;
	jd.dampingRatio = damping_;
	jd.localAnchorA.Set(0.0f, 0.0f);
	jd.localAnchorB.Set(0.0f, 0.0f);
	jd.length = lengthFactor_;

	jd.bodyA = bodies[bodyIndexA];
	jd.bodyB = bodies[bodyIndexB];
	b2Vec2 p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
	b2Vec2 p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
	b2Vec2 d = p2 - p1;
	jd.length *= d.Length();
	connectionJoint_ = (b2DistanceJoint*) this->getWorld()->CreateJoint(&jd);

	this->softElement_->updateSurface();

	this->length_ = connectionJoint_->GetLength();

}

void LinearActuatorInternalConnection::increase() {

	float curLength = connectionJoint_->GetLength();
	if ((curLength / this->length_) < 1) {

		connectionJoint_->SetLength(curLength * (1 + 1.0 / actuationSteps_));

	}

	this->softElement_->updateSurface();
}

void LinearActuatorInternalConnection::setLength(float length) {

	this->connectionJoint_->SetLength(
			this->length_
					* (this->minLengthFactor_
							+ (1 - this->minLengthFactor_) * length));
	this->softElement_->updateSurface();
}

void LinearActuatorInternalConnection::decrease() {

	float curLength = connectionJoint_->GetLength();
	if (curLength / this->length_ > this->minLengthFactor_) {

		connectionJoint_->SetLength(curLength / (1 + 1.0 / actuationSteps_));

	}

	this->softElement_->updateSurface();
}

float LinearActuatorInternalConnection::getLength() {
	return this->connectionJoint_->GetLength() / this->length_;
}

}
