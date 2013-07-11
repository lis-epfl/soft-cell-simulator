/*
 * @(#) RopeInternalConnection.cpp   1.0   Dec 02, 2012
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
#include <vector>
#include <iostream>
#include "connections/internal/RopeInternalConnection.h"
#include "SoftElement.h"

namespace scs {

RopeInternalConnection::RopeInternalConnection(b2World* world, float maxLength) :
		InternalConnection(world), maxLength_(maxLength), ropeJoint_(NULL) {

}

RopeInternalConnection::~RopeInternalConnection() {

	if (this->ropeJoint_ != NULL) {
		this->getWorld()->DestroyJoint(this->ropeJoint_);
	}

}

void RopeInternalConnection::bindConnection(SoftElement* softElement,
		unsigned int bodyIndexA, unsigned int bodyIndexB) {

	this->softElement_ = softElement;

	std::vector<b2Body*>& bodies = softElement->getBodies();

	// Definition for distance joint with spring effect
	b2RopeJointDef jd;
	jd.localAnchorA.Set(0.0f, 0.0f);
	jd.localAnchorB.Set(0.0f, 0.0f);
	jd.maxLength = maxLength_;

	if (bodyIndexA < bodies.size() && bodyIndexB < bodies.size()) {

		jd.bodyA = bodies[bodyIndexA];
		jd.bodyB = bodies[bodyIndexB];
		this->ropeJoint_ = (b2RopeJoint*) this->getWorld()->CreateJoint(&jd);
	}

	this->softElement_->updateSurface();
}

float RopeInternalConnection::getMaxLength() {
	return this->ropeJoint_->GetMaxLength();
}

void RopeInternalConnection::setMaxLength(float maxLength) {
	this->ropeJoint_->SetMaxLength(maxLength);
	this->softElement_->updateSurface();
}

}
