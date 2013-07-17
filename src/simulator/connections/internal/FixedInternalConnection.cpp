/*
 * @(#) FixedInternalConnection.cpp   1.0   Dec 14, 2011
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
#include <vector>

#include "connections/internal/FixedInternalConnection.h"
#include "SoftElement.h"

namespace scs {

FixedInternalConnection::FixedInternalConnection(b2World* world,
		float lengthFactor, float frequency, float damping) :
		InternalConnection(world), lengthFactor_(lengthFactor), frequency_(
				frequency), damping_(damping), joint_(NULL) {

}

FixedInternalConnection::~FixedInternalConnection() {

	if (this->joint_ != NULL) {
		this->getWorld()->DestroyJoint(this->joint_);
	}

}

void FixedInternalConnection::bindConnection(SoftElement* softElement,
		unsigned int bodyIndexA, unsigned int bodyIndexB) {

	this->softElement_ = softElement;

	std::vector<b2Body*>& bodies = softElement->getBodies();

	// Definition for distance joint with spring effect
	b2DistanceJointDef jd;
	jd.collideConnected = true;

	jd.frequencyHz = frequency_;
	jd.dampingRatio = damping_;
	jd.localAnchorA.Set(0.0f, 0.0f);
	jd.localAnchorB.Set(0.0f, 0.0f);
	jd.length = lengthFactor_;

	if (bodyIndexA < bodies.size() && bodyIndexA < bodies.size()) {

		jd.bodyA = bodies[bodyIndexA];
		jd.bodyB = bodies[bodyIndexB];
		b2Vec2 p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
		b2Vec2 p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
		b2Vec2 d = p2 - p1;
		jd.length *= d.Length();
		this->joint_ = this->getWorld()->CreateJoint(&jd);
	}

	this->softElement_->updateSurface();

}

}
