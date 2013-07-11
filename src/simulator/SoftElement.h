/*
 * @(#) SoftElement.h   1.0   Dec 14, 2011
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
#ifndef SIMULATOR_SOFT_ELEMENT_H_
#define SIMULATOR_SOFT_ELEMENT_H_

#include <vector>

#include "Box2D/Box2D.h"

#include "SoftElementDef.h"

namespace scs {

class SoftElement {

public:

	SoftElement(b2World* world, const SoftElementDef& definition,
			unsigned int id) :
			world_(world), definition_(definition), id_(id) {

	}

	virtual ~SoftElement() {

	}

	b2World* getWorld() {
		return world_;
	}

	SoftElementDef& getDefinition() {
		return this->definition_;
	}

	unsigned int getId() const {
		return id_;
	}

	virtual std::vector<b2Body*>& getBodies() = 0;

	virtual float getBodyLength() const = 0;

	virtual float getBodyWidth() const = 0;

	virtual void computeDynamics(float globalFriction) = 0;

	virtual void updateSurface() = 0;

	virtual float getSurface() = 0;

	virtual void setSurface(float surface) = 0;

	virtual b2Vec2 getCenter() const = 0;

	virtual void setCenter(b2Vec2 newBodyCenter) = 0;

	virtual b2Vec2 getLinearVelocity(float delta_t) const = 0;

private:

	// pointer on world, needed to create all the bodies and joints
	b2World* world_;

	// Total definition of the Soft Element
	SoftElementDef definition_;

	unsigned int id_;

};

}

#endif /* SIMULATOR_SOFT_ELEMENT_H_ */
