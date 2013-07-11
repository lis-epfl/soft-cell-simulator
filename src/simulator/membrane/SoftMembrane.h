/*
 * @(#) SoftMembrane.h   1.0   Dec 14, 2011
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
#ifndef SIMULATOR_SOFT_MEMBRANE_H_
#define SIMULATOR_SOFT_MEMBRANE_H_

#include <string>

#include "membrane/SoftMembraneDef.h"

#include "SoftElement.h"

namespace scs {

class SoftMembrane: public SoftElement {

public:

	SoftMembrane(b2World* world, const SoftElementDef& softElementDefinition,
			const SoftMembraneDef& definition, unsigned int id);

	virtual ~SoftMembrane();

	SoftMembraneDef& getDefinition();

	std::vector<b2Body*>& getBodies();

	float getBodyLength() const;

	float getBodyWidth() const;

	void computeDynamics(float globalFriction);

	void updateSurface();

	virtual float getSurface();

	virtual void setSurface(float surface);

	b2Vec2 getCenter() const;

	void setCenter(b2Vec2 newBodyCenter);

	b2Vec2 getLinearVelocity(float delta_t) const;

	/**
	 * Merge the membrane with another one
	 */
	void mergeMembrane(std::string mergeEdge, SoftMembrane* other,
			float elementsToConnect);

private:

	// Total definition of the element
	SoftMembraneDef definition_;

	// Initial surface of this circular Soft Element
	float initSurface_;

	// vector of bodies
	std::vector<b2Body*> bodies_;
	std::vector<b2Joint*> joints_;

	// Length of one Body
	float bodyLength_;

	// Width of one Body
	float bodyWidth_;

	// body centers positions in time
	b2Vec2 oldBodyCenter_;
	b2Vec2 newBodyCenter_;

};

}

#endif /* SIMULATOR_SOFT_MEMBRANE_H_ */
