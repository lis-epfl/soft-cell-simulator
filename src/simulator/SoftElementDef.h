/*
 * @(#) SoftElementDef.h   1.0   Dec 14, 2011
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
#ifndef SIMULATOR_SOFT_ELEMENT_DEF_H_
#define SIMULATOR_SOFT_ELEMENT_DEF_H_

#include "Box2D/Box2D.h"

namespace scs {

class SoftElementDef {

public:

	SoftElementDef() {
		nbrBodies_ = 30; // 100, softness 200, radius 5, damping 50
		initPosition_ = b2Vec2(0.0f, 0.0f);
		density_ = 1;
		restitution_ = 0;
		friction_ = 0.1;
	}

	SoftElementDef(unsigned int nbrBodies, float density, float restitution,
			float friction, b2Vec2 initPosition) :
			nbrBodies_(nbrBodies), initPosition_(initPosition), density_(
					density), restitution_(restitution), friction_(friction) {
	}

	void setNbrBodies(unsigned int nbrBodies) {
		nbrBodies_ = nbrBodies;
	}

	void setInitPosition(float x, float y) {
		initPosition_ = b2Vec2(x, y);
	}

	void setInitPosition(b2Vec2 initPosition) {
		initPosition_ = initPosition;
	}

	float getDensity() const {
		return density_;
	}

	float getRestitution() const {
		return restitution_;
	}

	float getFriction() const {
		return friction_;
	}

	void setFriction(float friction) {
		this->friction_ = friction;
	}

	void setDensity(float density) {
		this->density_ = density;
	}

	unsigned int getNbrBodies() const {
		return nbrBodies_;
	}

	b2Vec2 getInitPosition() const {
		return initPosition_;
	}

private:

	// The number of bodies that builds up the soft element surface
	unsigned int nbrBodies_;

	// Initial position of the element
	b2Vec2 initPosition_;

	float density_;

	float restitution_;

	float friction_;

};

}

#endif /* SIMULATOR_SOFT_ELEMENT_DEF_H_ */
