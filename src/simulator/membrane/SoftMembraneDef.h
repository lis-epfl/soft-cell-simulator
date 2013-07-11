/*
 * @(#) SoftMembraneDef.h   1.0   Dec 6, 2011
 *
 * Manuel Stockli (manuel.stockli@epfl.ch)
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
#ifndef SIMULATOR_SOFT_MEMBRANE_DEF_H_
#define SIMULATOR_SOFT_MEMBRANE_DEF_H_

namespace scs {

class SoftMembraneDef {
public:

	// Default version of a good Soft Membrane
	SoftMembraneDef() {
		radius_ = 3;
		softness_ = 10;
		damping_ = 1;
	}

	SoftMembraneDef(float radius, float softness, float damping) :
			radius_(radius), softness_(softness), damping_(damping) {
	}

	void setRadius(float radius) {
		radius_ = radius;
	}

	void setSoftness(float softness) {
		softness_ = softness;
	}

	void setDamping(float damping) {
		damping_ = damping;
	}

	float getRadius() const {
		return radius_;
	}

	float getSoftness() const {
		return softness_;
	}

	float getDamping() const {
		return damping_;
	}

private:

	float radius_;

	float softness_;

	float damping_;

};

}

#endif /* SIMULATOR_SOFT_MEMBRANE_DEF_H_ */
