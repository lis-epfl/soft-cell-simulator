/*
 * @(#) ConnectionMechanismDef.h   1.0   Dec 14, 2011
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
#ifndef SIMULATOR_CONNECTION_MECHANISM_DEF_H_
#define SIMULATOR_CONNECTION_MECHANISM_DEF_H_

namespace scs {

class ConnectionMechanismDef {

public:

	ConnectionMechanismDef() {
		interactionRadius_ = 2.0f;
		amplitude_ = 500.0f;
	}

	ConnectionMechanismDef(float interactionRadius, float amplitude) :
			interactionRadius_(interactionRadius), amplitude_(amplitude) {
	}

	virtual ~ConnectionMechanismDef() {

	}

	float getInteractionRadius() const {
		return interactionRadius_;
	}

	float getAmplitude() const {
		return amplitude_;
	}

private:

	float interactionRadius_;

	float amplitude_;

};

}

#endif /* SIMULATOR_CONNECTION_MECHANISM_DEF_H_ */
