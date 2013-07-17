/*
 * @(#) FixedInternalConnection.h   1.0   Dec 14, 2011
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
#ifndef SIMULATOR_FIXED_INTERNAL_CONNECTION_H_
#define SIMULATOR_FIXED_INTERNAL_CONNECTION_H_

#include "connections/internal/InternalConnection.h"

namespace scs {

class FixedInternalConnection: public InternalConnection {

public:

	FixedInternalConnection(b2World* world, float lengthFactor, float frequency,
			float damping);

	virtual ~FixedInternalConnection();

	void bindConnection(SoftElement* softElement, unsigned int bodyIndexA,
			unsigned int bodyIndexB);

private:

	float lengthFactor_;

	// Factor for the stiffness, designates the frequency of the spring between ear ends
	float frequency_;

	// Factor for the damping ratio of the spring
	float damping_;

	// Soft element where the internal connection is applied
	SoftElement* softElement_;

	/**
	 * The managed joint
	 */
	b2Joint* joint_;

};

}

#endif /* SIMULATOR_FIXED_INTERNAL_CONNECTION_H_ */
