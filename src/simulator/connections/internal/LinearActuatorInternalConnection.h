/*
 * @(#) LinearActuatorInternalConnection.h   1.0   Dec 14, 2011
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
#ifndef SIMULATOR_LINEAR_ACTUATOR_INTERNAL_CONNECTION_H_
#define SIMULATOR_LINEAR_ACTUATOR_INTERNAL_CONNECTION_H_

#include "connections/internal/InternalConnection.h"

namespace scs {

class LinearActuatorInternalConnection: public InternalConnection {

public:

	LinearActuatorInternalConnection(b2World* world, float lengthFactor,
			float frequency, float damping, float minLengthFactor,
			unsigned int actuationSteps);

	virtual ~LinearActuatorInternalConnection();

	void bindConnection(SoftElement* softElement, unsigned int bodyIndexA,
			unsigned int bodyIndexB);

	void increase();

	void decrease();

	/**
	 * Set the current length, normalized in [0,1]
	 */
	void setLength(float length);

	/**
	 * @return the current length of the connection, normalized in [0,1]
	 */
	float getLength();

private:

	float lengthFactor_;

	// Factor for the stiffness, designates the frequency of the spring between ear ends
	float frequency_;

	// Factor for the damping ratio of the spring
	float damping_;

	// The initial length
	float length_;

	float minLengthFactor_;

	unsigned int actuationSteps_;

	b2DistanceJoint* connectionJoint_;

	SoftElement* softElement_;

};

}

#endif /* SIMULATOR_LINEAR_ACTUATOR_INTERNAL_CONNECTION_H_ */
