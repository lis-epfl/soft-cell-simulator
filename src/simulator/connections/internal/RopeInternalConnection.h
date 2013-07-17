/*
 * @(#) RopeInternalConnection.h   1.0   Dec 02, 2012
 *
 * Jurg Germann (jurg.germann@epfl.ch)
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
#ifndef SIMULATOR_ROPE_INTERNAL_CONNECTION_H_
#define SIMULATOR_ROPE_INTERNAL_CONNECTION_H_

#include "connections/internal/InternalConnection.h"

namespace scs {

class RopeInternalConnection: public InternalConnection {

public:

	RopeInternalConnection(b2World* world, float maxLength);

	virtual ~RopeInternalConnection();

	void bindConnection(SoftElement* softElement, unsigned int bodyIndexA,
			unsigned int bodyIndexB);

	float getMaxLength();

	void setMaxLength(float maxLength);

private:

	float maxLength_;

	// Factor for the stiffness, designates the frequency of the spring between ear ends
	float frequency_;

	// Factor for the damping ratio of the spring
	float damping_;

	// Soft element where the internal connection is applied
	SoftElement* softElement_;

	/**
	 * The managed joint
	 */
	b2RopeJoint* ropeJoint_;

};

}

#endif /* SIMULATOR_ROPE_INTERNAL_CONNECTION_H_ */
