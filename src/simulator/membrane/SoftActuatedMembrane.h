/*
 * @(#) SoftActuatedMembrane.h   1.0   Dec 20, 2011
 *
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
#ifndef SIMULATOR_SOFT_ACTUATED_MEMBRANE_H_
#define SIMULATOR_SOFT_ACTUATDE_MEMBRANE_H_

#include <vector>

#include "connections/internal/LinearActuatorInternalConnection.h"

#include "membrane/SoftMembrane.h"

namespace scs {

/**
 * A SoftActuatedMembrane consists of a {@link SoftMembrane} with an internal
 * {@link LinearActuatedInternalConnection}.
 * When a linear actuated internal connection is fully extended,
 * the membrane can fix some of the particles that is made with, to the groud.
 * The particles will unstick to the ground when the internal connections decrease to
 * a certain extent.
 */
class SoftActuatedMembrane {

public:

	/**
	 * Initializes a SoftActuatedMembrane from a SoftMembrane object
	 *
	 * @param membrane the soft membrane object, not managed.
	 *                 When a SoftActuatedMembrane object is destroyed
	 *                 the associated membrane is NOT deallocated.
	 */
	SoftActuatedMembrane(SoftMembrane* membrane, float thresholdA,
			float thresholdB, unsigned int connectionAnchor);

	/**
	 * Destructor
	 */
	virtual ~SoftActuatedMembrane();

	/**
	 * Increase the linear actuator along the axis by one step
	 */
	void increase();

	/**
	 * Decrease the linear actuator along the axis by one step
	 */
	void decrease();

	/**
	 * Set the actuator to the specified length
	 */
	void setActuatorLength(float length);

private:

	enum AttachmentState {
		ATTACHED, DETACHED
	};

	/**
	 * Update the attachment state
	 */
	void updateAttachmentState();

	/**
	 * The soft membrane
	 */
	SoftMembrane *membrane_;

	/**
	 * Cur attachment state:
	 * DETACHED = 0, ATTACHED = 1
	 */
	AttachmentState curState_;

	/**
	 * The actuated connections
	 */
	std::vector<LinearActuatorInternalConnection*> actuatedConnection_;

	/**
	 * The threshold above which the particles will adhere to the ground
	 */
	float thresholdA_;

	/**
	 * The threshold below which the particles will disconnect from the ground
	 */
	float thresholdB_;

	/**
	 * The bodies that will stick to the ground surface
	 */
	std::vector<b2Body*> attachmentPoints_;

	/**
	 * The joints used to fix the attachment point
	 */
	std::vector<b2Joint*> joints_;

	/**
	 * The ground body connected with the attachment points
	 */
	std::vector<b2Body*> groundBody_;

};

}

#endif /* SIMULATOR_SOFT_ACTUATED_MEMBRANE_H_ */
