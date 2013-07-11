/*
 * @(#) MagneticConnectionMechanism.h   1.0   Dec 14, 2011
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
#ifndef SIMULATOR_MAGNETIC_CONNECTION_MECHANISM_H_
#define SIMULATOR_MAGNETIC_CONNECTION_MECHANISM_H_

#include "connections/external/ConnectionMechanism.h"

namespace scs {

class MagneticConnectionMechanism: public ConnectionMechanism {

public:

	MagneticConnectionMechanism(b2World* world,
			const ConnectionMechanismDef& definition);

	virtual ~MagneticConnectionMechanism();

	void bindToSoftElement(SoftElement* softElement, unsigned int bodyIndex);

	void applyForces(ConnectionMechanism* otherPoint);

	float getLength() const;

	float getWidth() const;

	b2Body* getConnectedBody();

	int getElementId() const;

	b2Body* getConnectPoint();

private:

	// body of the connectPoint
	b2Body* connectPoint_;

	// body of the soft element which the conneciton point is connected to
	b2Body* connectedBody_;

	// designates the number of the element which the connection mechanism is on
	int elementId_;

	float length_;

	float width_;

};

}

#endif /* SIMULATOR_MAGNETIC_CONNECTION_MECHANISM_H_ */
