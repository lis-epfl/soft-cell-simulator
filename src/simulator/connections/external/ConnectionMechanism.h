/*
 * @(#) ConnectionMechanism.h   1.0   Dec 14, 2011
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
#ifndef SIMULATOR_CONNECTION_MECHANISM_H_
#define SIMULATOR_CONNECTION_MECHANISM_H_

#include "Box2D/Box2D.h"

namespace scs {

class ConnectionMechanismDef;
class SoftElement;

class ConnectionMechanism {

public:

	// Definitions of sensor with states: active, passive and inactive, state is saved
	// in UserData of each body
	enum SensorState {
		SENSOR = 1, ACTIVE = 10, PASSIVE = 11, INACTIVE = 12
	};

	ConnectionMechanism(b2World* world,
			const ConnectionMechanismDef& definition);

	virtual ~ConnectionMechanism();

	ConnectionMechanismDef& getDefinition();

	b2World* getWorld();

	int getState() const;

	int getType() const;

	int getNbrContacts() const;

	void setState(int state);

	void setType(int type);

	void setNbrContacts(int nbrContacts);

	virtual void bindToSoftElement(SoftElement* softElement,
			unsigned int bodyIndex) = 0;

	virtual void applyForces(ConnectionMechanism* otherPoint) = 0;

	virtual b2Body* getConnectedBody() = 0;

	virtual int getElementId() const = 0;

	virtual b2Body* getConnectPoint() = 0;

private:

	b2World* world_;

	ConnectionMechanismDef definition_;

	// designates the state in which the connection mechanism is
	int state_;

	// designates the type of the connection mechanism
	int type_;

	// number of contacts (in case there are more than one at a time)
	int nbrContacts_;

};

}

#endif /* SIMULATOR_CONNECTION_MECHANISM_H_ */
