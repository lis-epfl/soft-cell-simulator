/*
 * @(#) ConnectionMechanism.cpp   1.0   Dec 14, 2011
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
#include "connections/external/ConnectionMechanismDef.h"
#include "connections/external/ConnectionMechanism.h"

namespace scs {

ConnectionMechanism::ConnectionMechanism(b2World* world,
		const ConnectionMechanismDef& definition) :
		world_(world), definition_(definition), state_(
				ConnectionMechanism::INACTIVE), nbrContacts_(0) {

}

ConnectionMechanism::~ConnectionMechanism() {

}

b2World* ConnectionMechanism::getWorld() {
	return world_;
}

ConnectionMechanismDef& ConnectionMechanism::getDefinition() {
	return definition_;
}

int ConnectionMechanism::getState() const {
	return state_;
}

int ConnectionMechanism::getType() const {
	return type_;
}

int ConnectionMechanism::getNbrContacts() const {
	return nbrContacts_;
}

void ConnectionMechanism::setState(int state) {
	state_ = state;
}

void ConnectionMechanism::setType(int type) {
	type_ = type;
}

void ConnectionMechanism::setNbrContacts(int nbrContacts) {
	nbrContacts_ = nbrContacts;
}

}

