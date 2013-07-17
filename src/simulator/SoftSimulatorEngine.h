/*
 * @(#) SoftSimulatorEngine.h   1.0   Dec 14, 2011
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Manuel Stockli (manuel.stockli@epfl.ch)
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
#ifndef SIMULATOR_SOFT_SIMULATOR_ENGINE_H_
#define SIMULATOR_SOFT_SIMULATOR_ENGINE_H_

#include <vector>

#include "Box2D/Box2D.h"

namespace scs {

class SoftElement;
class ConnectionMechanism;
class ConnectPair;
class InternalConnection;

class SoftSimulatorEngine: public b2ContactListener {

public:

	SoftSimulatorEngine();
	SoftSimulatorEngine(b2World* world);

	virtual ~SoftSimulatorEngine();

	void registerToWorld(SoftElement* element);
	void registerToWorld(ConnectionMechanism* connectMech);
	void registerToWorld(InternalConnection* intConnect);

	void step(float timestep);
	void step();

	void BeginContact(b2Contact* contact);
	void EndContact(b2Contact* contact);

	b2World* getWorld();

	std::vector<SoftElement*>& getSoftElements();
	std::vector<ConnectionMechanism*>& getConnectionMechanisms();
	std::vector<ConnectPair*>& getConnectPairs();
	std::vector<InternalConnection*>& getInternalConnections();

	void setGlobalFriction(float globalFriction);

	void setExternalFlow(b2Vec2 flow);

	void setGravity(b2Vec2 gravity);

	float getTimestep();

	int32 getStepCount();
	
	void moveSoftElements(int softElements, b2Vec2* velocity);

private:

	float engineTimeStep_;

	unsigned int engineVelocityIterations_;

	unsigned int enginePositionIterations_;

	// worlds global friction parameter
	float globalFriction_;

	// vector with all elements created for this world
	std::vector<SoftElement*> softElements_;

	std::vector<ConnectionMechanism*> connectionMechanisms_;

	std::vector<ConnectPair*> connectPairs_;

	std::vector<InternalConnection*> internalConnections_;

	b2Body* groundBody_;

	b2World* world_;
	bool freeWorld_;

	int32 stepCount_;

};

}

#endif /* SIMULATOR_SOFT_SIMULATOR_ENGINE_H_ */
