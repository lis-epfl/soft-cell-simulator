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
#include "connections/external/ConnectionMechanismDef.h"
#include "connections/external/ConnectionMechanism.h"
#include "connections/external/ConnectionPair.h"
#include "connections/external/MagneticConnectionMechanism.h"
#include "connections/internal/InternalConnection.h"

#include "SoftElementDef.h"
#include "SoftElement.h"
#include "SoftSimulatorEngine.h"
#include <time.h>

#define deg2rad 0.0174532925199432957f
#define rad2deg 57.295779513082320876f

#include <iostream>
using namespace std;

using std::cout;
using std::endl;

namespace scs {

SoftSimulatorEngine::SoftSimulatorEngine() :
		freeWorld_(true) {

	// normal is 60.0, bigger timestep will make engine less accurate
	this->engineTimeStep_ = 1.0f / 60.0f;
	// JG: change from 8 to 200, and 3 to 100
	this->engineVelocityIterations_ = 200;
	this->enginePositionIterations_ = 100;

	b2Vec2 gravity;
	gravity.Set(0.0f, 0.0f);

	world_ = new b2World(gravity);

	stepCount_ = 0;

	world_->SetContactListener(this);
}

SoftSimulatorEngine::SoftSimulatorEngine(b2World* world) :
		freeWorld_(false) {

	this->engineTimeStep_ = 1.0f / 60.0f;
	// JG: change from 8 to 200, and 3 to 100
	this->engineVelocityIterations_ = 200;
	this->enginePositionIterations_ = 100;

	this->world_ = world;
	b2Vec2 gravity(0.0f, 0.0f);
	world_->SetGravity(gravity);

	stepCount_ = 0;

	world_->SetContactListener(this);
}

SoftSimulatorEngine::~SoftSimulatorEngine() {

	// Free all the managed objects
	for (unsigned int i = 0; i < connectPairs_.size(); ++i) {
		delete connectPairs_[i];
	}

	if (freeWorld_) {
		delete world_;
		world_ = NULL;
	}

}

// implement a contact listener in this class
void SoftSimulatorEngine::BeginContact(b2Contact* contact) {

	//check if two sensors contact and if they are not on the same element
	ConnectionMechanism* connectMechA =
			(ConnectionMechanism*) contact->GetFixtureA()->GetBody()->GetUserData();
	ConnectionMechanism* connectMechB =
			(ConnectionMechanism*) contact->GetFixtureB()->GetBody()->GetUserData();

	// check if both connection mechanism are real
	if ((connectMechA != 0) && (connectMechB != 0)) {

			// check if both connections are NOT PASSIVE
			if (!(connectMechA->getState() == ConnectionMechanism::PASSIVE
					|| connectMechB->getState() == ConnectionMechanism::PASSIVE)) {

				if ((connectMechA->getElementId())
						!= (connectMechB->getElementId())) {
					// add the two connection bodies to ConnectPairs
					ConnectPair* pair = new ConnectPair;
					pair->first = connectMechA;
					pair->second = connectMechB;
					connectPairs_.push_back(pair);

					// set the connection state to ACTIVE
					connectMechA->setState(ConnectionMechanism::ACTIVE);
					connectMechB->setState(ConnectionMechanism::ACTIVE);

					// update the number of contacts for both connection points
					connectMechA->setNbrContacts(
							connectMechA->getNbrContacts() + 1);
					connectMechB->setNbrContacts(
							connectMechB->getNbrContacts() + 1);
				}
			}
	}
}

// for the end of a contact
void SoftSimulatorEngine::EndContact(b2Contact* contact) {

	//check if two sensors contact and if they are not on the same element
	ConnectionMechanism* connectMechA =
			(ConnectionMechanism*) contact->GetFixtureA()->GetBody()->GetUserData();
	ConnectionMechanism* connectMechB =
			(ConnectionMechanism*) contact->GetFixtureB()->GetBody()->GetUserData();

	// if both connections are not empty
	if ((connectMechA != 0) && (connectMechB != 0)) {

			// check if both connections are NOT PASSIVE
			if (!(connectMechA->getState() == ConnectionMechanism::PASSIVE
					|| connectMechB->getState() == ConnectionMechanism::PASSIVE)) {

				//check that connections are not from same element
				if ((connectMechA->getElementId())
						!= (connectMechB->getElementId())) {

					if (this->connectPairs_.size() > 0) {

						// find the two connection bodies in ConnectPairs and delete them
						int connectionMechanismIx = -1;
						for (unsigned int i = 0; i < this->connectPairs_.size();
								++i) {
							if (this->connectPairs_[i]->first == connectMechA
									&& this->connectPairs_[i]->second
											== connectMechB) {
								connectionMechanismIx = i;
								break;
							} else if (this->connectPairs_[i]->first
									== connectMechB
									&& this->connectPairs_[i]->second
											== connectMechA) {
								connectionMechanismIx = i;
								break;
							}
						}

						connectPairs_.erase(
								connectPairs_.begin() + connectionMechanismIx);

						// update the number of contacts for both connection points
						connectMechA->setNbrContacts(
								connectMechA->getNbrContacts() - 1);
						connectMechB->setNbrContacts(
								connectMechB->getNbrContacts() - 1);

						// if there is no contact anymore,set the connection state to INACTIVE
						if (connectMechA->getNbrContacts() <= 0) {
							connectMechA->setState(
									ConnectionMechanism::INACTIVE);
							connectMechA->setNbrContacts(0);
						}
						if (connectMechB->getNbrContacts() <= 0) {
							connectMechB->setState(
									ConnectionMechanism::INACTIVE);
							connectMechB->setNbrContacts(0);
						}
					}

				}
			}
	}
}

void SoftSimulatorEngine::registerToWorld(SoftElement* element) {
	softElements_.push_back(element);
}

void SoftSimulatorEngine::registerToWorld(ConnectionMechanism* connectMech) {
	connectionMechanisms_.push_back(connectMech);
}

void SoftSimulatorEngine::registerToWorld(InternalConnection* intConnect) {
	internalConnections_.push_back(intConnect);
}

void SoftSimulatorEngine::step(float timestep) {

	world_->SetWarmStarting(true);
	world_->SetContinuousPhysics(true);

	// update all the elements and apply internal pressure if needed
	for (unsigned int i = 0; i < softElements_.size(); i++) {
		softElements_[i]->computeDynamics(globalFriction_);
	}

	// apply all connection forces
	for (unsigned int i = 0; i < connectPairs_.size(); i++) {
		connectPairs_[i]->first->applyForces(connectPairs_[i]->second);
	}

	world_->Step(timestep, this->engineVelocityIterations_,
			this->enginePositionIterations_);

	++stepCount_;

}

void SoftSimulatorEngine::step() {
	this->step(this->engineTimeStep_);
}

b2World* SoftSimulatorEngine::getWorld() {
	return this->world_;
}

std::vector<SoftElement*>& SoftSimulatorEngine::getSoftElements() {
	return this->softElements_;
}

std::vector<ConnectionMechanism*>& SoftSimulatorEngine::getConnectionMechanisms() {
	return this->connectionMechanisms_;
}

std::vector<ConnectPair*>& SoftSimulatorEngine::getConnectPairs() {
	return this->connectPairs_;
}

std::vector<InternalConnection*>& SoftSimulatorEngine::getInternalConnections() {
	return this->internalConnections_;
}

void SoftSimulatorEngine::setGlobalFriction(float globalFriction) {
	globalFriction_ = globalFriction;
}

void SoftSimulatorEngine::setExternalFlow(b2Vec2 flow) {
	float velocity = flow.Normalize();
	world_->SetGravity(flow);
	this->setGlobalFriction(1.0f / velocity);
}

void SoftSimulatorEngine::setGravity(b2Vec2 gravity) {
	this->world_->SetGravity(gravity);
}

float SoftSimulatorEngine::getTimestep() {
	return this->engineTimeStep_;
}

int32 SoftSimulatorEngine::getStepCount() {
	return this->stepCount_;
}

}
