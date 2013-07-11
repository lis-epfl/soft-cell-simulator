/*
 * @(#) SoftActuatedMembrane.cpp   1.0   Dec 20, 2011
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
#include "connections/internal/LinearActuatorInternalConnection.h"
#include "membrane/SoftActuatedMembrane.h"

#include <iostream>

namespace scs {

SoftActuatedMembrane::SoftActuatedMembrane(SoftMembrane* membrane,
		float thresholdA, float thresholdB, unsigned int connectionAnchor) :
		membrane_(membrane), thresholdA_(thresholdA), thresholdB_(thresholdB) {

	int nBodies = membrane->getBodies().size();

	// Create the internal connections

	// The main connection

	this->actuatedConnection_.push_back(
			new scs::LinearActuatorInternalConnection(membrane->getWorld(), 1,
					50, 1, 0.2, 30));

	int startPoint = connectionAnchor;
	if (startPoint < 0) {
		startPoint += nBodies;
	}
	startPoint = startPoint % nBodies;

	int endPoint = (connectionAnchor + nBodies / 2);
	if (endPoint < 0) {
		endPoint += nBodies;
	}
	endPoint = endPoint % nBodies;

	int curPos = 0;

	this->actuatedConnection_[this->actuatedConnection_.size() - 1]->bindConnection(
			membrane, startPoint, endPoint);
	curPos++;

	for (int i = 0; i < nBodies / 4 - 1; ++i) {
		this->actuatedConnection_.push_back(
				new scs::LinearActuatorInternalConnection(membrane->getWorld(),
						1, 50, 1, 0.4, 30));

		startPoint = connectionAnchor + i + 1;
		if (startPoint < 0) {
			startPoint += nBodies;
		}
		startPoint = startPoint % nBodies;

		endPoint = (connectionAnchor + i + 1 + nBodies / 2 - (i + 1) * 2);
		if (endPoint < 0) {
			endPoint += nBodies;
		}
		endPoint = endPoint % nBodies;

		this->actuatedConnection_[this->actuatedConnection_.size() - 1]->bindConnection(
				membrane, startPoint, endPoint);
		curPos++;
	}

	for (int i = 0; i < nBodies / 4 - 1; ++i) {
		this->actuatedConnection_.push_back(
				new scs::LinearActuatorInternalConnection(membrane->getWorld(),
						1, 50, 1, 0.4, 30));
		startPoint = connectionAnchor - i - 1;
		if (startPoint < 0) {
			startPoint += nBodies;
		}
		startPoint = startPoint % nBodies;

		endPoint = (connectionAnchor - i - 1 + nBodies / 2 + (i + 1) * 2);
		if (endPoint < 0) {
			endPoint += nBodies;
		}
		endPoint = endPoint % nBodies;

		this->actuatedConnection_[this->actuatedConnection_.size() - 1]->bindConnection(
				membrane, startPoint, endPoint);
	}

	float attachmentSurface = 0.05;
	int nAttachmentPoints = nBodies * attachmentSurface;
	int startAttachmentPoint = connectionAnchor - nAttachmentPoints / 2;
	if (startAttachmentPoint < 0) {
		startAttachmentPoint = nBodies + startAttachmentPoint;
	}

	for (int i = 0; i < nAttachmentPoints; ++i) {
		this->attachmentPoints_.push_back(
				membrane->getBodies()[(startAttachmentPoint + i) % nBodies]);
	}

	this->groundBody_.resize(nAttachmentPoints);
	this->joints_.resize(nAttachmentPoints);

	this->curState_ = DETACHED;
}

SoftActuatedMembrane::~SoftActuatedMembrane() {

	// If it's ATTACHED
	if (this->curState_ == ATTACHED) {

		for (unsigned int i = 0; i < this->joints_.size(); ++i) {
			this->membrane_->getWorld()->DestroyJoint(this->joints_[i]);
			this->membrane_->getWorld()->DestroyBody(this->groundBody_[i]);
		}

	}

	for (unsigned int i = 0; i < this->actuatedConnection_.size(); ++i) {
		delete this->actuatedConnection_[i];
	}

}

void SoftActuatedMembrane::increase() {
	for (unsigned int i = 0; i < this->actuatedConnection_.size(); ++i) {
		this->actuatedConnection_[i]->increase();
	}
	this->updateAttachmentState();
}

void SoftActuatedMembrane::decrease() {
	for (unsigned int i = 0; i < this->actuatedConnection_.size(); ++i) {
		this->actuatedConnection_[i]->decrease();
	}
	this->updateAttachmentState();
}

void SoftActuatedMembrane::setActuatorLength(float length) {
	this->actuatedConnection_[0]->setLength(length);
	this->updateAttachmentState();
}

void SoftActuatedMembrane::updateAttachmentState() {

	if (this->curState_ == ATTACHED
			&& this->actuatedConnection_[0]->getLength() < this->thresholdB_) {

		// Detach!
		this->curState_ = DETACHED;

		for (unsigned int i = 0; i < this->joints_.size(); ++i) {
			this->membrane_->getWorld()->DestroyJoint(this->joints_[i]);
			this->membrane_->getWorld()->DestroyBody(this->groundBody_[i]);
		}

	} else if (this->curState_ == DETACHED
			&& this->actuatedConnection_[0]->getLength() > this->thresholdA_) {

		// Attach!
		this->curState_ = ATTACHED;

		for (unsigned int i = 0; i < this->attachmentPoints_.size(); ++i) {

			const b2Vec2& curBodyPos =
					this->attachmentPoints_[i]->GetPosition();

			// Create a new ground body
			b2BodyDef groundBodyDef;
			groundBodyDef.position.Set(curBodyPos.x, curBodyPos.y);
			b2Body* groundBody = this->membrane_->getWorld()->CreateBody(
					&groundBodyDef);

			// Create a new joint
			b2DistanceJointDef jointDef;
			jointDef.Initialize(groundBody, this->attachmentPoints_[i],
					groundBody->GetWorldCenter(),
					this->attachmentPoints_[i]->GetWorldCenter());
			jointDef.length = 0;
			jointDef.collideConnected = false;
			b2Joint* joint = this->membrane_->getWorld()->CreateJoint(
					&jointDef);

			this->groundBody_[i] = groundBody;
			this->joints_[i] = joint;

		}

	}

}

}
