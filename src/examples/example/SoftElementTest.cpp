/*
 * @(#) SoftElementTest.cpp   1.0   Dec 14, 2011
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
#include "SoftElementTest.h"

#include "GL/glut.h"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

SoftElementTest::SoftElementTest() {

	this->softSimulator_ = new scs::SoftSimulatorEngine(this->m_world);

	this->softSimulator_->setGlobalFriction(0.1);
	this->softSimulator_->setGravity(b2Vec2(0, -10));

	// create a fenced area
	b2Body* ground = NULL;
	{
		b2BodyDef bd;
		bd.type = b2_staticBody;
		ground = this->softSimulator_->getWorld()->CreateBody(&bd);

		b2EdgeShape shape;
		shape.Set(b2Vec2(-30.0f, -30.0f), b2Vec2(30.0f, -30.0f));
		ground->CreateFixture(&shape, 0.0f);
		shape.Set(b2Vec2(-30.0f, -30.0f), b2Vec2(-30.0f, 30.0f));
		ground->CreateFixture(&shape, 0.0f);
		shape.Set(b2Vec2(-30.0f, 30.0f), b2Vec2(30.0f, 30.0f));
		ground->CreateFixture(&shape, 0.0f);
		shape.Set(b2Vec2(30.0f, 30.0f), b2Vec2(30.0f, -30.0f));
		ground->CreateFixture(&shape, 0.0f);

	}

	scs::SoftElementDef elementDef;
	scs::SoftMembraneDef membraneDef;
	scs::ConnectionMechanismDef connectDef;

	// Create 5 Soft Elements with internal pressure as previously defined
	for (unsigned int i = 0; i < 8; i++) {

		b2Vec2 pos((-2.0f) * 12.0f, 10.0f);

		int alpha = 45;
		int pi = 3.41;

		pos.x = 12 * cos(alpha * i * pi / 180);
		pos.y = 12 * sin(alpha * i * pi / 180);

		elementDef.setInitPosition(pos);
		elementDef.setFriction(0.2 * (i + 1));
		scs::SoftElement* softElement = new scs::SoftMembrane(
				this->softSimulator_->getWorld(), elementDef, membraneDef,
				i + 1);
		this->softSimulator_->registerToWorld(softElement);

		this->softElements_.push_back(softElement);

		scs::ConnectionMechanism* connectMech =
				new scs::MagneticConnectionMechanism(
						this->softSimulator_->getWorld(), connectDef);
		connectMech->bindToSoftElement(softElement, 5 * (1 + i));
		this->softSimulator_->registerToWorld(connectMech);
	}

	std::cout << this->softElements_[0]->getCenter().x << "-"
			<< this->softElements_[1]->getCenter().x << std::endl;

	dynamic_cast<scs::SoftMembrane*>(this->softElements_[0])->mergeMembrane("r",
			dynamic_cast<scs::SoftMembrane*>(this->softElements_[1]), 0.05);

	this->actuatedMembrane_ = new scs::SoftActuatedMembrane(
			dynamic_cast<scs::SoftMembrane*>(this->softElements_[1]), 0.98, 0.7,
			0);

}

SoftElementTest::~SoftElementTest() {

}

void SoftElementTest::Keyboard(unsigned char key) {

	Test::Keyboard(key);
	switch (key) {
	case 'q':

		actuatedMembrane_->increase();
		break;
	case 'a':
		actuatedMembrane_->decrease();
		break;
	default:
		break;
	}
}

void SoftElementTest::StepSimulator(Settings* /*settings*/,
		float /*timeStep*/) {

	// run the default physics and rendering
	this->softSimulator_->step();

	// Rendering of the connection mechanisms
	std::vector<scs::ConnectionMechanism*> connectMechanism =
			this->softSimulator_->getConnectionMechanisms();

	for (unsigned int i = 0; i < connectMechanism.size(); i++) {

		if (dynamic_cast<scs::MagneticConnectionMechanism*>(connectMechanism[i])
				!= 0) {

			b2Body* connectedBody = connectMechanism[i]->getConnectedBody();
			int state = connectMechanism[i]->getState();

			//get current position of the body connected to the Sensor/ConnectBody
			b2Vec2 pos = connectedBody->GetPosition();
			float angle = connectedBody->GetAngle();

			//call normal render at different position/rotation
			glPushMatrix();
			glTranslatef(pos.x, pos.y, 0);
			glRotatef(angle * rad2deg, 0, 0, 1); //OpenGL uses degrees here

			switch (state) {
			case scs::ConnectionMechanism::ACTIVE:
				//red
				glColor3f(1, 0, 0);
				break;
			case scs::ConnectionMechanism::PASSIVE:
				// white
				glColor3f(1, 1, 1);
				break;
			case scs::ConnectionMechanism::INACTIVE:
				// yellow
				glColor3f(1, 1, 0.3);
				break;
			default:
				glColor3f(1, 0.5, 0.5);
				break;
			}

			float radius =
					dynamic_cast<scs::MagneticConnectionMechanism*>(connectMechanism[i])->getLength();

			// Draw the same circle as for the fixture
			glBegin(GL_LINE_LOOP);
			for (float a = 0; a < 360 * DEGTORAD; a += 30 * DEGTORAD) {
				glVertex2f(radius * sinf(a), radius * cosf(a));
			}
			glEnd();

			glPopMatrix();

		}

	}

	// write
	m_textLine += 8;
	m_debugDraw.DrawString(3, m_textLine, "Change joint length  with (q/a)");

}

Test* SoftElementTest::Create() {
	return new SoftElementTest();
}

