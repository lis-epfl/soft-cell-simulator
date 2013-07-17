/*
 * @(#) RandomWalk.cpp   1.0   Jun 14, 2013
 *
 * Jurg Germann (jurg.germann@epfl.ch)
 *
 * Copyright © 2011-2013 Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the Soft Cell Simulator (SCS) software.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include "RandomWalk.h"

#include "GL/glut.h"

#include <ostream>
#include <iostream>
#include <fstream>

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

RandomWalk::RandomWalk() {

	// position parameters
	center_ = 0.0f;
	edgeLength_ = 400.0f;
	radius_ = 60;

	srand(time(NULL)); // setup random generator

	this->softSimulator_ = new scs::SoftSimulatorEngine(this->m_world);
//	this->softSimulator_->setGlobalFriction(0.5);
//	this->softSimulator_->setGravity(b2Vec2(0, -10));

	CreateArena();

	// soft cell parameters
	softness_ = 6e6;
	amplitude_ = 500000.0f;
	interaction_radius_ = 25.0f;
	numberOfConnectionMech_ = 4;
	numberOfSoftElements_ = 5;

	scs::SoftElementDef elementDef;
	scs::SoftMembraneDef membraneDef;
	scs::ConnectionMechanismDef connectDef(interaction_radius_, amplitude_);

	elementDef.setNbrBodies(100);
	membraneDef.setSoftness(softness_);
	membraneDef.setRadius(radius_);
	//membraneDef.setDamping(100);

	// allocation
	std::list<b2Vec2> elemPos;

	// allocate soft elements vector
	std::vector<scs::SoftElement*> softElements;
	softElements.reserve(numberOfSoftElements_);

	// allocate connection mechanism vector
	std::vector<std::vector<scs::ConnectionMechanism*> > connectionMechanisms;
	connectionMechanisms.reserve(numberOfSoftElements_);
	for (int i = 0; i < numberOfSoftElements_; i++) {
		connectionMechanisms[i].reserve(numberOfConnectionMech_); // Add an empty row
	}

	for (int j = 0; j < numberOfSoftElements_; j++) {
		// add a new element coordinates and check its overlap with the other elements
		CheckOverlap(&elemPos);

		// set a position to the new element
		elementDef.setInitPosition(*elemPos.begin());
		//elementDef.setInitPosition(0,20);

		softElements[j] = new scs::SoftMembrane(
				this->softSimulator_->getWorld(), elementDef, membraneDef, j);
		this->softSimulator_->registerToWorld(softElements[j]);
		softElements_.push_back(softElements[j]);

		if (true) {

			// initialize magnetic connections
			for (int k = 0; k < numberOfConnectionMech_; k++) {
				connectionMechanisms[j][k] =
						new scs::MagneticConnectionMechanism(
								this->softSimulator_->getWorld(), connectDef);
				connectionMechanisms[j][k]->bindToSoftElement(softElements[j],
						k * 100 / numberOfConnectionMech_);
			}
		}

		// deformation
		scs::InternalConnection* intConnect = new scs::RopeInternalConnection(
				this->softSimulator_->getWorld(), 120.0);
		intConnect->bindConnection(softElements[j], 16, 46);
		this->softSimulator_->registerToWorld(intConnect);

	}

}

void RandomWalk::CreateArena() {

	float32 edgeLengthHalf = edgeLength_ / 2;

	// create a fenced area
	b2Body* ground = NULL;
	{
		b2BodyDef bd;
		bd.type = b2_staticBody;
		ground = this->softSimulator_->getWorld()->CreateBody(&bd);

		float32 topP = center_ + edgeLengthHalf + 5;
		float32 bottomP = center_ - edgeLengthHalf - 5;

		float32 tubeL = center_ - radius_;
		float32 tubeR = center_ + radius_;
		float32 tubeLength = topP + 3 * edgeLengthHalf;

		b2EdgeShape shape;
		// bottom wall
		shape.Set(b2Vec2(bottomP, bottomP), b2Vec2(topP, bottomP));
		ground->CreateFixture(&shape, 0.0f);
		// top wall
		shape.Set(b2Vec2(bottomP, topP), b2Vec2(tubeL, topP));
		ground->CreateFixture(&shape, 0.0f);
		shape.Set(b2Vec2(tubeR, topP), b2Vec2(topP, topP));
		ground->CreateFixture(&shape, 0.0f);
		// left upper wall
		shape.Set(b2Vec2(bottomP, bottomP), b2Vec2(bottomP, topP));
		ground->CreateFixture(&shape, 0.0f);
		// right upper wall
		shape.Set(b2Vec2(topP, topP), b2Vec2(topP, bottomP));
		ground->CreateFixture(&shape, 0.0f);
		// left wall tube to top wall
		shape.Set(b2Vec2(tubeL, topP), b2Vec2(tubeL, tubeLength));
		ground->CreateFixture(&shape, 0.0f);
		// right wall tube to top wall
		shape.Set(b2Vec2(tubeR, topP), b2Vec2(tubeR, tubeLength));
		ground->CreateFixture(&shape, 0.0f);

	}

}

/// Set random coordinates of a round element in square area:
b2Vec2* RandomWalk::GetRandPosition() {
	int px = rand() % (int) (edgeLength_ - 2 * radius_)
			- (edgeLength_ / 2 - radius_);
	int py = rand() % (int) (edgeLength_ - 2 * radius_)
			- (edgeLength_ / 2 - radius_);

	b2Vec2* _vec = new b2Vec2(px, py);
	return _vec;
}

/// Add a new coordinate of an element
/// Check a distance between a new and all the other elements is >= 2*radius (no overlap)
void RandomWalk::CheckOverlap(list<b2Vec2>* elemPos) {
	// check new position for overlapping with other elements
	b2Vec2 newP;
	int overlap = 1;

	if (elemPos->end() != elemPos->begin()) {
		do {
			newP = *GetRandPosition();
			for (list<b2Vec2>::iterator it = elemPos->begin();
					it != elemPos->end(); it++) {
				b2Vec2 itVal = *it;
				float32 distance = (itVal - newP).Length();
				if (distance <= 2 * radius_) {
					overlap = 1;
					break;
				}

				overlap = -1;
			}
		} while (overlap == 1);
		elemPos->push_front(newP);
	} else {
		newP = *GetRandPosition();
		elemPos->push_front(newP);
	}

}

/// Random walking: gives back a 2D column vector of velocities
b2Vec2* RandomWalk::RandomWalking() {
	float vx = 0;
	float vy = 0;
	int upperLimit = 100;
	int factor = 6; // speed factor

	// normal distribution
	vx = rand() % upperLimit - upperLimit / 2;
	vy = rand() % upperLimit - upperLimit / 2;

	vx = factor * vx / float(upperLimit - 1);
	vy = factor * vy / float(upperLimit - 1);

	b2Vec2* newPos = new b2Vec2(vx, vy);
	return newPos;
}

RandomWalk::~RandomWalk() {

}

void RandomWalk::Keyboard(unsigned char key) {

	Test::Keyboard(key);

	std::vector<scs::InternalConnection*> internalConnections =
			this->softSimulator_->getInternalConnections();

	switch (key) {
	case 'q':
		// decrease softness of elements
		for (int i = 0; i < numberOfSoftElements_; i++) {
			softElements_[i]->setSurface(softElements_[i]->getSurface() * 1.05);
		}
		break;
	case 'a':
		// increase softness of elements
		for (int i = 0; i < numberOfSoftElements_; i++) {
			softElements_[i]->setSurface(softElements_[i]->getSurface() * 0.95);
		}
		break;
	case 'd':
		// contract muscle

		for (int i = 0; i < internalConnections.size(); i++) {
			float maxLength =
					dynamic_cast<scs::RopeInternalConnection*>(internalConnections[i])->getMaxLength();
			dynamic_cast<scs::RopeInternalConnection*>(internalConnections[i])->setMaxLength(
					0.98 * maxLength);
		}
		break;
	case 'f':
		// relax muscle

		for (int i = 0; i < internalConnections.size(); i++) {
			float maxLength =
					dynamic_cast<scs::RopeInternalConnection*>(internalConnections[i])->getMaxLength();
			dynamic_cast<scs::RopeInternalConnection*>(internalConnections[i])->setMaxLength(
					1.02 * maxLength);
		}
		break;
	default:
		break;
	}
}

void RandomWalk::StepSimulator(Settings* /*settings*/, float /*timeStep*/) {

	srand(time(NULL)); // setup random generator

	// run the default physics and rendering
	this->softSimulator_->step();

	// render the connections
	RenderConnections();

	// write
	m_debugDraw.DrawString(3, m_textLine, "Change softness  with (q/a)");
	m_textLine += 15;
	m_debugDraw.DrawString(3, m_textLine,
			"Deformation: Contract/relax with (d/f)");
	m_textLine += 15;
	m_debugDraw.DrawString(5, m_textLine, "Stiffness:%.0f", softness_);
	m_textLine += 15;
	m_debugDraw.DrawString(5, m_textLine, "Interaction radius:%.0f",
			interaction_radius_);
	m_textLine += 15;
	m_debugDraw.DrawString(5, m_textLine, "Connection strength:%.0f",
			amplitude_);
	m_textLine += 15;
	m_debugDraw.DrawString(5, m_textLine, "Number of connections:%.0f",
			numberOfConnectionMech_);
	m_textLine += 15;

	// random walk
	std::vector<scs::SoftElement*> softElements =
			this->softSimulator_->getSoftElements();
	for (unsigned int k = 0; k < softElements.size(); k++) {
		this->softSimulator_->moveSoftElements(k, RandomWalking());
	}

}

Test* RandomWalk::Create() {
	return new RandomWalk();
}

void RandomWalk::RenderConnections() {
	// Rendering of the connection mechanisms
	std::vector<scs::ConnectionMechanism*> connectMechanism =
			this->softSimulator_->getConnectionMechanisms();

	for (unsigned int i = 0; i < connectMechanism.size(); i++) {

		if (dynamic_cast<scs::MagneticConnectionMechanism*>(connectMechanism[i])
				!= 0) {

			b2Body* connectedBody = connectMechanism[i]->getConnectedBody();
			int state = connectMechanism[i]->getState();

			int type = connectMechanism[i]->getType();

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
				//				cout << "active" << endl;
				break;
			case scs::ConnectionMechanism::PASSIVE:
				// white
				glColor3f(1, 1, 1);
				//				cout << "passive" << endl;
				break;
			case scs::ConnectionMechanism::INACTIVE:

				switch (type) {
				case 0:
					// yellow
					glColor3f(1, 1, 0.3);
					break;
				case 1:
					// blue
					glColor3f(0, 0, 1);
					break;
				case 2:
					// green
					glColor3f(1, 0.5, 0.3);
					break;
				case 3:
					// blue
					glColor3f(1, 0, 1);
					break;
				case 4:
					// yellow
					glColor3f(0.5, 1, 0.3);
					break;
				default:
					glColor3f(1, 0.5, 0.5);
					break;
				}
				break;
			default:
				glColor3f(1, 0.5, 0.5);
				break;
			}

			float radius =
					dynamic_cast<scs::MagneticConnectionMechanism*>(connectMechanism[i])->getLength();

			// Draw the same circle as for the fixture
			glBegin(GL_LINE_LOOP);
			for (float a = 0; a < 360 * DEGTORAD; a += 30 * DEGTORAD
			)
				glVertex2f(radius * sinf(a), radius * cosf(a));
			glEnd();

			glPopMatrix();

		}

	}

}
