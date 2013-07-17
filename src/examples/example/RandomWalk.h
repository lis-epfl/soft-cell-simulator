/*
 * @(#) RandomWalk.h   1.0   Jun 14, 2013
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
#ifndef RANDOM_WALK_H_
#define RANDOM_WALK_H_

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <list>

#include <ostream>
#include <fstream>

using namespace std;

#include "../testbed/Test.h"

#include "Box2D/Box2D.h"

#include "SCS.h"

#define deg2rad 0.0174532925199432957f
#define rad2deg 57.295779513082320876f

class RandomWalk: public Test {

public:

	RandomWalk();

	virtual ~RandomWalk();

	void Keyboard(unsigned char key);

	void StepSimulator(Settings* settings, float timeStep);

	static Test* Create();

	void SetTextLine(int32 line);

	void DrawTitle(int x, int y, const char *string);

	void RenderConnections();

	b2Vec2* RandomWalking();

	void CheckOverlap(list<b2Vec2>* elemPos);

	void CreateArena();

private:

	scs::LinearActuatorInternalConnection* intConnect3_;

	scs::LinearActuatorInternalConnection* intConnect4_;

	scs::SoftSimulatorEngine* softSimulator_;

	std::vector<scs::SoftElement*> softElements_;

	int numberOfSoftElements_;

	scs::SoftActuatedMembrane* actuatedMembrane_;

	b2Vec2* GetRandPosition();

	// position parameters
	float center_;
	float edgeLength_;
	int radius_;

	// factors
	float softness_;
	float amplitude_;
	float interaction_radius_;
	float numberOfConnectionMech_;

	// Log to file
	ofstream myfile_;
};

#endif /* RANDOM_WALK_H_ */
