/*
 * @(#) SoftElementTest.h   1.0   Dec 14, 2011
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
#ifndef TESTBED_SOFT_ELEMENT_TEST_H_
#define TESTBED_SOFT_ELEMENT_TEST_H_

#include <iostream>
#include <vector>

#include "../testbed/Test.h"

#include "Box2D/Box2D.h"

#include "SCS.h"

#define deg2rad 0.0174532925199432957f
#define rad2deg 57.295779513082320876f

class SoftElementTest: public Test {

public:

	SoftElementTest();

	virtual ~SoftElementTest();

	void Keyboard(unsigned char key);

	void StepSimulator(Settings* settings, float timeStep);

	static Test* Create();

	void SetTextLine(int32 line);

	void DrawTitle(int x, int y, const char *string);

private:

	scs::LinearActuatorInternalConnection* intConnect3_;

	scs::LinearActuatorInternalConnection* intConnect4_;

	scs::SoftSimulatorEngine* softSimulator_;

	std::vector<scs::SoftElement*> softElements_;

	scs::SoftActuatedMembrane* actuatedMembrane_;
};

#endif /* TESTBED_SOFT_ELEMENT_TEST_H_ */
