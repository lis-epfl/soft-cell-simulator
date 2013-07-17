/*
 * @(#) SoftElement.h   1.0   Dec 14, 2011
 *
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
#ifndef SCS_H_
#define SCS_H_

#include "connections/external/ConnectionMechanismDef.h"
#include "connections/external/ConnectionMechanism.h"
#include "connections/external/ConnectionPair.h"
#include "connections/external/MagneticConnectionMechanism.h"

#include "connections/internal/FixedInternalConnection.h"
#include "connections/internal/LinearActuatorInternalConnection.h"
#include "connections/internal/InternalConnection.h"
#include "connections/internal/RopeInternalConnection.h"

#include "membrane/SoftActuatedMembrane.h"
#include "membrane/SoftMembraneDef.h"
#include "membrane/SoftMembrane.h"
#include "membrane/SoftRopeMembrane.h"
#include "membrane/SoftWeldMembrane.h"

#include "SoftElementDef.h"
#include "SoftElement.h"
#include "SoftSimulatorEngine.h"

#endif /* SCS_H_ */
