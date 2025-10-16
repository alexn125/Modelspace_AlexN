###############################################################################
# Copyright (c) ATTX LLC 2024. All Rights Reserved.
#
# This software and associated documentation (the "Software") are the 
# proprietary and confidential information of ATTX, LLC. The Software is 
# furnished under a license agreement between ATTX and the user organization 
# and may be used or copied only in accordance with the terms of the agreement.
# Refer to 'license/attx_license.adoc' for standard license terms.
#
# EXPORT CONTROL NOTICE: THIS SOFTWARE MAY INCLUDE CONTENT CONTROLLED UNDER THE
# INTERNATIONAL TRAFFIC IN ARMS REGULATIONS (ITAR) OR THE EXPORT ADMINISTRATION 
# REGULATIONS (EAR99). No part of the Software may be used, reproduced, or 
# transmitted in any form or by any means, for any purpose, without the express 
# written permission of ATTX, LLC.
###############################################################################
"""
Simple LivePlot tutorial
-------------------------
This is an example code that uses the Spacecraft construct 
and propagates a spacecraft orbit for 100 seconds.
It liveplots the spacecraft orbit in 2D.

Author: Alex Reynolds
"""
import sys
from modelspace.ModelSpacePy import (Time, SimulationExecutive, Frame, Node, 
                                     Body, CartesianVector3, CsvLogger, connectSignals,
                                     DEGREES_TO_RADIANS)
from modelspace.Spacecraft import Spacecraft
from modelspace.OrbitalElementsStateInit import OrbitalElementsStateInit
from modelspace.SpicePlanet import SpicePlanet
from modelspace.FrameStateSensorModel import FrameStateSensorModel
from modelspaceutils.vizkit.VizKitConstellationLink import VizKitConstellationLink

# Create a simple simulation to plot a spacecraft
exc = SimulationExecutive()     # Create our executive -- by convention named exc
exc.args().addDefaultArgument("end",     30000)
exc.parseArgs(sys.argv)         # this interperets command-line inputs

# Create a planet and spacecraft
earth = SpicePlanet(exc, "earth")

oe_state = OrbitalElementsStateInit(exc, "oe")

sc = Spacecraft(exc, "sc")
connectSignals(oe_state.outputs.pos__inertial, sc.params.initial_position)
connectSignals(oe_state.outputs.vel__inertial, sc.params.initial_velocity)
sc.params.planet_ptr(earth)

vk = VizKitConstellationLink(exc)
vk.target(sc.outputs.body())
vk.planet(earth.outputs.inertial_frame())
vk.max_range(10000000)
vk.max_range_rate(10000)
exc.logManager().addLog(vk, Time(100))

oe_state.params.a(9314000.0)
oe_state.params.e(0.3)
oe_state.params.i(0.4)
oe_state.params.RAAN(0.0)
oe_state.params.w(0.0)
oe_state.params.f(0.0)

scs = []
oes = []
for i in range(10):
    oes.append(OrbitalElementsStateInit(exc, "oe_" + str(i)))
    scs.append(Spacecraft(exc, "sc_" + str(i)))
    vk.addSpacecraft(scs[i].outputs.body())
    
    connectSignals(oes[i].outputs.pos__inertial, scs[i].params.initial_position)
    connectSignals(oes[i].outputs.vel__inertial, scs[i].params.initial_velocity)
    scs[i].params.planet_ptr(earth)
    
    oes[i].params.a(8000000)
    oes[i].params.e(0.01)
    oes[i].params.i(97.0*DEGREES_TO_RADIANS)
    oes[i].params.RAAN(9.0*i*DEGREES_TO_RADIANS)
    oes[i].params.w(1.0)
    oes[i].params.f(2.0*i)

exc.startup()

exc.run()