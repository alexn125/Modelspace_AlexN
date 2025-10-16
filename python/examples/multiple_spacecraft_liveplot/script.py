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
from modelspace.ModelSpacePy import Time, SimulationExecutive, Frame, Node, Body, CartesianVector3, CsvLogger, connectSignals
from modelspace.Spacecraft import Spacecraft
from modelspace.OrbitalElementsStateInit import OrbitalElementsStateInit
from modelspace.SpicePlanet import SpicePlanet
from modelspace.FrameStateSensorModel import FrameStateSensorModel
from modelspaceutils.vizkit.VizKitPlanetRelative import VizKitPlanetRelative
from modelspaceutils.vizkit.VizKitGroundTrack import VizKitGroundTrack

# Create a simple simulation to plot a spacecraft
exc = SimulationExecutive()     # Create our executive -- by convention named exc
exc.args().addDefaultArgument("end",     30000)
exc.parseArgs(sys.argv)         # this interperets command-line inputs

# Create a planet and spacecraft
earth = SpicePlanet(exc, "earth")

oe_state = OrbitalElementsStateInit(exc, "oe")
oe_state2 = OrbitalElementsStateInit(exc, "oe2")

sc = Spacecraft(exc, "sc")
connectSignals(oe_state.outputs.pos__inertial, sc.params.initial_position)
connectSignals(oe_state.outputs.vel__inertial, sc.params.initial_velocity)
sc.params.planet_ptr(earth)

sc2 = Spacecraft(exc, "sc2")
connectSignals(oe_state2.outputs.pos__inertial, sc2.params.initial_position)
connectSignals(oe_state2.outputs.vel__inertial, sc2.params.initial_velocity)
sc2.params.planet_ptr(earth)

vk = VizKitPlanetRelative(exc)
vk.target(sc.outputs.body())
vk.planet(earth.outputs.inertial_frame())
vk.addSpacecraft(sc2.outputs.body())
exc.logManager().addLog(vk, Time(100))

gt = VizKitGroundTrack(exc)
connectSignals(sc.outputs.latitude_detic, gt.lat)
connectSignals(sc.outputs.longitude, gt.lon)
exc.logManager().addLog(gt, Time(100))

oe_state.params.a(9314000.0)
oe_state.params.e(0.3)
oe_state.params.i(0.4)
oe_state.params.RAAN(0.0)
oe_state.params.w(0.0)
oe_state.params.f(0.0)

oe_state2.params.a(8314000.0)
oe_state2.params.e(0.1)
oe_state2.params.i(0.2)
oe_state2.params.RAAN(0.3)
oe_state2.params.w(0.1)
oe_state2.params.f(1.0)

exc.startup()

exc.run()