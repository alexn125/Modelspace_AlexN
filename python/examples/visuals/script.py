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
Simple Visuals Tutorial
-----------------------
This script demonstrates using the built-in high quality simulation
visuals model. It builds a standard simulation with a single spacecraft and
ground station, adds them to visuals, and runs the simulation.

Author: Alex Reynolds
"""
import sys
from modelspace.Spacecraft import Spacecraft
from modelspace.OrbitalElementsStateInit import OrbitalElementsStateInit
from modelspace.ModelSpacePy import SimulationExecutive, CsvLogger, DEGREES_TO_RADIANS, connectSignals
from modelspace.SpicePlanet import SpicePlanet
from modelspace.FrameStateSensorModel import FrameStateSensorModel
from modelspace.PlanetRelativeStatesModel import PlanetRelativeStatesModel
from modelspace.VisualsModel import VisualsModel
from modelspace.GroundStationModel import GroundStationModel

exc = SimulationExecutive()     # Create our executive -- by convention named exc
exc.parseArgs(sys.argv)         # this interperets command-line inputs
exc.setRateHz(1)                #  We can setRateHz or setRateSec -- default is 1 second

# Calling exc.enableVisuals() is necessary for simulation visuals to work. This
# call creates the (internal) visuals model and allows it to run. Calling visuals
# functions without having called enableVisuals will result in an error, as the
# visuals model would not exist.
exc.enableVisuals()

earth = SpicePlanet(exc, "earth")

oe_state = OrbitalElementsStateInit(exc)

sc = Spacecraft(exc, "sc")
connectSignals(oe_state.outputs.pos__inertial, sc.params.initial_position)
connectSignals(oe_state.outputs.vel__inertial, sc.params.initial_velocity)
connectSignals(earth.outputs.self_id, sc.params.planet_ptr)

# Create our attx groundstation
attx = GroundStationModel(exc, "attx")                                  #Create the ground station and establish the ENU frame   
attx.params.spacecraft_frame.mapTo(sc.outputs.body)                     #Assign the spacecraft that we will be tracking
attx.params.planet_rotating_frame.mapTo(earth.outputs.rotating_frame)   #Set the planet fixed frame
attx.params.R_planet(6378140.0)                                         #Set the ground station radius from the center of the Earth 
attx.params.latitude_detic_rad(DEGREES_TO_RADIANS*40.0)               #Set the ground station geocentric longitude in radians
attx.params.longitude_rad(DEGREES_TO_RADIANS*-105.0)                    #Set the ground station longitude in radians
attx.params.elevation_mask_rad(DEGREES_TO_RADIANS*10.0)                 #Set the elevation mask in radians

# Now add our spacecraft frame and our ground station to the visuals model.
# addFrame and addGroundStation may be called on as many frames and ground
# stations as the user would like, without restriction. Calling these functions
# is necessary for visuals to run in the simulation. If a spacecraft exists
# in ModelSpace but addFrame is not called, it will still run, but it will
# not render.
exc.visualsModel().addFrame(sc.outputs.body())
exc.visualsModel().addGroundStation(attx)
exc.visualsModel().params.planet_frame(earth.outputs.inertial_frame())

states = CsvLogger(exc, "sc_states.csv")
states.addParameter(exc.time().base_time, "sim_time")
states.addParameter(attx.outputs.masked, "masked")
states.addParameter(sc.outputs.pos_sc_pci, "sc_inrtl_position")
states.addParameter(sc.planetRelativeModel().outputs.latitude_detic, "latitude")
states.addParameter(sc.planetRelativeModel().outputs.longitude, "longitude")
states.addParameter(sc.planetRelativeModel().outputs.altitude_detic, "altitude_m")
states.addParameter(sc.outputs.vel_sc_pci,"sc_inrtl_velocity")        
exc.logManager().addLog(states, 1)

oe_state.params.a(7278137.0)
oe_state.params.e(0.1)
oe_state.params.i(DEGREES_TO_RADIANS*45.0)
oe_state.params.RAAN(DEGREES_TO_RADIANS*120.0)
oe_state.params.w(DEGREES_TO_RADIANS*10.0)
oe_state.params.f(DEGREES_TO_RADIANS*30.0)

exc.startup()

exc.run()