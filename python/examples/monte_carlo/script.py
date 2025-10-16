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
Monte Carlo Tutorial
--------------------
This script illustrates a simple example with a single spacecraft orbiting
the Earth with dispersed orbit values

Author: Alex Reynolds
"""
import sys
from modelspace.ModelSpacePy import SimulationExecutive, ALL, START_STEP, END_STEP, CsvLogger, connectSignals
from modelspace.CustomPlanet import CustomPlanet
from modelspace.Spacecraft import Spacecraft
from modelspace.OrbitalElementsStateInit import OrbitalElementsStateInit
from modelspace.FrameStateSensorModel import FrameStateSensorModel
from modelspace.PointMassGravityModel import PointMassGravityModel
from modelspace.LvlhFrameManagerModel import LvlhFrameManagerModel

if __name__ == '__main__':
    # First, initialize our executive. Set default arguments for time, etc. that can
    # be overwritten from command line
    exc = SimulationExecutive()
    exc.args().addDefaultArgument("end",     15000)
    exc.parseArgs(sys.argv)         # this interperets command-line inputs

    # Create a planet and spacecraft
    earth = CustomPlanet(exc, "earth")

    oe_state = OrbitalElementsStateInit(exc)

    sc = Spacecraft(exc, "sc")
    connectSignals(oe_state.outputs.pos__inertial, sc.params.initial_position)
    connectSignals(oe_state.outputs.vel__inertial, sc.params.initial_velocity)
    sc.params.planet_ptr(earth)

    # Disperse our spacecraft orbit. In this example, we create three dispersions:
    # semimajor_axis, eccentricity, and inclination, to disperse our orbit. Valid choices
    # are a uniform or gaussian input distribution. Values are set on each to parameterize
    # the dispersed values, as well as set a default value. The default value is the value
    # which the sim uses for run 0, which is undispersed. All other runs, which may be
    # set using the --run=<number> command line argument, will use dispersed values drawn from
    # the dispersion.
    semimajor_axis = exc.dispersions().createUniformInputDispersion("semimajor_axis", 10678140.0, 10878040.0, 10778240.0)   # (name, min, max, default) for uniform
    eccentricity = exc.dispersions().createUniformInputDispersion("eccentricity", 0.001, 1e-6, 0.1)     # (name, mean, std, default) for gaussian
    inclination = exc.dispersions().createUniformInputDispersion("inclination", 0.3, 0.5, 1.0)

    # Set up our logging. Here we'll create a simple CSV logger to record our
    # time and spacecraft state in the root frame
    state = CsvLogger(exc, "states.csv") 
    state.addParameter(exc.time().base_time, "time")
    state.addParameter(sc.outputs.pos_sc_pci, "sc_pos")  # Add our spacecraft position via direct ref
    exc.logManager().addLog(state, 1)   

    # Here we actually apply our dispersions to initialize the spacecraft orbit from their values. Note
    # how each dispersion is called twice with () -- the first call returns the actual dispersion value,
    # which contains information about the dispersion as well as its current value. The second call returns
    # the current value of the dispersion as set by the run number.
    oe_state.params.a(semimajor_axis()())
    oe_state.params.e(eccentricity()())
    oe_state.params.i(inclination()())
    oe_state.params.RAAN(0.0)
    oe_state.params.w(0.0)
    oe_state.params.f(0.0)

    # Call startup on our executive. This will initialize our exec and,
    # recursively, all of our models.
    exc.startup()

    # Now run our simulation by calling our sim.run. This will automatically
    # terminate when our sim reaches its end time
    exc.run()