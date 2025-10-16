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
# <<EXCLUDE FROM DOCS>>
# TODO: Reactivate once lambert targeting is back in baseline
# """
# Apophis Intercept Mission Example
# ---------------------------------
# This complex example generates a trajectory between Earth and the asteroid
# Apophis using ModelSpace multibody effects and demonstrates it with the 
# LivePlot tool. In this scenario, the following occurs:
# - A spacecraft begins in a parking orbit around Earth and performs a burn
#   to escape Earth gravity
# - Once the spacecraft has escaped, it performs another burn using a lambert
#   solver to target the asteroid Apophis
# - The spacecraft glides toward Apophis around the sum
# - The simulation terminates when the spacecraft is within range of Apophis

# Author: Alex Reynolds 
# """
# import sys
# from modelspace.ModelSpacePy import Time, connectSignals, SimulationExecutive, END_STEP, Frame, Body, Node, \
#                          CartesianVector3, CsvLogger, DEGREES_TO_RADIANS, SpiceManager
# from modelspace.Spacecraft import Spacecraft
# from modelspace.SpicePlanet import SpicePlanet
# from modelspace.FrameStateSensorModel import FrameStateSensorModel
# from modelspace.PointMassGravityModel import PointMassGravityModel
# from modelspace.AsphericalGravityModel import AsphericalGravityModel
# from modelspace.ProximityMonitor import ProximityMonitor
# from modelspace.SimTerminationEvent import SimTerminationEvent
# from modelspace.locationsPy import defaultSpiceKernels, modelspaceDir, slash
# from library.intercept import identifyIntercept
# from modelspaceutils.vizkit.VizKitPlanetRelative import VizKitPlanetRelative

# # Define a sim constant for the apophis range at which the sim will terminate
# APOPHIS_TERMINATION_RANGE = 100000000

# # Create our simulation and configure it with step size and end time appropriate
# # for a sun orbiting spacecraft
# exc = SimulationExecutive()     # Create our executive -- by convention named exc
# exc.args().addDefaultArgument("end",     86400*365*2)
# exc.parseArgs(sys.argv)         # this interperets command-line inputs
# exc.setRateSec(Time(1000))

# # Set the start time for our simulation. This is key because our apophis asteroid
# # flyby occurs at a specific time, and if the launch window is not set properly
# # we will not be able to intercept. Here we set our date to October 2027 for the 
# # flyby. By default, times are loaded in UTC
# exc.setTime("2027 OCT 15 12:00:00.000")

# # Manually set our SPICE kernels. Note that the default kernels set by 
# # defaultSpiceKernels will be loaded unless manually specified and loaded
# # via a call to loadKernels in Spice Manager. Note here the use of the locations
# # module with modelspaceDir() and slash() guarantees that the load will work
# # on all systems (Windows/Linux) and deployments
# kernels = defaultSpiceKernels()
# kernels.append(modelspaceDir() + "cspice" + slash() + "data" + slash() + "20099942.bsp")
# exc.spiceManager().loadKernels(kernels)

# # Create the planet objects which will be used in this simulation. In this case, we 
# # need the Earth, the Sun, and the actual asteroid apophis, which will be built
# # using a SpicePlanet with provided ephemeris for Apophis
# earth = SpicePlanet(exc, "earth")
# sun = SpicePlanet(exc, "sun")
# apophis = SpicePlanet(exc, "20099942")  # This is the ID for Apophis
# apophis.params.enable_attitude(False)

# # Set the central body for the SPICE manager to the Sun. This will 
# # cause it to output all states relative to that reference point.
# # Note this is different than the solar system barycenter, "SSB"
# exc.spiceManager().setCentralBody("SUN")

# # Now create our spacecraft. Note that to add the sun and apophis as bodies which are 
# # "tracked" by the spacecraft we need to call the "addPlanet" function. This function
# # adds a sensor to track the spacecraft state relative to planet frames, adds third
# # body gravity from those planets, and also looks for keywords (such as "sun") which
# # can tie planets to other models within the spacecraft, such as solar eclipse and
# # power generation models
# sc = Spacecraft(exc, "sc")
# sc.params.planet_ptr(sun)
# fs_sc_sun = sc.planetInertialStateSensor()

# # Add frame state sensor model to sense the Earth state wrt the Sun
# fs_earth_sun = FrameStateSensorModel(exc, END_STEP, "fs_earth_sun")
# fs_earth_sun.params.target_frame_ptr(earth.outputs.inertial_frame())
# fs_earth_sun.params.reference_frame_ptr(sun.outputs.inertial_frame())

# # Add frame state sensor model to sense the Earth state wrt the Sun
# fs_apophis_sun = FrameStateSensorModel(exc, END_STEP, "fs_apophis_sun")
# fs_apophis_sun.params.target_frame_ptr(apophis.outputs.inertial_frame())
# fs_apophis_sun.params.reference_frame_ptr(sun.outputs.inertial_frame())

# # Add a proximity monitor to evaluate when the sim has approached Apophis and a 
# # termination event to end the sim once within range of Apophis. Tie them together
# # so sim terminates when within range
# prox = ProximityMonitor(exc, END_STEP, "prox")
# prox.params.trigger_range(APOPHIS_TERMINATION_RANGE)
# connectSignals(fs_apophis_sun.outputs.pos_tgt_ref__out, prox.inputs.chief_position)
# connectSignals(fs_sc_sun.outputs.pos_tgt_ref__out, prox.inputs.deputy_position)

# term = SimTerminationEvent(exc, END_STEP, "term")
# connectSignals(prox.trigger, term.trigger)

# states = CsvLogger(exc, "sunrelstates.csv")
# states.addParameter(exc.time().base_time, "sim_time")
# states.addParameter(fs_sc_sun.outputs.pos_tgt_ref__out, "sc_sun_pos")
# states.addParameter(fs_sc_sun.outputs.vel_tgt_ref__out, "sc_sun_vel")     
# states.addParameter(fs_earth_sun.outputs.pos_tgt_ref__out, "earth_sun_pos")
# states.addParameter(fs_earth_sun.outputs.vel_tgt_ref__out, "earth_sun_vel")     
# exc.logManager().addLog(states, 1)

# # Create a vizkit to visualize our spacecraft, Earth, and Apophis relative
# # to the sun. Vizkit is a very simple, lightweight way to visualize a simulation as
# # it happens. 
# vk = VizKitPlanetRelative(exc)
# vk.target(sc.outputs.body())
# vk.planet(sun.outputs.inertial_frame())
# vk.addSpacecraft(earth.outputs.inertial_frame())
# vk.addSpacecraft(apophis.outputs.inertial_frame())
# exc.logManager().addLog(vk, Time(86400*3))

# exc.startup()

# # Even though we've added planets for the sun and apophis, our "base" planet is still
# # Earth. Set an initial state relative to Earth for our LEO parking orbit
# sc.initializePositionVelocity(earth.outputs.inertial_frame().pos_f_p__p(), earth.outputs.inertial_frame().vel_f_p__p())

# targeted = False
# while not exc.isTerminated():
#     if not targeted and exc.time().base_time().asFloatingPoint() > 86400*180:
#         initial, intercept = identifyIntercept(exc, sc, apophis, 86400*300)
#         sc.programManeuver(initial, exc.time().base_time().asFloatingPoint(), exc.rootFrame())
#         sc.programManeuver(intercept, exc.time().base_time().asFloatingPoint() + 86400*300, exc.rootFrame())
#         targeted = True
    
#     exc.step()