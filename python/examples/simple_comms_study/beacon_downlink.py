###############################################################################
# Copyright (c) ATTX LLC 2025. All Rights Reserved.
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
Simple Comms Study
-----------------------
This script demonstrates how to set up a simple data transfer simulation. It 
initializes a spacecraft, two ground stations, and simulates data transfer
between the spacecraft and the ground stations over a specified time period.

Author: Daniel Krobath
"""
import sys
from modelspace.ModelSpacePy import SimulationExecutive, CsvLogger, Time, connectSignals, DEGREES_TO_RADIANS
from modelspace.Spacecraft import Spacecraft
from modelspace.SpicePlanet import SpicePlanet
from modelspace.GroundStationModel import GroundStationModel
from modelspace.OrbitalElementsStateInit import OrbitalElementsStateInit
from modelspace.CommunicationsDataModel import CommunicationsDataModel
from modelspace.VisualsModel import VisualsModel

exc = SimulationExecutive()                     # Create our executive -- by convention named exc
exc.args().addDefaultArgument("end", 86400*15)  # Set end time in seconds
exc.parseArgs(sys.argv)                         # this interperets command-line inputs
exc.setRateSec(10)                              # We can setRateHz or setRateSec

# Create the Earth from SPICE kernels
earth = SpicePlanet(exc, "earth")

# Initialize orbital elements state
oe = OrbitalElementsStateInit(exc)
oe.params.a(6778140.0)                              # Semi-major axis in meters
oe.params.e(0.0006)                                   # Eccentricity
oe.params.i(DEGREES_TO_RADIANS*51.64)                # Inclination in radians
oe.params.RAAN(DEGREES_TO_RADIANS*0.0)              # Right Ascension of Ascending Node in radians
oe.params.w(DEGREES_TO_RADIANS*0.0)                 # Argument of Periapsis in radians
oe.params.f(DEGREES_TO_RADIANS*0.0)                 # Mean Anomaly in radians

# Create the spacecraft and connect its initial state to the orbital elements
sc = Spacecraft(exc, "sc")
connectSignals(oe.outputs.pos__inertial, sc.params.initial_position)
connectSignals(oe.outputs.vel__inertial, sc.params.initial_velocity)
connectSignals(earth.outputs.self_id, sc.params.planet_ptr)

rolla = GroundStationModel(exc, "Rolla")                               # Create the rolla ground station and establish the ENU frame
rolla.params.spacecraft_frame.mapTo(sc.outputs.body)                    # Assign the spacecraft that we will be tracking
rolla.params.planet_rotating_frame.mapTo(earth.outputs.rotating_frame)  # Set the planet fixed frame
rolla.params.R_planet(6378140.0)                                        # Set the ground station radius from the center of the Earth
rolla.params.latitude_detic_rad(DEGREES_TO_RADIANS*37.9485)            # Set the geometric latitude in radians
rolla.params.longitude_rad(DEGREES_TO_RADIANS*-91.7715)                 # Set the longitude in radians
rolla.params.elevation_mask_rad(DEGREES_TO_RADIANS*10)                # Set the elevation mask in radians

rolla_comm = CommunicationsDataModel(exc, "rolla_comm")               # Create the communications data model for the Kaena Point ground station
connectSignals(rolla.outputs.range, rolla_comm.inputs.range)
connectSignals(rolla.outputs.masked, rolla_comm.inputs.masked)
rolla_comm.params.frequency(2.299e9)                                      # Frequency in Hz
rolla_comm.params.bit_rate(0.0004e6)                                          # Bit rate in bits per second
rolla_comm.params.power_TX(-0.5)                                           # Transmit power in decibel-milliwatts (dBm)
rolla_comm.params.gain_TX(6)                                           # Transmit antenna gain in (dBi)
rolla_comm.params.loss_TX(2.0)                                           # Transmit system losses in dB
rolla_comm.params.gain_RX(2.0)                                           # Receive antenna gain in (dBi)
rolla_comm.params.noise_temperature(307.55)                                 # Noise temperature in Kelvin
rolla_comm.params.loss_RX(4.0)                                           # Recieve system losses in dB
rolla_comm.params.cross_pol_loss(3.0)                                   # Cross-polarization loss in dB
rolla_comm.params.energy_per_bit_to_noise_ratio_required(12.0)              # Required energy per bit to noise ratio in dB for link budget closure
# Set parameters for logging
# Be sure to name output parameters in similar fashion to what follows:

# Anything relating to a ground station will have to have the ground station name postpended
# to the output parameter name like below for the analysis script to work properly

# Additionally, both time and tdb_time must be logged with the exact names below
# for the analysis script to work properly
states = CsvLogger(exc, "states.csv")
states.addParameter(exc.time().base_time, "time")
states.addParameter(exc.time().tdb_time, "tdb_time")
states.addParameter(rolla.outputs.masked, "masked_rolla")
states.addParameter(rolla.outputs.range, "range_rolla")
states.addParameter(rolla.outputs.range_rate, "range_rate_rolla")
states.addParameter(rolla_comm.outputs.free_space_path_loss, "FSPL_rolla")
states.addParameter(rolla_comm.outputs.eff_isotropic_radiated_pwr, "EIRP_rolla")
states.addParameter(rolla_comm.outputs.carrier_to_noise_density_ratio, "C/N0_rolla")
states.addParameter(rolla_comm.outputs.energy_per_bit_to_noise_ratio, "Eb/N0_rolla")
states.addParameter(rolla_comm.outputs.link_margin, "link_margin_rolla")
exc.logManager().addLog(states, 1)

# Enable visuals
exc.enableVisuals()                                 # Enable visuals for the simulation
exc.visualsModel().addFrame(sc.outputs.body())      # Add the spacecraft body frame to the visuals
exc.visualsModel().addGroundStation(rolla)         # Add the rolla ground station to the visuals

# Run the simulation
exc.startup()
exc.run()