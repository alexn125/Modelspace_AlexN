""" =====================================================
The following python script is a script to test the 
constructor and methods of the Dexstr class

Author James Tabony <jrtkdy> : 09/09/25
Revision <name> <email> : <date>
    Description line 1
    Description line 2
    ...
Revision <name> <email> : <date>
    Description line 1
    Description line 2
    ...
===================================================== """

import sys
import os
from modelspace.ModelSpacePy import (SimulationExecutive, CartesianVector3, LOG_INFO, DEGREES_TO_RADIANS, CsvLogger)
from Dexstr import Dexstr
from modelspace.Spacecraft import Spacecraft
from modelspace.SpicePlanet import SpicePlanet

# Log directory
results_path = os.path.join(os.path.dirname(__file__), "results")

# Set up the simulation executive
exc = SimulationExecutive()
exc.setRateHz(1)
exc.args().addDefaultArgument("end", 4*3600)
exc.parseArgs(sys.argv)
exc.logLevel(LOG_INFO)
exc.logManager().outDir(results_path)

# Create an instance of the Dexstr spacecraft
dexstr = Dexstr(exc)

# Configure an IMU
# dexstr.configImuFromJson("ideal", 10)
# Configure a GPS
# dexstr.configGpsFromJson("ideal", 10)
# Configure the Solar Panels
dexstr.configSolarPanelFromJson("EnduroSat")
# Configure the Battery
dexstr.configBatteryFromJson("GOMspace_nanopower_bp4_2s2p", 1.0)

# Create Logger
logger = CsvLogger(dexstr._exc, "states.csv")               # Create an instance of the logger and configure the file name
logger.addParameter(dexstr._exc.time().base_time, "time")   # Save simulation time [sec]
logger.addParameter(dexstr.outputs.pos_sc_pci, "pos")       # Save the position of the spacecraft in ECI frame [m]
logger.addParameter(dexstr.outputs.vel_sc_pci, "vel")       # Save the velocity of the spacecraft in ECI frame [m/s]
exc.logManager().addLog(logger, 1)                          # Add the logger to the simulation executive log manager and save data at 1 hz

# Start the simulaiton executive
exc.startup()

# Initialize the position and velocity of DEXSTR
ALTITUDE = 420.0*1000.0     # Altitude above the Earth in meters
dexstr.initializeState(ALTITUDE, 0.001, 45.0, 0.0, 0.0, 0.0)

# Run the simulation executive
exc.run()