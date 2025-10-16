""" =====================================================
The following python script is a derived class of the 
existing ModelSpace Task class. This class is used 
to contain all the general information of the DEXSTR 
spacecraft model and act as a single source of sim truth

The DEXSTR body frame shall be defined consistently with the 
structures subsystem. The DEXSTR body frame is defined as such:
   +x -> [2U face] Nadir face, has antenna
   -x -> [2U face] Zenith face, has GPS receiver
   +y -> [3U face]
   -y -> [3U face]
   +z -> [6U face] Sun face, sun pointing in nominal mode
   -z -> [6U face] Shadow face, anti-sun pointing in nominal mode

Author James Tabony <jrtkdy> : 09/09/25
    Created underlying architecture for DEXSTR spacecraft model
    and implemented constructor. Started creating hardware
    configuration methods based on JSON files. Everything is
    subject to change.
Revision James Tabony <jrtkdy> : 10/14/25
    Created state initialization method
Revision <name> <email> : <date>
    Description line 1
    Description line 2
    ...
===================================================== """

#########################################################
# Imports
#########################################################
# Spacecraft for tracking the DEXSTR body
from modelspace.Spacecraft import Spacecraft
# SpicePlaney for Earth data/configuration
from modelspace.SpicePlanet import SpicePlanet
# CartesianVector, Quaternion, Matrix3, and DCM for data holding
from modelspace.ModelSpacePy import (CartesianVector3, Quaternion, Matrix3, DCM, DEGREES_TO_RADIANS)
# IMU, Magnetometer, GPS and Sun sensor for sensor suite
from modelspace.IMU import IMU
from modelspace.Magnetometer import Magnetometer
from modelspace.GPS import GPS
from modelspace.SunSensor import SunSensor
# Reaction wheel and magnetorquer for actuator suite
from modelspace.ReactionWheelModel import ReactionWheelModel
#! NO MAGNETORQUER MODEL IN MODELSPACE
# Solar panel and battery for power hardware suite
from modelspace.SolarPanelModel import SolarPanelModel
from modelspace.SimpleBatterySystem import SimpleBatterySystem
# Radio for communications suite
#! NO RADIO MODEL IN MODELSPACE

# os for file pathing
import os
# Math for trig and sqrt functions
import math
# Numpy for matrix and vector operations
import numpy as np



class Dexstr(Spacecraft):
#########################################################
# Dexstr Class Constructor
#########################################################
    def __init__(self, exc):
        '''
        HOW TO USE THIS CLASS:
            # Import the Dexstr class
            from Dexstr import Dexstr

            # Create an instance of the Dexstr class
            dexstr = Dexstr(exc)

            # Configure hardware using the configuration methods
            # Only sets params, connect signals must be done externally
            dexstr.config<hardware_name>FromJson(<Json_file_name>, <other_parameters>)

            # Now all configured hardware is accesable through the dexstr instance
            # For example, to access the IMU instance:
            imu = dexstr._imu
            # In general, hardware is stored in the instance variable
            <hardware_name> = dexstr._<hardware_name>
        '''
        # Initialize our self Spacecraft
        Spacecraft.__init__(self, exc, "DEXSTR")
        # Define members of this class at initial configuration
        self._exc = exc

        # Config DEXSTR to be around Earth
        self.earth = SpicePlanet(exc, "Earth")
        self.params.planet_ptr(self.earth)

        # Create a Sun model for sensor use
        self.sun = SpicePlanet(exc, "Sun")

        # TODO: Pending CAD Model
        # Define mass of DEXSTR [kg]
        self.params.mass(12.0)
        # Define mass moment of inertia of DEXSTR [kg-m^2]
        self.params.inertia(Matrix3([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]))



#########################################################
# Dexstr Set Initial State Method
#########################################################
    def initializeState(self, altitude, eccentricity, inclination, RAAN, argument_periapsis, mean_anomaly):
        '''
        The method is used to initialize the position, velocity, attitude, and 
        angular velocity of DEXSTR.

        This method does not account for spice data so the generated initial 
        cartesian position and velocities are not necessarily exact.

        INPUTS
        altitude (float):
            The altitude of the semi-major axis of the orbit in meters
        eccentricity (float):
            The eccentricity of the orbit
        inclination (float):
            The inclination of the orbit in degrees
        RAAN (float):
            The right ascension of the ascending node of the orbit in degrees
        argument_periapsis (float):
            The argument of periapsis of the orbit in degrees
        mean_anomaly (float):
            The mean anomaly of the spacecraft within the orbit in degrees
        '''
        # Convert degrees to radians
        inclination         = inclination           * DEGREES_TO_RADIANS
        RAAN                = RAAN                  * DEGREES_TO_RADIANS
        argument_periapsis  = argument_periapsis    * DEGREES_TO_RADIANS
        mean_anomaly        = mean_anomaly          * DEGREES_TO_RADIANS

        # Find Eccentric Anomaly Using Newtons Method
        tolerance = 1e-15
        error = 1
        eccentric_anomaly = mean_anomaly
        while (error > tolerance):
            eccentric_anomaly = eccentric_anomaly - (eccentric_anomaly-eccentricity*math.sin(eccentric_anomaly)-mean_anomaly)/(1-eccentricity*math.cos(eccentric_anomaly))
            error = eccentric_anomaly-eccentricity*math.sin(eccentric_anomaly)-mean_anomaly
        
        # Find True Anomaly from Eccentric Anamoly
        true_anomaly = 2*math.atan(math.sqrt((1+eccentricity)/(1-eccentricity))*math.tan(eccentric_anomaly/2))

        # Finding position and velocity vectors in LVLH frame
        a = altitude+self.earth.outputs.eq_radius()         # Semi-major axis
        r = a*(1-eccentricity*math.cos(eccentric_anomaly))  # Radius
        h = math.sqrt(self.earth.outputs.mu()*a*(1-eccentricity*eccentricity))  # Specific angular momentum
        vr = self.earth.outputs.mu()*eccentricity*math.sin(true_anomaly)/h      # Radial component of velocity
        vt = h/r    # Theta-hat component of velocity
        position = np.array([r, 0.0, 0.0])
        velocity = np.array([vr, vt, 0.0])

        # Finding the 3-2-3 DCM for LVLH to ECI coordinate conversion
        theta = argument_periapsis + true_anomaly
        ROT1 = np.array([[math.cos(RAAN), -math.sin(RAAN), 0.0],
                        [math.sin(RAAN),  math.cos(RAAN), 0.0],
                        [     0.0,             0.0,       1.0]])    # Z-rotation by RAAN
        ROT2 = np.array([[1.0,        0.0,                    0.0           ],
                        [0.0, math.cos(inclination), -math.sin(inclination)],
                        [0.0, math.sin(inclination),  math.cos(inclination)]])  # X-rotation by inclination
        ROT3 = np.array([[math.cos(theta), -math.sin(theta), 0.0],
                        [math.sin(theta),  math.cos(theta), 0.0],
                        [      0.0,              0.0,       1.0]])  # Z-rotation by theta
        DCM = ROT1 @ ROT2 @ ROT3

        # Rotate the position and velocity vectors and set the spacecraft initial position and velocity
        position = CartesianVector3((DCM @ position).tolist())
        velocity = CartesianVector3((DCM @ velocity).tolist())
        self.initializePositionVelocity(position, velocity)



#########################################################
# Dexstr IMU Configuration Method
#########################################################
    def configImuFromJson(self, imu_name, rate, latency=0, accel_rng_seed=0, gyro_rng_seed=0):
        '''
        JSON files for the IMU are saved in the directory 'python/DEXSTR/imu'
        Each JSON file corresponds to an IMU that MSAT is considering using.

        This method is where the alignment and position of the IMU
        within DEXSTR is defined. Truth as defined from CAD model.

        INPUTS
        imu_name (string):
            Name of the imu you wish to config DEXSTR with, the name should 
            match the name of the JSON file and manufacturer provided name.
        rate (integer):
            Measurement rate of the IMU in Hz.
        latency (integer) [optional]:
            Latency of the IMU in milliseconds, default is 0.
        accel_rng_seed (integrer) [optional]:
            Seed for the random number generator of the accelerometer,
            default is 0.
        gyro_rng_seed (integer) [optional]:
            Seed for the random number generator of the gyroscope,
            default is 0.
        '''
        # Create the IMU and configure it from the JSON file
        self.imu = IMU(self._exc, "DEXSTR_IMU")
        json_path = os.path.join(os.path.dirname(__file__), 'imu', imu_name + '.json')
        self.imu.configureFromJson(json_path)

        # Set the rate of the IMU [Hz]
        self.imu.params.rate_hz(rate)

        # Set the rng seed of the IMU
        self.imu.params.accelerometer_seed_value(accel_rng_seed)
        self.imu.params.gyro_seed_value(gyro_rng_seed)

        # Set the latency of the IMU [ms]
        self.imu.params.latency(latency)

        # TODO: Pending CAD Model
        ########## Mount the IMU to the body frame of DEXSTR ##########
        self.imu.params.mount_frame(self.outputs.body())
        # Position of the IMU frame wrt body frame resolved in body frame [m]
        self.imu.params.mount_position__mf(CartesianVector3([0.0, 0.0, 0.0]))
        # Orientation of the IMU frame wrt body frame, provided as a quaternion
        self.imu.params.mount_alignment_mf(Quaternion([1.0, 0.0, 0.0, 0.0]))

        ########## Configure gravity experienced by the IMU [m/s^2] ##########
        #* This value should not matter, ModelSpace has an informational output for
        #* the IMU that includes gravity. This is not what the IMU actually measures.
        #* Which is also why we set the gravity vector to zero in the IMU params.
        self.imu.params.gravity_frame(self.earth.outputs.inertial_frame())
        self.imu.inputs.gravity__gf(CartesianVector3([0.0, 0.0, 0.0]))



#########################################################
# Dexstr Magnetometer Configuration Method
#########################################################
    def configMagnetometerFromJson(self, mag_name, rate, latency=0, mag_rng_seed=0):
        '''
        JSON files for the magnetometer are saved in the directory 
        'python/DEXSTR/magnetometer'. Each JSON file corresponds 
        to a magnetometer that MSAT is considering using.

        This method is where the alignment and position of the magnetometer
        within DEXSTR is defined. Truth as defined from CAD model.

        INPUTS
        mag_name (string):
            Name of the magnetometer you wish to config DEXSTR with, the name 
            should match the name of the JSON file and manufacturer provided name.
        rate (integer):
            Measurement rate of the magnetometer in Hz.
        latency (integer) [optional]:
            Latency of the magnetometer in milliseconds, default is 0.
        mag_rng_seed (integer) [optional]:
            Seed for the random number generator of the magnetometer,
            default is 0.
        '''
        # Create the magnetometer and configure it from the JSON file
        self.magnetometer = Magnetometer(self._exc, "DEXSTR_Magnetometer")
        json_path = os.path.join(os.path.dirname(__file__), "magnetometer", mag_name + ".json")
        self.magnetometer.configureFromJson(json_path)

        # Set the rate of the magnetometer [Hz]
        self.magnetometer.params.rate_hz(rate)

        # Set the rng seed of the magnetometer
        self.magnetometer.params.seed_value(mag_rng_seed)

        # Set the latency of the magnetometer [ms]
        self.magnetometer.params.latency(latency)

        # TODO: Pending CAD Model
        ########## Mount the magnetometer to the body frame of DEXSTR ##########
        self.magnetometer.params.mount_frame(self.outputs.body())
        # Position of the magnetometer frame wrt body frame resolved in body frame [m]
        self.magnetometer.params.mount_position__mf(CartesianVector3([0.0, 0.0, 0.0]))
        # Orientation of the magnetometer frame wrt body frame, provided as a quaternion
        self.magnetometer.params.mount_alignment_mf(Quaternion([1.0, 0.0, 0.0, 0.0]))

        ########## Configure magnetic field experienced by the magnetometer ##########
        #! WMM Model in Next ModelSpace Release
        # TODO import WMM model
        # TODO create an instance of the WMM model
        # TODO configure and connect signals of the WMM model
        # TODO Set the WMM model outputs to the magenetometer inputs



#########################################################
# Dexstr GPS Configuration Method
#########################################################
    def configGpsFromJson(self, gps_name, rate, latency=0, gps_rng_seed=0):
        '''
        JSON files for the GPS are saved in the directory 
        'python/DEXSTR/gps'. Each JSON file corresponds 
        to a gps that MSAT is considering using.

        This method is where the alignment and position of the gps
        within DEXSTR is defined. Truth as defined from CAD model.

        INPUTS
        gps_name (string):
            Name of the gps you wish to config DEXSTR with, the name 
            should match the name of the JSON file and manufacturer provided name.
        rate (integer):
            Measurement rate of the gps in Hz.
        latency (integer) [optional]:
            Latency of the GPS in milliseconds, default is 0.
        gps_rng_seed (integer) [optional]:
            Seed for the random number generator of the gps,
            default is 0.
        '''
        # Create the gps and configure it from the JSON file
        self.gps = GPS(self._exc, "DEXSTR_GPS")
        json_path = os.path.join(os.path.dirname(__file__), 'gps', gps_name + '.json')
        self.gps.configureFromJson(json_path)

        # Set the rate of the GPS [Hz]
        self.gps.params.rate_hz(rate)

        # Set the rng seed of the GPS
        self.gps.params.seed_value(gps_rng_seed)

        # Set the latency of the GPS [ms]
        self.gps.params.latency(latency)

        ########## Set initial noise characteristics ##########
        self.gps.params.pos_bias_initial(CartesianVector3([0.0, 0.0, 0.0]))
        self.gps.params.vel_bias_initial(CartesianVector3([0.0, 0.0, 0.0]))

        # TODO: Pending CAD Model
        ########## Mount the GPS to the body frame of DEXSTR ##########
        self.gps.params.mount_frame(self.outputs.body())
        # Position of the GPS frame wrt body frame resolved in body frame [m]
        self.gps.params.mount_position__mf(CartesianVector3([0.0, 0.0, 0.0]))

        ########## Configure GPS dead zones ##########
        # Set the maximum altitude very high (there is no maximum altitude) [m]
        self.gps.params.max_altitude(1000000000.0)
        # Set the maximum velocity very high (there is no maximum velocity) [m/s]
        self.gps.params.max_speed(1000000000.0)

        ########## Configure the Earth rotating frame into the GPS model ##########
        self.gps.params.earth_rotating_frame(self.earth.outputs.rotating_frame())



#########################################################
# Dexstr Sun Sensor Configuration Method
#########################################################



#########################################################
# Dexstr Reaction Wheel Configuration Method
#########################################################



#########################################################
# Dexstr Magnetorquer Configuration Method
#########################################################



#########################################################
# Dexstr Solar Panel Configuration Method
#########################################################
    def configSolarPanelFromJson(self, solar_panel_name, area_per_u=0.01):
        '''
        JSON files for the solar panels are saved in the directory
        'python/DEXSTR/solarpanel'. Each JSON file corresponds
        to a solar panel that MSAT is considering using.

        The solar panels are acessable through
            self._panels[i] for i = [0, 1, 2, 3]
            # i = 0 -> +y face
            # i = 1 -> -y face
            # i = 2 -> +z face
            # i = 3 -> -z face

        INPUTS
        solar_panel_name (string):
            Name of the solar panel you wish to config DEXSTR with, the name
            should match the name of the JSON file and manufacturer provided name.
            This name should be concatenated with _nU where n is the expected U
            of a given face (e.g., 2, 3, 6, etc.)
        area_per_u (float) [optional]:
            The aera of the solar panels in m^2 per U of surface area.
            This value cannot be greater than 0.01 m^2 and defaults to that value.
        '''
        # Define the order of body faces that solar panels are configured
        SOLAR_PANEL_FACE = ['posY',
                            'negY',
                            'posZ',
                            'negZ']
        # Define the orientations of the solar panels in body frame coordinates
        SOLAR_PANEL_NORMALS = [CartesianVector3([0.0,  1.0,  0.0]),  # +y -> 3U
                               CartesianVector3([0.0, -1.0,  0.0]),  # -y -> 3U
                               CartesianVector3([0.0,  0.0,  1.0]),  # +z -> 6U
                               CartesianVector3([0.0,  0.0, -1.0])]  # -z -> 6U
        # Define the size of each of the faces [U]
        FACE_SIZE_U = [3,
                       3,
                       6,
                       6]

        # Create the solar panels and configure it from the JSON file
        self.panels = []
        for i in range(len(SOLAR_PANEL_FACE)):
            # Create the next solar panel
            self.panels.append(SolarPanelModel(self._exc, "DEXSTR_solar_panel_" + SOLAR_PANEL_FACE[i]))
            # Define the path to the JSON file
            json_path = os.path.join(os.path.dirname(__file__), 'solarpanel', solar_panel_name + '_' + str(FACE_SIZE_U[i]) + 'U' +'.json')
            # Configure it from JSON
            self.panels[-1].configureFromJson(json_path)
            # Define the area
            self.panels[-1].params.panel_area(area_per_u * FACE_SIZE_U[i])
            # Define the orientation in body frame
            self.panels[-1].params.body_frame_ptr(self.outputs.body())
            self.panels[-1].params.panel_normal__body(SOLAR_PANEL_NORMALS[i])
            # Configure the sun and earth frames into the solar panel model
            self.panels[-1].params.sun_frame_ptr(self.sun.outputs.inertial_frame())
            self.panels[-1].params.r_sun(self.sun.outputs.eq_radius())
            self.panels[-1].params.planet_frame_ptr(self.earth.outputs.inertial_frame())
            self.panels[-1].params.r_planet(self.earth.outputs.eq_radius())



#########################################################
# Dexstr Battery Configuration Method
#########################################################
    def configBatteryFromJson(self, battery_name, initial_charge=1.0):
        '''
        JSON files for the battery are saved in the directory 
        'python/DEXSTR/battery'. Each JSON file corresponds 
        to a battery that MSAT is considering using.

        INPUTS
        battery_name (string):
            Name of the battery you wish to config DEXSTR with, the name 
            should match the name of the JSON file and manufacturer provided name.
        initial_charge (float) [optional]:
            The initial percentage charge of the battery, value between 0 and 1,
            with 1 being fully charged and 0 being initially empty. Default is 1.
        '''
        # Create the battery and configure it from the JSON file
        self.battery = SimpleBatterySystem(self._exc, "DEXSTR_battery")
        json_path = os.path.join(os.path.dirname(__file__), 'battery', battery_name + '.json')
        self.battery.configureFromJson(json_path)

        # Set the initial charge of the battery [%]
        self.battery.params.initial_charge_state(initial_charge)



#########################################################
# Dexstr Radio Configuration Method
#########################################################