"""
Alex Newett - Modelspace GNC Research 2025

"""
## Important stuff
import sys, math, os
import numpy as np
import pandas as pad
## Basic imports
from modelspace.Spacecraft import Spacecraft
from modelspace.SpicePlanet import SpicePlanet
from modelspace.OrbitalElementsStateInit import OrbitalElementsStateInit
## Sensor imports
from modelspace.StarTracker import StarTracker
from modelspace.Gyro import Gyro
# from modelspace.Magnetometer import Magnetometer
## Navigation imports
from modelspace.AttitudeEkfTimeUpdate import AttitudeEkfTimeUpdate
from modelspace.AttitudeEkfMeasUpdate import AttitudeEkfMeasUpdate
from modelspace.SimpleDiscreteProcessNoise import SimpleDiscreteProcessNoise
## Guidance imports
from modelspace.TriadGuidance import TriadGuidance
## Control imports
from modelspace.PidAttitudeControl import PidAttitudeControl
## numpy matrix math custom functions
from transforms import skew_sym, shadowset, MRPsubtract, quat2MRP

## MODELSPACE.PY IMPORTS --------------------------------------------------------------------------------------------------------------------------------------------------
# Base simulation stuff
from modelspace.ModelSpacePy import SimulationExecutive, connectSignals, Time, CsvLogger, Node, START_STEP, END_STEP
# Unit conversions
from modelspace.ModelSpacePy import DEGREES_TO_RADIANS
# Vectors, matrices, attitude
from modelspace.ModelSpacePy import CartesianVector3, Matrix6, Matrix63, Matrix3, MRP, Quaternion
# Visuals
from modelspaceutils.vizkit.VizKitPlanetRelative import VizKitPlanetRelative
# Frame state sensor model
from modelspace.FrameStateSensorModel import FrameStateSensorModel
## Clear terminal
os.system('cls' if os.name == 'nt' else 'clear')

# REACTION_WHEELS = [(Euler321([0.0, 0.0, 0.0]).toDCM()).toQuaternion(),
#                     (Euler321([0.0, 90.0*DEGREES_TO_RADIANS, 0.0]).toDCM()).toQuaternion(),
#                     (Euler321([0.0, 0.0, 90.0*DEGREES_TO_RADIANS]).toDCM()).toQuaternion()]

"Overall Simulation Setup ------------------------------------------------------------------------------------------------------"

sim_rate = 1 # Hz
sim_length = 3600 # seconds

commands = np.genfromtxt('commands.csv',delimiter=',')
## Simulation Executive
exc = SimulationExecutive()
exc.parseArgs(sys.argv)
exc.setRateHz(sim_rate)
exc.end(sim_length)
# exc.end(100.0)

## Create Planet and Sun
earth = SpicePlanet(exc, "earth")
sun = SpicePlanet(exc, "sun")

"-------------------------------------------------------------------------------------------------------------------------------"

"Read TLE/Initialize Spacecraft ------------------------------------------------------------------------------------------------"
## Create object to initialize position and velocity
orbels_init = OrbitalElementsStateInit(exc)

## Read TLE and assign Keplerian elements
with open('INIT_TLE.txt','r') as f:
    ## Orbital paramters data
    read_data = f.readlines()
    line0 = read_data[0]
    line1 = read_data[1]
    line2 = read_data[2]

    yr_past_2k = int(line1[18:20])
    days = float(line1[20:32])
    inclination = float(line2[8:16])
    raan = float(line2[17:25])
    ecc = float("0." + line2[26:33])
    argofp = float(line2[34:42])
    meananom = float(line2[43:51])
    meananom = DEGREES_TO_RADIANS*meananom

    meanmot = float(line2[52:63])
    meanmot = meanmot*((2*math.pi)/86400)

    ## Time data
    seconds = (days - math.floor(days))*86400
    days    = math.floor(days)
    hours   = math.floor(seconds/3600)
    seconds = seconds - hours*3600
    minutes = math.floor(seconds/60)
    seconds = seconds - minutes*60
    yr = 2000 + yr_past_2k

    starttimestring = str(yr) + "-" + str(days) + "::" + str(hours) + ":" + str(minutes) + ":" + str(seconds)

    exc.setTime(starttimestring)

    f.close() 

## Calculate remaining elements
semimajoraxis = (earth.outputs.mu()/(meanmot*meanmot))**(1/3)
period = 2*math.pi*math.sqrt((semimajoraxis**3)/earth.outputs.mu())

## Newton-Raphson to calculate eccentric anomaly
max_iterations = 100
tolerance = 1e-14
its = 0
Eg = meananom
while its<max_iterations:
    E = meananom + (ecc*math.sin(Eg)) - Eg
    if abs(E) < tolerance:
        EccAnom = Eg 
        trueAnom = 2*math.atan(math.sqrt((1+ecc)/(1-ecc))*math.tan(EccAnom/2)) # True anomaly from eccentric anomaly
        break
    else:
        bottom = (ecc*math.cos(Eg)) - 1
        Eg2 = Eg - (E/bottom)
        Eg = Eg2
        its += 1
        if its == max_iterations:
            raise RuntimeError

## Spacecraft Object
sc = Spacecraft(exc,"sc")

Ixx = 0.026
Iyy = 0.06
Izz = 0.085

sc.params.mass(8.0)
sc.params.inertia(Matrix3([[Ixx, 0.0, 0.0],
                   [0.0, Iyy, 0.0],
                   [0.0, 0.0, Izz]]))

## Initial Truth Attitude and Angular Velocity
init_attitude_truth = MRP([0.0, 0.1, 0.0])
init_angvel_truth = CartesianVector3([-0.2*DEGREES_TO_RADIANS, 0.2*DEGREES_TO_RADIANS, 0.2*DEGREES_TO_RADIANS])

## Assign initial state parameters
connectSignals(orbels_init.outputs.pos__inertial, sc.params.initial_position)
connectSignals(orbels_init.outputs.vel__inertial, sc.params.initial_velocity)
sc.params.initial_attitude(init_attitude_truth.toQuaternion())

sc.params.initial_ang_vel(init_angvel_truth)
sc.params.planet_ptr(earth)

scnode = Node("sc_node",sc.body())

## Initial truth position and velocity from Keplerian elements
orbels_init.params.a(semimajoraxis)
orbels_init.params.e(ecc)
orbels_init.params.i(DEGREES_TO_RADIANS*inclination)
orbels_init.params.RAAN(DEGREES_TO_RADIANS*raan)
orbels_init.params.w(DEGREES_TO_RADIANS*argofp)
orbels_init.params.f(trueAnom)

"-------------------------------------------------------------------------------------------------------------------------------"

"Sensor Setup ------------------------------------------------------------------------------------------------------------------"
## Star Tracker
st = StarTracker(exc, START_STEP, "st")
st.configureFromDefault("Arcsec_Sagitta")
st.params.mount_frame(sc.body())
st.params.reference_frame(earth.outputs.inertial_frame())

## Gyro
imu = Gyro(exc,START_STEP, "imu")
imu.params.mount_frame(sc.body())
imu.params.bias(CartesianVector3([-1*(1/3600)*DEGREES_TO_RADIANS, 2*(1/3600)*DEGREES_TO_RADIANS, -3*(1/3600)*DEGREES_TO_RADIANS]))
imu.params.mount_position__mf(CartesianVector3([0.0,0.0,0.0]))
imu.params.mount_alignment_mf(Quaternion([1.0,0.0,0.0,0.0]))
imu.params.rate_hz(int(sim_rate))

## Sun Sensor
sun_sens = FrameStateSensorModel(exc, START_STEP, "sun_sens")
sun_sens.params.target_frame_ptr(sc.body())
sun_sens.params.reference_frame_ptr(sun.outputs.inertial_frame())
sun_sens.params.output_frame_ptr(earth.outputs.inertial_frame())                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

## GPS/Earth Sensor
erf_sens = FrameStateSensorModel(exc, START_STEP, "erf_sens")
erf_sens.params.target_frame_ptr(sc.body())
erf_sens.params.reference_frame_ptr(earth.outputs.inertial_frame())
erf_sens.params.output_frame_ptr(earth.outputs.inertial_frame())

"-------------------------------------------------------------------------------------------------------------------------------"

"Navigation --------------------------------------------------------------------------------------------------------------------"
## Attitude EKF Time Update (Propagation)
ekf_prop = AttitudeEkfTimeUpdate(exc, START_STEP, "ekf_prop")
connectSignals(ekf_prop.outputs.time_state, ekf_prop.inputs.time_prev)
connectSignals(exc.time().base_time, ekf_prop.inputs.time_update)

## Process Noise 
process_noise = SimpleDiscreteProcessNoise(exc, START_STEP, "process_noise")
process_noise.params.Qc(Matrix3([[3e-9, 0, 0], [0, 3e-9, 0], [0, 0, 3e-9]]))
B = Matrix63([[1,0,0],[0,1,0],[0,0,1],[0,0,0],[0,0,0],[0,0,0]])
process_noise.params.B(B)
process_noise.inputs.time_prev(Time(0))
process_noise.inputs.time_update(Time(1))
connectSignals(ekf_prop.outputs.cov_update, process_noise.inputs.cov_pre_snc)

## Attitude EKF Measurement Update
ekf_meas = AttitudeEkfMeasUpdate(exc, START_STEP, "ekf_meas")
ekf_meas.params.att_residual_filter(4)
ekf_meas.params.meas_covariance(Matrix3([[3e-5, 0, 0], [0, 3e-5, 0], [0, 0, 3e-5]]))

## Assign EKF parameters
connectSignals(exc.time().base_time, ekf_meas.inputs.time_update)
connectSignals(st.outputs.meas_quat_sf_ref, ekf_meas.inputs.meas_quat_body_inertial)
connectSignals(ekf_prop.outputs.mrp_update_body_inertial, ekf_meas.inputs.att_minus_mrp_body_inertial)
connectSignals(ekf_prop.outputs.gyro_bias, ekf_meas.inputs.bias_minus)
connectSignals(process_noise.outputs.cov_post_snc, ekf_meas.inputs.cov_minus)

connectSignals(ekf_meas.outputs.att_plus_mrp_body_inertial, ekf_prop.inputs.mrp_prev_body_inertial)
connectSignals(ekf_meas.outputs.bias_plus, ekf_prop.inputs.gyro_bias)
connectSignals(ekf_meas.outputs.cov_plus, ekf_prop.inputs.cov_prev)
connectSignals(imu.outputs.meas_ang_vel_sf,ekf_prop.inputs.ang_vel_meas_body_inertial)

"-------------------------------------------------------------------------------------------------------------------------------"

"Guidance ----------------------------------------------------------------------------------------------------------------------"
## Triad Guidance setup
triad = TriadGuidance(exc, START_STEP, "triad")
triad.inputs.current_primary_body(CartesianVector3([1.0, 0.0, 0.0]))
triad.inputs.current_secondary_body(CartesianVector3([0.0, 1.0, 0.0]))
connectSignals(erf_sens.outputs.pos_tgt_ref__out, triad.inputs.desired_primary)

"-------------------------------------------------------------------------------------------------------------------------------"

"Control -----------------------------------------------------------------------------------------------------------------------"
## Control setup
pd = PidAttitudeControl(exc, START_STEP, "PD")
pd.params.P(-0.0007)
pd.params.K(-0.004)

## Connecting signals
connectSignals(triad.outputs.quat_body_ref, pd.inputs.cmd_state)
connectSignals(st.outputs.meas_quat_sf_ref,pd.inputs.act_state)
connectSignals(imu.outputs.meas_ang_vel_sf,pd.inputs.act_ang_vel)

connectSignals(pd.outputs.control_cmd,scnode.moment)

"-------------------------------------------------------------------------------------------------------------------------------"

"Logging -----------------------------------------------------------------------------------------------------------------------"
## Save truth data
truth = CsvLogger(exc, "truth.csv")
truth.addParameter(exc.time().base_time,"sim_time")
truth.addParameter(sc.outputs.quat_sc_pci,"quat_true")
truth.addParameter(sc.outputs.ang_vel_sc_pci__body,"angvel_true")
truth.addParameter(imu.params.bias,"gyro_bias_true")
exc.logManager().addLog(truth,Time(1))

## Save nav outputs
navout = CsvLogger(exc, "nav_log.csv")
navout.addParameter(exc.time().base_time, "time")                     
navout.addParameter(ekf_prop.outputs.mrp_update_body_inertial,"mrp_minus")     
navout.addParameter(ekf_prop.outputs.gyro_bias,"bias_minus")     
navout.addParameter(ekf_prop.outputs.cov_update,"cov_minus")   
navout.addParameter(ekf_meas.outputs.att_plus_mrp_body_inertial,"mrp_plus")     
navout.addParameter(ekf_meas.outputs.bias_plus,"bias_plus")     
navout.addParameter(ekf_meas.outputs.cov_plus,"cov_plus")   
navout.addParameter(ekf_meas.outputs.meas_processed,"meas_processed")   
navout.addParameter(ekf_meas.outputs.meas_pre_residual,"pre_update_residual")   
exc.logManager().addLog(navout, Time(1))

guidout = CsvLogger(exc, "guid_log.csv")
guidout.addParameter(exc.time().base_time, "time")
guidout.addParameter(triad.outputs.quat_body_ref,"quat_body_ref")     
guidout.addParameter(triad.inputs.desired_secondary, "sun_pnt")
exc.logManager().addLog(guidout, Time(1))

contout = CsvLogger(exc, "control_log.csv")
contout.addParameter(exc.time().base_time, "time")
contout.addParameter(pd.outputs.control_cmd, "torque commands")
contout.addParameter(pd.outputs.error_quat, "error_quat")
exc.logManager().addLog(contout, Time(1))

sensors = CsvLogger(exc, "sensors.csv")
sensors.addParameter(exc.time().base_time, "time")
sensors.addParameter(erf_sens.outputs.pos_tgt_ref__out, "earth_sen")
sensors.addParameter(sun_sens.outputs.pos_tgt_ref__out, "sun_sen")
sensors.addParameter(imu.outputs.meas_ang_vel_sf, "gyro_sen")
sensors.addParameter(st.outputs.meas_quat_sf_ref, "st_sen")
exc.logManager().addLog(sensors, Time(1))

"-------------------------------------------------------------------------------------------------------------------------------"

"Visuals -----------------------------------------------------------------------------------------------------------------------"
# # Visuals (if on)

# vk_planet_rel = VizKitPlanetRelative(exc)
# connectSignals(earth.outputs.inertial_frame, vk_planet_rel.planet)
# connectSignals(sc.outputs.body, vk_planet_rel.target)

# vk_planet_rel_rate = Time()
# vk_planet_rel_rate.fromDouble(1.0)
# exc.logManager().addLog(vk_planet_rel, vk_planet_rel_rate)

# vk = VizKitPlanetRelative(exc)
# vk.target(sc.outputs.body())
# vk.planet(earth.outputs.inertial_frame())
# exc.logManager().addLog(vk, Time(100))

# "-------------------------------------------------------------------------------------------------------------------------------"

"Simulation Loop ---------------------------------------------------------------------------------------------------------------"
## Startup (initializes exc and models)
exc.startup()

## Navigation initialization
att_est_init = MRP()
att_est_init.set(0,0.1)
att_est_init.set(1,0.3)
att_est_init.set(2,-0.2)

bias_est_init = CartesianVector3([0.0,0.0,0.0])

ekf_prop.inputs.mrp_prev_body_inertial(att_est_init)
ekf_prop.inputs.gyro_bias(bias_est_init)

COV_initial = Matrix6()
COV_initial.set(0, 0, 0.1)
COV_initial.set(1, 1, 0.1)
COV_initial.set(2, 2, 0.1)
COV_initial.set(3, 3, 0.1)
COV_initial.set(4, 4, 0.1)
COV_initial.set(5, 5, 0.1)
ekf_prop.inputs.cov_prev(COV_initial)

## Run simulation

triad.inputs.desired_secondary(CartesianVector3([0.0, 1.0, 0.0]))

first_step = True
second_step = False
GNCstart = False

initpos_w = sc.params.initial_position()
initpos = np.array([initpos_w.get(0),initpos_w.get(1),initpos_w.get(2)])

while not exc.isTerminated():

    if first_step:
        triad.inputs.desired_secondary(CartesianVector3([0.0,1.0,0.0])) #before gnc starts, set triad to placeholder vector
    elif second_step:
        sunout = sun_sens.outputs.pos_tgt_ref__out()
        triad.inputs.desired_secondary(CartesianVector3([-1*sunout.get(0),-1*sunout.get(1),-1*sunout.get(2)]))
    elif GNCstart: # runs GNC loop for every step now that gps measurements have populated
        sunout = sun_sens.outputs.pos_tgt_ref__out()
        triad.inputs.desired_secondary(CartesianVector3([-1*sunout.get(0),-1*sunout.get(1),-1*sunout.get(2)]))

        # Rough estimate of velocity from GPS position measurements (finite difference)
        vel_est = (gps_pos_k - gps_pos_km1)/(1/sim_rate)

        # estimated angular momentum unit vector of orbit
        rnorm = gps_pos_k/np.linalg.norm(gps_pos_k)
        velnorm = vel_est/np.linalg.norm(vel_est)
        hhat = np.cross(rnorm, velnorm)

        # estimated desired angular velocity of orbit (assuming circular orbit, in the inertial frame)
        mag = (2*math.pi)/period
        w_des_inertial = mag*hhat
        est_att = ekf_meas.outputs.att_plus_mrp_body_inertial().toDCM() # inertial = DCM * body
        DCMn = np.array([[est_att.get(0,0), est_att.get(0,1), est_att.get(0,2)],
                        [est_att.get(1,0), est_att.get(1,1), est_att.get(1,2)],
                        [est_att.get(2,0), est_att.get(2,1), est_att.get(2,2)]])
        DCMt = np.transpose(DCMn) # body = DCMtranspose * inertial
        w_des = DCMt @ w_des_inertial # desired angular velocity in body frame
        
        pd.inputs.cmd_ang_vel(CartesianVector3([w_des[0],w_des[1],w_des[2]]))

    exc.step() # <---------------- ACTUALLY STEPS THE SIMULATION!

    # The following ifs are to generate two GPS measurements for the velocity measurement in control step

    if second_step:
        gps_pos_km1 = gps_pos_k
        gps_pos_k = np.array([erf_sens.outputs.pos_tgt_ref__out().get(0), erf_sens.outputs.pos_tgt_ref__out().get(1), erf_sens.outputs.pos_tgt_ref__out().get(2)])
        second_step = False

    if first_step:
        gps_pos_k = np.array([erf_sens.outputs.pos_tgt_ref__out().get(0), erf_sens.outputs.pos_tgt_ref__out().get(1), erf_sens.outputs.pos_tgt_ref__out().get(2)])
        first_step = False
        second_step = True   

    if not first_step and not second_step:
        gps_pos_k = np.array([erf_sens.outputs.pos_tgt_ref__out().get(0), erf_sens.outputs.pos_tgt_ref__out().get(1), erf_sens.outputs.pos_tgt_ref__out().get(2)])
        GNCstart = True