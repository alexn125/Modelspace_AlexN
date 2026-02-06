"""
Alex Newett - Modelspace GNC Research 2025

"""
## Important stuff
import sys, math, os
import numpy as np
## Basic imports
from modelspace.Spacecraft import Spacecraft
from modelspace.CustomPlanet import CustomPlanet
from modelspace.SpicePlanet import SpicePlanet
from modelspace.OrbitalElementsStateInit import OrbitalElementsStateInit
## Sensor imports
from modelspace.StarTracker import StarTracker
from modelspace.IMU import IMU
from modelspace.BiasNoiseModel import BiasNoiseModel
# from modelspace.Magnetometer import Magnetometer
## Navigation imports
from modelspace.AttitudeEkfTimeUpdate import AttitudeEkfTimeUpdate
from modelspace.AttitudeEkfMeasUpdate import AttitudeEkfMeasUpdate
from modelspace.SimpleDiscreteProcessNoise import SimpleDiscreteProcessNoise
## Guidance imports
from modelspace.TriadGuidance import TriadGuidance
from modelspace.ReactionWheelModel import ReactionWheelModel
## Control imports
from modelspace.PidAttitudeControl import PidAttitudeControl

## MODELSPACE.PY IMPORTS --------------------------------------------------------------------------------------------------------------------------------------------------
# Base simulation stuff
from modelspace.ModelSpacePy import SimulationExecutive, connectSignals, Time, CsvLogger, Node, START_STEP, END_STEP, NOT_SCHEDULED
# Unit conversions
from modelspace.ModelSpacePy import DEGREES_TO_RADIANS, RADIANS_TO_DEGREES
# Vectors, matrices, attitude
from modelspace.ModelSpacePy import CartesianVector3, CartesianVector4, Matrix6, Matrix63, Matrix3, MRP, Quaternion, Euler321, DCM
# Data IO
from modelspace.ModelSpacePy import DataIOBase, DataIOMatrix3DPtr
# Visuals
from modelspaceutils.vizkit.VizKitPlanetRelative import VizKitPlanetRelative
# Planet relative states model
# from modelspace.PlanetRelativeStatesModel import PlanetRelativeStatesModel
# Frame state sensor model
from modelspace.FrameStateSensorModel import FrameStateSensorModel
## Clear terminal
os.system('cls' if os.name == 'nt' else 'clear')

# REACTION_WHEELS = [(Euler321([0.0, 0.0, 0.0]).toDCM()).toQuaternion(),
#                     (Euler321([0.0, 90.0*DEGREES_TO_RADIANS, 0.0]).toDCM()).toQuaternion(),
#                     (Euler321([0.0, 0.0, 90.0*DEGREES_TO_RADIANS]).toDCM()).toQuaternion()]

"Overall Simulation Setup ------------------------------------------------------------------------------------------------------"

sim_rate = 1 # Hz
sim_length = 100 # seconds

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

# print(earth.outputs.mu())
## Calculate remaining elements
semimajoraxis = (earth.outputs.mu()/(meanmot*meanmot))**(1/3)
# print("Semimajor Axis (m):", semimajoraxis)
period = 2*math.pi*math.sqrt((semimajoraxis**3)/earth.outputs.mu())
# print("Orbital Period (s):", period)
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
        # print(math.atan(math.sqrt((1+ecc)/(1-ecc))))
        # print(math.tan(EccAnom/2))
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
# sc.configureFromDefault("6U")

sc.params.mass(8.0)
sc.params.inertia(Matrix3([[0.026, 0.0, 0.0],
                   [0.0, 0.06, 0.0],
                   [0.0, 0.0, 0.085]]))

# print("Earth mu (m^3/s^2):", earth.outputs.mu())

## Initial Truth Attitude and Angular Velocity
init_attitude_truth = MRP([0.0, 0.1, 0.0])
init_angvel_truth = CartesianVector3([-0.2*DEGREES_TO_RADIANS, 0.2*DEGREES_TO_RADIANS, 0.2*DEGREES_TO_RADIANS])

## Assign initial state parameters
connectSignals(orbels_init.outputs.pos__inertial, sc.params.initial_position)
connectSignals(orbels_init.outputs.vel__inertial, sc.params.initial_velocity)
sc.params.initial_attitude(init_attitude_truth.toQuaternion())

sc.params.initial_ang_vel(init_angvel_truth)
sc.params.planet_ptr(earth)

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
st.params.mount_frame(sc.outputs.body())
st.params.reference_frame(earth.outputs.inertial_frame())

## IMU
imu = IMU(exc, START_STEP, "imu")
imu.params.mount_frame(sc.outputs.body())
imu.params.gyro_bias(CartesianVector3([-1*(1/3600)*DEGREES_TO_RADIANS, 2*(1/3600)*DEGREES_TO_RADIANS, -3*(1/3600)*DEGREES_TO_RADIANS]))

## Sun Sensor
sun_sens = FrameStateSensorModel(exc, START_STEP, "sun_sens")
sun_sens.params.target_frame_ptr(sc.outputs.body())
sun_sens.params.reference_frame_ptr(sun.outputs.inertial_frame())
sun_sens.params.output_frame_ptr(earth.outputs.inertial_frame())                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

## GPS/Earth Sensor
erf_sens = FrameStateSensorModel(exc, START_STEP, "erf_sens")
erf_sens.params.target_frame_ptr(sc.outputs.body())
erf_sens.params.reference_frame_ptr(earth.outputs.inertial_frame())
erf_sens.params.output_frame_ptr(earth.outputs.inertial_frame())

# GPSstd = 500 # meters
# GPSseed = 42
# np.random.seed(GPSseed)

## Magnetomer
# mag = Magnetometer(exc, "mag")
# mag.params.mount_frame(sc.outputs.body())
# mag.params.mag_field_model_frame(earth.outputs.inertial_frame())

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
ekf_meas = AttitudeEkfMeasUpdate(exc, END_STEP, "ekf_meas")
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
triad = TriadGuidance(exc, END_STEP, "triad")
triad.inputs.current_primary_body(CartesianVector3([1.0, 0.0, 0.0]))
triad.inputs.current_secondary_body(CartesianVector3([0.0, 1.0, 0.0]))
connectSignals(erf_sens.outputs.pos_tgt_ref__out, triad.inputs.desired_primary)
connectSignals(sun_sens.outputs.pos_tgt_ref__out, triad.inputs.desired_secondary)

"-------------------------------------------------------------------------------------------------------------------------------"

"Control -----------------------------------------------------------------------------------------------------------------------"
## Control setup
# pd = PidAttitudeControl(exc, END_STEP, "PD")
# pd.params.P(-1/100000)
# pd.params.K(-1/20)

## Reaction wheel orientations (body frame)
# REACTION_WHEELS = [Quaternion([math.cos(math.pi/4),0,math.sin(math.pi/4),0]),
#                     Quaternion([math.cos(math.pi/4),math.sin(math.pi/4),0,0]),
#                     Quaternion([1,0,0,0])]

## Reaction wheel setup
rw0 = ReactionWheelModel(exc, END_STEP, "rw_0")
rw0.params.sc_body(sc.body())
rw0.params.quat_wheel_body(Quaternion([1,0,0,0]))
rw0.params.mom_inertia(0.0004)
rw0.params.peak_torque(1000)
rw0.params.momentum_cap(1000)
rw0.params.mass(0.115)
rw0.params.wheel_location__body(CartesianVector3([0.05,0.0,0.0]))

rw1 = ReactionWheelModel(exc, END_STEP, "rw_1")
rw1.params.sc_body(sc.body())
rw1.params.quat_wheel_body(Quaternion([math.cos(math.pi/4),0,math.sin(math.pi/4),0]))
rw1.params.mom_inertia(0.0004)
rw1.params.peak_torque(1000)
rw1.params.momentum_cap(1000)
rw1.params.mass(0.115)
rw1.params.wheel_location__body(CartesianVector3([0.0,0.05,0.0]))

rw2 = ReactionWheelModel(exc, END_STEP, "rw_2")
rw2.params.sc_body(sc.body())
rw2.params.quat_wheel_body(Quaternion([math.cos(math.pi/4),math.sin(math.pi/4),0,0]))
rw2.params.mom_inertia(0.0004)
rw2.params.peak_torque(1000)
rw2.params.momentum_cap(1000)
rw2.params.mass(0.115)
rw2.params.wheel_location__body(CartesianVector3([0.0,0.0,0.05]))

## Connecting signals
# connectSignals(triad.outputs.quat_body_ref, pd.inputs.cmd_state)
# pd.inputs.cmd_state(Quaternion([1,0,0,0]))
# connectSignals(st.outputs.meas_quat_sf_ref,pd.inputs.act_state)
# connectSignals(imu.outputs.meas_ang_vel_sf,pd.inputs.act_ang_vel)
# w_des = CartesianVector3([0.0,0.0,0.0]) # desired angular velocity
# pd.inputs.cmd_ang_vel(w_des)

"-------------------------------------------------------------------------------------------------------------------------------"

"Logging -----------------------------------------------------------------------------------------------------------------------"
## Save truth data
truth = CsvLogger(exc, "truth.csv")
truth.addParameter(exc.time().base_time,"sim_time")
truth.addParameter(sc.outputs.quat_sc_pci,"quat_true")
truth.addParameter(sc.outputs.ang_vel_sc_pci__body,"angvel_true")
truth.addParameter(imu.params.gyro_bias,"gyro_bias_true")
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
exc.logManager().addLog(guidout, Time(1))

contout = CsvLogger(exc, "cont_log.csv")
contout.addParameter(exc.time().base_time,"sim_time")
# contout.addParameter(pd.outputs.control_cmd, "pd output")
contout.addParameter(rw0.outputs.applied_torque, "torque_rw0")
contout.addParameter(rw1.outputs.applied_torque, "torque_rw1")
contout.addParameter(rw2.outputs.applied_torque, "torque_rw2")
exc.logManager().addLog(contout, Time(1))

posout = CsvLogger(exc, "pos_log.csv")
posout.addParameter(exc.time().base_time,"sim_time")
posout.addParameter(sc.outputs.pos_sc_pci, "pos_sc_pci")
exc.logManager().addLog(posout, Time(1))
"-------------------------------------------------------------------------------------------------------------------------------"

"Visuals -----------------------------------------------------------------------------------------------------------------------"
## Visuals (if on)

# vk_planet_rel = VizKitPlanetRelative(exc)
# connectSignals(earth.outputs.inertial_frame, vk_planet_rel.planet)
# connectSignals(sc.outputs.body, vk_planet_rel.target)

# vk_planet_rel_rate = Time()
# vk_planet_rel_rate.fromDouble(10.0)
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

torquecommand = CartesianVector3([0.0,0.0,0.0])

first_step = True
last_step = False
## Control Gains

Kval = 0.001
Pval = 0.003

K = np.array([[Kval, 0, 0], [0, Kval, 0], [0, 0, Kval]])
P = np.array([[Pval, 0, 0], [0, Pval, 0], [0, 0, Pval]])

while not exc.isTerminated():

    exc.step()

    if first_step:
        first_step = False
        gps_pos_km1 = np.array([erf_sens.outputs.pos_tgt_ref__out().get(0), erf_sens.outputs.pos_tgt_ref__out().get(1), erf_sens.outputs.pos_tgt_ref__out().get(2)])
        innit = orbels_init.outputs.pos__inertial()
        print(gps_pos_km1)
        print(innit.get(0), innit.get(1), innit.get(2))
    if not first_step:
        if exc.simTime() + (1/sim_rate) == sim_length:
            last_step = True
        if not last_step:
            # Preallocation time
            vel_est = np.zeros([3,1])

            # Rough estimate of velocity from GPS position measurements (finite difference)
            gps_pos_k = np.array([erf_sens.outputs.pos_tgt_ref__out().get(0), erf_sens.outputs.pos_tgt_ref__out().get(1), erf_sens.outputs.pos_tgt_ref__out().get(2)])

            # print(gps_pos_k - gps_pos_km1)

            vel_est = (gps_pos_k - gps_pos_km1)/(1/sim_rate)
            # print(exc.simTime(),vel_est)
            vel_est = vel_est/np.linalg.norm(vel_est)

            gps_pos_km1 = gps_pos_k
    # torquecommand = pd.outputs.control_cmd()
    # rw0.inputs.torque_com(-1*torquecommand.get(2))
    # rw1.inputs.torque_com(-1*torquecommand.get(0))
    # rw2.inputs.torque_com(torquecommand.get(1))



    # exc.step()

    # control calculations

    # attitude estimation output
    # if not second_step:
    #     MRPk = ekf_meas.outputs.att_plus_mrp_body_inertial()
    #     sig0 = MRPk.get(0)
    #     sig1 = MRPk.get(1)                                                                  
    #     sig2 = MRPk.get(2)

    #     DCMk = MRPk.toDCM().transpose() # specifically, DCM FROM inertial TO body
    #     s00 = DCMk.get(0,0)
    #     s10 = DCMk.get(1,0)
    #     s20 = DCMk.get(2,0)
    #     s01 = DCMk.get(0,1)
    #     s11 = DCMk.get(1,1)
    #     s21 = DCMk.get(2,1)
    #     s02 = DCMk.get(0,2)
    #     s12 = DCMk.get(1,2)
    #     s22 = DCMk.get(2,2)

    #     wk = imu.outputs.meas_ang_vel_sf() # current angular velocity measurement
    #     wk0 = wk.get(0)
    #     wk1 = wk.get(1)
    #     wk2 = wk.get(2)

    #     mrp_des = triad.outputs.quat_body_ref().toMRP()
    #     mrp_des0 = mrp_des.get(0)
    #     mrp_des1 = mrp_des.get(1)
    #     mrp_des2 = mrp_des.get(2)
        
    #     # estimated velocity
    #     vest0 = rk0-rkm0
    #     vest1 = rk1-rkm1
    #     vest2 = rk2-rkm2 

    #     vestnorm = math.sqrt(vest0**2 + vest1**2 + vest2**2)
    #     vest0 = vest0/(vestnorm)
    #     vest1 = vest1/(vestnorm)
    #     vest2 = vest2/(vestnorm)

    #     rknorm = math.sqrt(rk0**2 + rk1**2 + rk2**2)
    #     rk0n = rk0/(rknorm)
    #     rk1n = rk1/(rknorm)
    #     rk2n = rk2/(rknorm)

    #     # momentum unit vector
    #     hhat0 = rk1n*vest2 - rk2n*vest1
    #     hhat1 = rk2n*vest0 - rk0n*vest2
    #     hhat2 = rk0n*vest1 - rk1n*vest0

    #     # desired angular velocity
    #     wdes0 = ((2*math.pi)/period)*(s00*hhat0 + s01*hhat1 + s02*hhat2)
    #     wdes1 = ((2*math.pi)/period)*(s10*hhat0 + s11*hhat1 + s12*hhat2)
    #     wdes2 = ((2*math.pi)/period)*(s20*hhat0 + s21*hhat1 + s22*hhat2)

    #     # control law
    #     u0 = -1*K*(sig0 - mrp_des0) - P*(wk0 - wdes0)
    #     u1 = -1*K*(sig1 - mrp_des1) + P*(wk1 - wdes1)
    #     u2 = -1*K*(sig2 - mrp_des2) + P*(wk2 - wdes2)

    #     print("at time:",exc.simTime(),"RW Torques (mN-m):", -1000*u2, -1000*u0, 1000*u1)

    #     rw0.inputs.torque_com(-1*u2)
    #     rw1.inputs.torque_com(-1*u0)
    #     rw2.inputs.torque_com(u1)

    #     rkm0 = rk0
    #     rkm1 = rk1
    #     rkm2 = rk2

        # if not first_step:
    #     second_step = False
    # first_step = False
    
    # rk0 = erf_sens.outputs.pos_tgt_ref__out().get(0)
    # rk1 = erf_sens.outputs.pos_tgt_ref__out().get(1)
    # rk2 = erf_sens.outputs.pos_tgt_ref__out().get(2)  

    

"-------------------------------------------------------------------------------------------------------------------------------"

# while not exc.isTerminated():
    # currentsimtime = exc.simTime()
    # check = currentsimtime - math.floor(currentsimtime)

    # rw0.inputs.torque_com(torquecommand.get(0))
    # rw1.inputs.torque_com(torquecommand.get(1))
    # rw2.inputs.torque_com(torquecommand.get(2))
       
    # rw0.step()
    # rw1.step()
    # rw2.step()

    # if abs(check) < tolerance:

    #     # print("sim_time", exc.simTime(), "RW Torques:", rw0.outputs.applied_torque(), rw1.outputs.applied_torque(), rw2.outputs.applied_torque())
    #     print(torquecommand.get(0), torquecommand.get(1), torquecommand.get(2))
    #     ## Navigation

    #     ekf_prop.step()
    #     process_noise.step()
    #     ekf_meas.step()

    #     ## Guidance

    #     # GPS/earth sensor
    #     erf_sens.step() 
    #     GPSout = erf_sens.outputs.pos_tgt_ref__out()
    #     GPSnoised = CartesianVector3([0.0,0.0,0.0]) # preallocate en-noised GPS measurement
    #     GPSnoise = np.random.normal(0,GPSstd,(3,1)) # noise gen
    #     for i in range(3):
    #         GPSnoised.set(i, GPSout.get(i) + GPSnoise[i][0])
    #     # print("at time:",exc.simTime(),"gps meas:",GPSout.get(0), GPSout.get(1), GPSout.get(2))

    #     # sun sensor
    #     sun_sens.step()
    #     sunout = sun_sens.outputs.pos_tgt_ref__out()
    #     sunnorm = math.sqrt(sunout.get(0)**2 + sunout.get(1)**2 + sunout.get(2)**2)
    #     # sun meas w/o noise, direction reversed to get s/c->sun vector and normalized
    #     sun_noised = CartesianVector3([-1*(sunout.get(0)/sunnorm),-1*(sunout.get(1)/sunnorm),-1*(sunout.get(2)/sunnorm)])
    #     # print(sun_noised.get(0), sun_noised.get(1), sun_noised.get(2))
        
    #     # TRIAD
    #     triad.inputs.desired_primary(GPSnoised)
    #     triad.inputs.desired_secondary(sun_noised)
    #     triad.step()

    #     ## Control
    #     pd.step()
        
    #     # errorq = pd.outputs.error_quat()
    #     # print("at time:",exc.simTime(),"error quat:",errorq.get(0), errorq.get(1), errorq.get(2), errorq.get(3))  
    #     # print(exc.simTime())
    #     # K = -1333333.0
    #     torquecommand = pd.outputs.control_cmd()
    #     # for i in range(3):
    #     #     torquecommand.set(i, K*errorq.get(i+1))  # P-control only for now

        

    #     # print(rw0.outputs.applied_torque(), rw1.outputs.applied_torque(), rw2.outputs.applied_torque())

    #     # exc.step()
    #     # current_att = sc.outputs.quat_sc_pci()
    #     # print("at time:",exc.simTime(),"attitude:",current_att.get(0),current_att.get(1),current_att.get(2),current_att.get(3))
    # exc.step()
    # # else:
    # #     rw0.inputs.torque_com(torquecommand.get(0))
    # #     rw1.inputs.torque_com(torquecommand.get(1))
    # #     rw2.inputs.torque_com(torquecommand.get(2))
    # #     rw0.step()
    # #     rw1.step()
    # #     rw2.step()
    # #     # print(rw0.outputs.applied_torque(), rw1.outputs.applied_torque(), rw2.outputs.applied_torque())
    # #     exc.step()

    # # attrn = sc.outputs.quat_sc_pci()
    # # print("Time (s):", exc.simTime(), "Attitude Quaternion:", attrn.get(0), attrn.get(1), attrn.get(2), attrn.get(3))

    # # angv = sc.outputs.ang_vel_sc_pci__body()
    # # print("Time (s):", exc.simTime(), "Angular Velocity (rad/s):", angv.get(0), angv.get(1), angv.get(2))

## OLD STUFF
    # pos = sc.outputs.pos_sc_pci()
    # print("sim_time (s):", exc.simTime(), "Position (m):", pos.get(0), pos.get(1), pos.get(2))

# erf_sens_noise0 = BiasNoiseModel(exc, NOT_SCHEDULED, "erf_sens_noise0")
# erf_sens_noise0.params.noise_std(GPSstd)
# erf_sens_noise0.params.bias(GPSbias)
# erf_sens_noise0.params.seed_value(GPSseed)

# erf_sens_noise1 = BiasNoiseModel(exc, NOT_SCHEDULED, "erf_sens_noise1")
# erf_sens_noise1.params.noise_std(GPSstd)
# erf_sens_noise1.params.bias(GPSbias)
# erf_sens_noise1.params.seed_value(GPSseed + 1)

# erf_sens_noise2 = BiasNoiseModel(exc, NOT_SCHEDULED, "erf_sens_noise2")
# erf_sens_noise2.params.noise_std(GPSstd)
# erf_sens_noise2.params.bias(GPSbias)
# erf_sens_noise2.params.seed_value(GPSseed + 2)      

# print("Semimajor Axis (m):", semimajoraxis)
# print("Eccentricity:", ecc)
# print("Inclination (deg):", inclination)
# print("RAAN (deg):", raan)
# print("Argument of Perigee (deg):", argofp)
# print("True Anomaly (deg):", trueAnom*RADIANS_TO_DEGREES)


#  u0 = -1*K*sig0 - P*wk0 + P*wdes0 + wk2*wdes1 - wk1*wdes2 - wdes2*wk1 + wdes1*wk2
#         u1 = -1*K*sig1 - P*wk1 + P*wdes1 - wk2*wdes0 + wk0*wdes2 + wdes2*wk0 - wdes0*wk2
#         u2 = -1*K*sig2 - P*wk2 + P*wdes2 + wk1*wdes0 - wk0*wdes1 - wdes1*wk0 + wdes0*wk1