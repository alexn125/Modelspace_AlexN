"""
Alex Newett - Modelspace GNC Research 2025

"""
## Important stuff
import sys, math, os
## Basic imports
from modelspace.Spacecraft import Spacecraft
from modelspace.CustomPlanet import CustomPlanet
from modelspace.SpicePlanet import SpicePlanet
from modelspace.OrbitalElementsStateInit import OrbitalElementsStateInit
## Sensor imports
from modelspace.StarTracker import StarTracker
from modelspace.IMU import IMU
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

REACTION_WHEELS = [Quaternion([math.cos(math.pi/4),0,math.sin(math.pi/4),0]),
                    Quaternion([math.cos(math.pi/4),math.sin(math.pi/4),0,0]),
                    Quaternion([1,0,0,0])]

"Overall Simulation Setup ------------------------------------------------------------------------------------------------------"
## Simulation Executive
exc = SimulationExecutive()
exc.parseArgs(sys.argv)
exc.setRateHz(10)
exc.end(3600.0)

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

print(semimajoraxis)
## Newton-Raphson to calculate eccentric anomaly
max_iterations = 100
tolerance = 1e-14
its = 0
Eg = meananom
while its<max_iterations:
    out = meananom + (ecc*math.sin(Eg)) - Eg
    if abs(out) < tolerance:
        EccAnom = Eg 
        trueAnom = 2*math.atan(math.sqrt((1+ecc)/(1-ecc))*math.tan(EccAnom/2)) # True anomaly from eccentric anomaly
        break
    else:
        bottom = (ecc*math.cos(Eg)) - 1
        Eg2 = Eg - (out/bottom)
        Eg = Eg2
        its += 1
        if its == max_iterations:
            raise RuntimeError
            
## Spacecraft Object
sc = Spacecraft(exc,"sc")
sc.configureFromDefault("6U")

## Initial Truth Attitude and Angular Velocity
init_attitude_truth = MRP([0.0, 0.1, 0.0])
init_angvel_truth = CartesianVector3([-0.2*DEGREES_TO_RADIANS, 0.2*DEGREES_TO_RADIANS, 0.2*DEGREES_TO_RADIANS])

## Assign initial state parameters
connectSignals(orbels_init.outputs.pos__inertial, sc.params.initial_position)
connectSignals(orbels_init.outputs.vel__inertial, sc.params.initial_velocity)
sc.params.initial_attitude(init_attitude_truth.toQuaternion())
sc.params.initial_ang_vel(init_angvel_truth)
sc.params.planet_ptr(earth)
#! Set mass and MOI tensor

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
st = StarTracker(exc, "st")
st.configureFromDefault("Arcsec_Sagitta")
st.params.mount_frame(sc.outputs.body())
st.params.reference_frame(earth.outputs.inertial_frame())

## IMU
imu = IMU(exc, "imu")
imu.params.mount_frame(sc.outputs.body())
imu.params.gyro_bias(CartesianVector3([-1*(1/3600)*DEGREES_TO_RADIANS, 2*(1/3600)*DEGREES_TO_RADIANS, -3*(1/3600)*DEGREES_TO_RADIANS]))

## Sun Sensor
sun_sens = FrameStateSensorModel(exc,"sun_sens")
sun_sens.params.target_frame_ptr(sc.outputs.body())
sun_sens.params.reference_frame_ptr(sun.outputs.inertial_frame())
sun_sens.params.output_frame_ptr(earth.outputs.inertial_frame())

## Nadir Earth Sensor
erf_sens = FrameStateSensorModel(exc, "erf_sens")
erf_sens.params.target_frame_ptr(earth.outputs.inertial_frame())
erf_sens.params.reference_frame_ptr(sc.outputs.body())
erf_sens.params.output_frame_ptr(earth.outputs.inertial_frame())

## Magnetomer
# mag = Magnetometer(exc, "mag")
# mag.params.mount_frame(sc.outputs.body())
# mag.params.mag_field_model_frame(earth.outputs.inertial_frame())

"-------------------------------------------------------------------------------------------------------------------------------"

"Navigation --------------------------------------------------------------------------------------------------------------------"
## Attitude EKF Time Update (Propagation)
ekf_prop = AttitudeEkfTimeUpdate(exc, NOT_SCHEDULED, "ekf_prop")
connectSignals(ekf_prop.outputs.time_state, ekf_prop.inputs.time_prev)
connectSignals(exc.time().base_time, ekf_prop.inputs.time_update)

## Process Noise 
process_noise = SimpleDiscreteProcessNoise(exc, NOT_SCHEDULED, "process_noise")
process_noise.params.Qc(Matrix3([[3e-9, 0, 0], [0, 3e-9, 0], [0, 0, 3e-9]]))
B = Matrix63([[1,0,0],[0,1,0],[0,0,1],[0,0,0],[0,0,0],[0,0,0]])
process_noise.params.B(B)
process_noise.inputs.time_prev(Time(0))
process_noise.inputs.time_update(Time(1))
connectSignals(ekf_prop.outputs.cov_update, process_noise.inputs.cov_pre_snc)

## Attitude EKF Measurement Update
ekf_meas = AttitudeEkfMeasUpdate(exc, NOT_SCHEDULED, "ekf_meas")
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
triad = TriadGuidance(exc, "triad")
triad.inputs.current_primary_body(CartesianVector3([0.0, 1.0, 0.0]))
triad.inputs.current_secondary_body(CartesianVector3([0.0, 0.0, 1.0]))
connectSignals(sc.outputs.pos_sc_pci, triad.inputs.desired_primary)
connectSignals(sun_sens.outputs.pos_tgt_ref__out, triad.inputs.desired_secondary)

"-------------------------------------------------------------------------------------------------------------------------------"

"Control -----------------------------------------------------------------------------------------------------------------------"
## Control setup
pd = PidAttitudeControl(exc, NOT_SCHEDULED, "PD")
pd.params.P(-1/20)
pd.params.K(-1/10)

## Reaction wheel setup
rw = []
for i in range(len(REACTION_WHEELS)):
    rw.append(ReactionWheelModel(exc, "rw_" + str(i)))
    rw[-1].params.sc_body(sc.body())
    rw[-1].params.quat_wheel_body(REACTION_WHEELS[i])
    rw[-1].params.mom_inertia(0.081)

## Connecting signals
connectSignals(triad.outputs.quat_body_ref, pd.inputs.cmd_state)
connectSignals(st.outputs.meas_quat_sf_ref,pd.inputs.act_state)
connectSignals(imu.outputs.meas_ang_vel_sf,pd.inputs.act_ang_vel)

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

# guidout = CsvLogger(exc, "guid_log.csv")
# guidout.addParameter(exc.time().base_time, "time")     
# exc.logManager().addLog(guidout, Time(1))

help = CsvLogger(exc, "help.csv")
help.addParameter(exc.time().base_time,"sim_time")
help.addParameter(pd.outputs.control_cmd, "command")
help.addParameter(sc.outputs.pos_sc_pci, "sc_eci_pos")
exc.logManager().addLog(help, Time(1))

"-------------------------------------------------------------------------------------------------------------------------------"

"Visuals -----------------------------------------------------------------------------------------------------------------------"
## Visuals (if on)

vk_planet_rel = VizKitPlanetRelative(exc)
connectSignals(earth.outputs.inertial_frame, vk_planet_rel.planet)
connectSignals(sc.outputs.body, vk_planet_rel.target)

vk_planet_rel_rate = Time()
vk_planet_rel_rate.fromDouble(10.0)
exc.logManager().addLog(vk_planet_rel, vk_planet_rel_rate)

# vk = VizKitPlanetRelative(exc)
# vk.target(sc.outputs.body())
# vk.planet(earth.outputs.inertial_frame())
# exc.logManager().addLog(vk, Time(100))

"-------------------------------------------------------------------------------------------------------------------------------"

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

stepval = 0

## Run simulation
while not exc.isTerminated():
    if stepval == 0:
        ekf_prop.step()
        process_noise.step()
        ekf_meas.step()
        triad.step()
        pd.step()
        
        # bruh = erf_sens.outputs.pos_tgt_ref__out()
        # print('earth sensor ---------')
        # print(bruh.get(0))
        # print(bruh.get(1))
        # print(bruh.get(2))
        # print('sc pos ---------')
        # bruh2 = sc.outputs.pos_sc_pci()
        # print(bruh2.get(0))
        # print(bruh2.get(1))
        # print(bruh2.get(2))
        # print('---------')

        # suns = erf_sens.outputs.pos_tgt_ref__out()

        # sunsmag = math.sqrt(suns.get(0)**2 + suns.get(1)**2 + suns.get(2)**2)
        # if exc.simTime() != 0.0:
        #     print('---------')
        #     print(suns.get(0)/sunsmag)
        #     print(suns.get(1)/sunsmag)
        #     print(suns.get(2)/sunsmag)
        #     print('---------')
        # Torque commands
        torquecommand = pd.outputs.control_cmd()
        rw[0].inputs.torque_com(torquecommand.get(0))
        rw[1].inputs.torque_com(torquecommand.get(1))
        rw[2].inputs.torque_com(torquecommand.get(2))
        # print(exc.simTime())

    stepval += 1
    if stepval == 10:
        stepval = 0

    exc.step()

# print('periapsis')
# print((semimajoraxis*(1-ecc) - (6378.14*1000))/1000)
# print('apoapsis')
# print((semimajoraxis*(1+ecc) - (6378.14*1000))/1000)
# print(inclination)
# print(raan)
# print(argofp)
# print(trueAnom*RADIANS_TO_DEGREES)

# bruh3 = orbels_init.outputs.pos__inertial()
# print(bruh3.get(0))
# print(bruh3.get(1))
# print(bruh3.get(2))


"-------------------------------------------------------------------------------------------------------------------------------"