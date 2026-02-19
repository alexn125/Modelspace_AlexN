"""

Alex Newett Simulation Analysis - Modelspace

"""

import pandas as pd
import math
from modelspaceutils.AutoDocPy import AutoDocPy
from modelspaceutils.analysisutils import plotStateAndCovariance
import matplotlib.pyplot as plt
from modelspace.ModelSpacePy import Quaternion, MRP, RADIANS_TO_DEGREES
from transforms import *

truth = pd.read_csv('results/truth.csv')
nav = pd.read_csv('results/nav_log.csv')
guid = pd.read_csv('results/guid_log.csv')
cont = pd.read_csv('results/control_log.csv')
# pos = pd.read_csv('results/pos_log.csv')
sen = pd.read_csv('results/sensors.csv')
mrper = pd.read_csv('results/MRPerror.csv')
cmdh = pd.read_csv('results/commandhistory.csv')
gyroh = pd.read_csv('results/gyrohistory.csv')
wh = pd.read_csv('results/whistory.csv')
th = pd.read_csv('results/termshistory.csv')

## Time vector
sim_time = nav["time"]

## Truth data storage
mrp_truth_0 = []
mrp_truth_1 = []
mrp_truth_2 = []
mrp_error_0 = []
mrp_error_1 = []
mrp_error_2 = []

mrp_guid_0 = []
mrp_guid_1 = []
mrp_guid_2 = []


for i in range(len(truth["quat_true_0"])):
    mrpcurrent = Quaternion([truth['quat_true_0'][i],truth['quat_true_1'][i],truth['quat_true_2'][i],truth['quat_true_3'][i]]).toMRP()
    mrp_truth_0.append(mrpcurrent.get(0))
    mrp_truth_1.append(mrpcurrent.get(1))
    mrp_truth_2.append(mrpcurrent.get(2))

    guidcurrent = Quaternion([guid['quat_body_ref_0'][i],guid['quat_body_ref_1'][i],guid['quat_body_ref_2'][i],guid['quat_body_ref_3'][i]])
    scalar = guidcurrent.get(0)
    v1 = guidcurrent.get(1)
    v2 = guidcurrent.get(2)
    v3 = guidcurrent.get(3)
    if scalar < 0.0:
        mrpguidcurrent = Quaternion([-1*scalar, -1*v1, -1*v2, -1*v3]).toMRP()
    else:
        mrpguidcurrent = Quaternion([scalar,v1,v2,v3]).toMRP()    

    mrp_guid_0.append(mrpguidcurrent.get(0))
    mrp_guid_1.append(mrpguidcurrent.get(1))
    mrp_guid_2.append(mrpguidcurrent.get(2))



    norm = math.sqrt(mrp_truth_0[i]**2 + mrp_truth_1[i]**2 + mrp_truth_2[i]**2)

    if norm > 1.0:   # shadow set transformation
        mrp_truth_0[i] = -mrp_truth_0[i]/(norm**2)
        mrp_truth_1[i] = -mrp_truth_1[i]/(norm**2)
        mrp_truth_2[i] = -mrp_truth_2[i]/(norm**2)

    mrp_error_0.append(mrp_truth_0[i] - nav['mrp_minus_0'][i])
    mrp_error_1.append(mrp_truth_1[i] - nav['mrp_minus_1'][i])
    mrp_error_2.append(mrp_truth_2[i] - nav['mrp_minus_2'][i])




Nav_cov_p = [3.0*math.sqrt(val) for val in nav['cov_plus_0_0']] # 3-sigma covariance
Nav_cov_n = [-1.0*val for val in Nav_cov_p]    

f1 = plt.figure(1)
plt.subplot(3,1,1)
plt.plot(sim_time,mrp_truth_0,sim_time,nav['mrp_minus_0'],sim_time,mrp_guid_0)
plt.subplot(3,1,2)
plt.plot(sim_time,mrp_truth_1,sim_time,nav['mrp_minus_1'],sim_time,mrp_guid_1)
plt.subplot(3,1,3)
plt.plot(sim_time,mrp_truth_2,sim_time,nav['mrp_minus_2'],sim_time,mrp_guid_2)
plt.title('Truth vs. Estimate vs. Guidance (MRP)')
plt.plot()

f2 = plt.figure(2)
plt.subplot(3,1,1)
plt.plot(sim_time,sen['gyro_sen_0']*RADIANS_TO_DEGREES)
# plt.plot(wh['time'],wh['w0'],sen['time'],sen['gyro_sen_0'])
plt.subplot(3,1,2)
plt.plot(sim_time,sen['gyro_sen_1']*RADIANS_TO_DEGREES)
# plt.plot(wh['time'],wh['w1'],sen['time'],sen['gyro_sen_1'])
plt.subplot(3,1,3)
plt.plot(sim_time,sen['gyro_sen_2']*RADIANS_TO_DEGREES)
# plt.plot(wh['time'],wh['w2'],sen['time'],sen['gyro_sen_2'])
plt.title("Gyro-measured Angular Velocity")

print(max(abs(sen['gyro_sen_0']*RADIANS_TO_DEGREES)))
print(max(abs(sen['gyro_sen_1']*RADIANS_TO_DEGREES)))
print(max(abs(sen['gyro_sen_2']*RADIANS_TO_DEGREES)))


f3 = plt.figure(3)
plt.subplot(3,1,1)
plt.plot(cont["time"],cont["torque commands_0"])
plt.subplot(3,1,2)
plt.plot(cont["time"],cont["torque commands_1"])
plt.subplot(3,1,3)
plt.plot(cont["time"],cont["torque commands_2"])
plt.show()

