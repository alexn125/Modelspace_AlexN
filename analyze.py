"""

Alex Newett Simulation Analysis - Modelspace

"""

import pandas as pd
import math
from modelspaceutils.AutoDocPy import AutoDocPy
from modelspaceutils.analysisutils import plotStateAndCovariance
import matplotlib.pyplot as plt
from modelspace.ModelSpacePy import Quaternion, MRP

truth = pd.read_csv('results/truth.csv')
nav = pd.read_csv('results/nav_log.csv')
guid = pd.read_csv('results/guid_log.csv')
cont = pd.read_csv('results/cont_log.csv')

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

    mrpguidcurrent = Quaternion([guid['quat_body_ref_0'][i],guid['quat_body_ref_1'][i],guid['quat_body_ref_2'][i],guid['quat_body_ref_3'][i]]).toMRP()
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
plt.plot(sim_time,mrp_truth_0,sim_time,nav['mrp_minus_0'])
plt.subplot(3,1,2)
plt.plot(sim_time,mrp_truth_1,sim_time,nav['mrp_minus_1'])
plt.subplot(3,1,3)
plt.plot(sim_time,mrp_truth_2,sim_time,nav['mrp_minus_2'])
plt.plot()

# f2 = plt.figure(2)
# plt.subplot(3,1,1)
# plt.plot(sim_time,truth['angvel_true_0'])
# plt.subplot(3,1,2)
# plt.plot(sim_time,truth['angvel_true_1'])
# plt.subplot(3,1,3)
# plt.plot(sim_time,truth['angvel_true_2'])
# plt.title("Truth Angular Velocity")
                        
# f2 = plt.figure(2)
# # plt.subplot(3,1,1)
# # plt.plot(sim_time,nav['mrp_minus_0'])
# # plt.subplot(3,1,2)
# # plt.plot(sim_time,nav['mrp_minus_1'])
# # plt.subplot(3,1,3)
# # plt.plot(sim_time,nav['mrp_minus_2'])
# # plt.title("ESt MRP")
# plt.subplot(3,1,1)
# plotStateAndCovariance(nav['time'], nav['mrp_minus_0'], mrp_truth_0, nav['cov_minus_0_0'], '', 'MRP Error ()')
# plt.subplot(3,1,2)
# plotStateAndCovariance(nav['time'], nav['mrp_minus_1'], mrp_truth_0, nav['cov_minus_1_1'], '', 'MRP Error ()')
# plt.subplot(3,1,3)
# plotStateAndCovariance(nav['time'], nav['mrp_minus_2'], mrp_truth_0, nav['cov_minus_2_2'], '', 'MRP Error ()')

# f3 = plt.figure(3)
# plt.subplot(3,1,1)
# plotStateAndCovariance(nav['time'], nav['bias_minus_0'], truth['gyro_bias_true_0'], nav['cov_minus_3_3'], '', 'MRP Error ()')
# plt.subplot(3,1,2)
# plotStateAndCovariance(nav['time'], nav['bias_minus_1'], truth['gyro_bias_true_1'], nav['cov_minus_4_4'], '', 'MRP Error ()')
# plt.subplot(3,1,3)
# plotStateAndCovariance(nav['time'], nav['bias_minus_2'], truth['gyro_bias_true_2'], nav['cov_minus_5_5'], '', 'MRP Error ()')

f4 = plt.figure(4)
plt.subplot(3,1,1)
plt.plot(sim_time,truth['angvel_true_0'])
plt.subplot(3,1,2)
plt.plot(sim_time,truth['angvel_true_1'])
plt.subplot(3,1,3)
plt.plot(sim_time,truth['angvel_true_2'])
plt.title("Truth Angular Velocity")

f7 = plt.figure(7)
plt.subplot(3,1,1)
plt.plot(sim_time,mrp_error_0,sim_time,Nav_cov_p,sim_time,Nav_cov_n)
plt.subplot(3,1,2)
plt.plot(sim_time,mrp_error_1)
plt.subplot(3,1,3)
plt.plot(sim_time,mrp_error_2)

f8 = plt.figure(8)
plt.subplot(4,1,1)
plt.plot(guid['time'],guid['quat_body_ref_0'])
plt.ylabel('Scalar Part')
plt.subplot(4,1,2)
plt.plot(guid['time'],guid['quat_body_ref_1'])
plt.ylabel('q1')
plt.subplot(4,1,3)
plt.plot(guid['time'],guid['quat_body_ref_2'])
plt.ylabel('q2')
plt.subplot(4,1,4)
plt.plot(guid['time'],guid['quat_body_ref_3'])
plt.ylabel('q3')

f9 = plt.figure(9)
plt.subplot(3,1,1)
plt.plot(sim_time,mrp_guid_0)
plt.subplot(3,1,2)
plt.plot(sim_time,mrp_guid_1)
plt.subplot(3,1,3)
plt.plot(sim_time,mrp_guid_2)
plt.title("TRIAD guidance MRP")

f10 = plt.figure(10)
plt.subplot(3,1,1)
plt.plot(sim_time,cont["pd output_0"],sim_time,cont["torque_rw1"])
plt.legend(['PD Output','RW Torque']) 
plt.subplot(3,1,2)
plt.plot(sim_time,cont["pd output_1"],sim_time,cont["torque_rw2"]) 
plt.subplot(3,1,3)
plt.plot(sim_time,cont["pd output_2"],sim_time,cont["torque_rw0"])

plt.show()

