"""

Alex Newett Simulation Analysis - Modelspace

"""

import pandas as pd
from modelspaceutils.AutoDocPy import AutoDocPy
from modelspaceutils.analysisutils import plotStateAndCovariance
import matplotlib.pyplot as plt
from modelspace.ModelSpacePy import Quaternion, MRP

truth = pd.read_csv('results/truth.csv')
nav = pd.read_csv('results/nav_log.csv')

## Time vector
sim_time = nav["time"]

## Truth data storage
mrp_truth_0 = []
mrp_truth_1 = []
mrp_truth_2 = []

for i in range(len(truth["quat_true_0"])):
    mrpcurrent = Quaternion([truth['quat_true_0'][i],truth['quat_true_1'][i],truth['quat_true_2'][i],truth['quat_true_3'][i]]).toMRP()
    mrp_truth_0.append(mrpcurrent.get(0))
    mrp_truth_1.append(mrpcurrent.get(1))
    mrp_truth_2.append(mrpcurrent.get(2))

f1 = plt.figure(1)
plt.subplot(3,1,1)
plt.plot(sim_time,mrp_truth_0)
plt.subplot(3,1,2)
plt.plot(sim_time,mrp_truth_1)
plt.subplot(3,1,3)
plt.plot(sim_time,mrp_truth_2)
plt.title("Truth MRP")

f2 = plt.figure(2)
plt.subplot(3,1,1)
plt.plot(sim_time,nav['mrp_minus_0'])
plt.subplot(3,1,2)
plt.plot(sim_time,nav['mrp_minus_1'])
plt.subplot(3,1,3)
plt.plot(sim_time,nav['mrp_minus_2'])
plt.title("ESt MRP")
# plt.subplot(3,1,1)
# plotStateAndCovariance(nav['time'], nav['mrp_minus_0'], mrp_truth_0, nav['cov_minus_0_0'], '', 'MRP Error ()')
# plt.subplot(3,1,2)
# plotStateAndCovariance(nav['time'], nav['mrp_minus_1'], mrp_truth_0, nav['cov_minus_1_1'], '', 'MRP Error ()')
# plt.subplot(3,1,3)
# plotStateAndCovariance(nav['time'], nav['mrp_minus_2'], mrp_truth_0, nav['cov_minus_2_2'], '', 'MRP Error ()')

f3 = plt.figure(3)
plt.subplot(3,1,1)
plotStateAndCovariance(nav['time'], nav['bias_minus_0'], truth['gyro_bias_true_0'], nav['cov_minus_3_3'], '', 'MRP Error ()')
plt.subplot(3,1,2)
plotStateAndCovariance(nav['time'], nav['bias_minus_1'], truth['gyro_bias_true_1'], nav['cov_minus_4_4'], '', 'MRP Error ()')
plt.subplot(3,1,3)
plotStateAndCovariance(nav['time'], nav['bias_minus_2'], truth['gyro_bias_true_2'], nav['cov_minus_5_5'], '', 'MRP Error ()')

f4 = plt.figure(4)
plt.subplot(3,1,1)
plt.plot(sim_time,truth['angvel_true_0'])
plt.subplot(3,1,2)
plt.plot(sim_time,truth['angvel_true_1'])
plt.subplot(3,1,3)
plt.plot(sim_time,truth['angvel_true_2'])
plt.title("Truth Angular Velocity")

plt.show()

