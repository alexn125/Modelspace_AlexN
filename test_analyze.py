
import pandas as pd
import math
from modelspaceutils.AutoDocPy import AutoDocPy
from modelspaceutils.analysisutils import plotStateAndCovariance
import matplotlib.pyplot as plt
from modelspace.ModelSpacePy import Quaternion, MRP
from transforms import *

truth = pd.read_csv('results/truth.csv')
nav = pd.read_csv('results/nav_log.csv')
guid = pd.read_csv('results/guid_log.csv')
cont = pd.read_csv('results/cont_log.csv')
# pos = pd.read_csv('results/pos_log.csv')
sen = pd.read_csv('results/sensors.csv')
mrper = pd.read_csv('results/MRPerror.csv')
cmdh = pd.read_csv('results/commandhistory.csv')
gyroh = pd.read_csv('results/gyrohistory.csv')
wh = pd.read_csv('results/whistory.csv')
th = pd.read_csv('results/termshistory.csv')

## Time vector
sim_time = truth["sim_time"]

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

f1 = plt.figure(1)
plt.subplot(3,1,1)
plt.plot(sim_time,mrp_truth_0)
plt.subplot(3,1,2)
plt.plot(sim_time,mrp_truth_1)
plt.subplot(3,1,3)
plt.plot(sim_time,mrp_truth_2)  
plt.title('MRP')  

f2 = plt.figure(2)
plt.subplot(3,1,1)
plt.plot(sim_time,truth["angvel_true_0"])
plt.subplot(3,1,2)
plt.plot(sim_time,truth["angvel_true_1"])
plt.subplot(3,1,3)
plt.plot(sim_time,truth["angvel_true_2"])
plt.title('Angular Velocity')
plt.show()