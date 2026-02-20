import math
import numpy as np

def skew_sym(v):  # returns numpy skew symmetric matrix of a 3x1 vector
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])

def shadowset(s):  # returns shadow transformation of MRP (numpy array)
    mag = np.transpose(s) @ s
    return (-1*s)/(mag)

def MRPsubtract(s1,s2): # relative MRP attitude vector (error), np array
    s1s = np.transpose(s1) @ s1
    s2s = np.transpose(s2) @ s2
    numerator = (1-s2s)*s1 - (1-s1s)*s2 + 2*np.cross(s1,s2)
    denominator = (1+s1s*s2s) + 2*np.dot(s1,s2)
    return numerator/denominator

def quat2MRP(q): # converts quaternions (scalar first) to MRPs, np array
    mrp = np.array([q[1]/(1+q[0]), q[2]/(1+q[0]), q[3]/(1+q[0])])
    return mrp    





