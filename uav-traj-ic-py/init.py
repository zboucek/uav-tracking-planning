import numpy as np
import polytope as pc
import pickle


# load parameters of dynamics and cost function
with open('dynamics.pkl', 'rb') as infile:
    result = pickle.load(infile)
dt, A, B, Q1, Q2, R1, R2 = result

# load linear constraints as polytopes
with open('sets.pkl', 'rb') as infile:
    result = pickle.load(infile)
xlim, ulim, wlim = result

from scipy.io import loadmat
annots = loadmat('params/ic_params_py.mat')
L1 = annots['ctrl1'][0,0]['L']
K1 = annots['ctrl1'][0,0]['K']
invSet1 = pc.Polytope(annots['ctrl1'][0,0]['F'],
                    annots['ctrl1'][0,0]['g'])
L2 = annots['ctrl2'][0,0]['L']
K2 = annots['ctrl2'][0,0]['K']
invSet2 = pc.Polytope(annots['ctrl2'][0,0]['F'],
                    annots['ctrl2'][0,0]['g'])