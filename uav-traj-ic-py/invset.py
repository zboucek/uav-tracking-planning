import numpy as np
import matplotlib.pyplot as plt
from numpy.ma.core import concatenate
import polytope as pc
import pickle
import control as ctrl


def calcInvSet(A,B,K,xlim,ulim):

    Ac = A+B@K
    X0 = pc.Polytope(np.concatenate((xlim.A,np.asarray(ulim.A@K))), np.concatenate((xlim.b,ulim.b)))
    i = 0
    while 1:
        P = pc.Polytope(np.concatenate((X0.A,np.asarray(X0.A@Ac))),
                        np.concatenate((X0.b,X0.b)))

        print("iter. {}".format(i))
        if P == X0:
            break
        X0 = P.copy()
        i = i+1
    return X0

if __name__ == "__main__":

    with open('dynamics.pkl', 'rb') as infile:
        result = pickle.load(infile)
    dt, A, B, Q1, Q2, R1, R2 = result

    # load linear constraints as polytopes
    with open('sets.pkl', 'rb') as infile:
        result = pickle.load(infile)
    xlim, ulim, wlim = result

    P,L,K = ctrl.dare(A,B,Q2,R2)
    
    set = calcInvSet(A,B,K,xlim,ulim)
    set.plot()
    plt.show()