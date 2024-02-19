import numpy as np
import pickle
import control as ctrl

def dlqtraj(A,B,Q,R,N):
    """ Calculates parameters of discrete-time Linear Quadratic trajectory tracker.

            Input:
                system - LTISystem with quadratic weights for state and input
                N - number of elements in reference trajectory
            Returns: stabilized gain K and compensation L
        
        Returns parameters of LQ controller for following of N-step trajectory
        based on  B. D. O. Anderson and J. B. Moore, Optimal Control. pp. 81:
        Discrete-time tracking

        Controller intended in form u(k) = K*x(k) + L*[xr(k);xr(k+1);...xr(N)] 
    """
    
    P,L,K = ctrl.dare(A,B,Q,R)
    dimu = B.shape[1]
    dimx = A.shape[0]
    Btp = B.T
    Atp = A.T
    K = -K
    Ktp = K.T
    L = np.zeros([dimu,dimx*(N)])
    for i in range(0,N-1):
        L[:,dimx*(i):dimx*(i+1)] = -np.linalg.inv(Btp@P@B+R)@Btp@(np.linalg.matrix_power(Atp+Ktp@Btp,i)@(-Q))

    return K, L

def dlqsp(A,B,Q,R):
    """ Calculates parameters of discrete-time Linear Quadratic setpoint regulator.

            Input:
                system - LTISystem with quadratic weights for state and input
            Returns: stabilized gain K and compensation L
        
        Returns parameters of LQ controller for control to setpoint
        based on  B. D. O. Anderson and J. B. Moore, Optimal Control. pp. 81:
        Discrete-time tracking

        Controller intended in form u(k) = K*x(k) + L*r(k)
    """
    
    P,L,K = ctrl.dare(A,B,Q,R)
    Btp = B.T
    Atp = A.T
    K = -K
    Ktp = K.T
    for i in range(1,int(1e6)):
        Ltemp = np.linalg.matrix_power(Atp + Ktp@Btp,i-1)
        if np.sum(np.sum(np.abs(Ltemp))) == 0:
            break
        L = L + Ltemp
        
    L = np.real(-np.linalg.inv(Btp@P@B+R)@Btp@(L@(-Q)))
    
    return K, L

if __name__ == "__main__":
    with open('dynamics.pkl', 'rb') as infile:
        result = pickle.load(infile)
    dt, A, B, Q1, Q2, R1, R2 = result
    K1sp, L1sp = dlqsp(A,B,Q1,R1)
    K2sp, L2sp = dlqsp(A,B,Q2,R2)
    K1, L1 = dlqtraj(A,B,Q1,R1,800)
    K2, L2 = dlqtraj(A,B,Q2,R2,800)
    
    
    print(K1sp, L1sp)
    print(K2sp, L2sp)
    
    from scipy.io import loadmat
    annots = loadmat('params/ic_params_py.mat')
    L1mat = annots['ctrl1'][0,0]['L']
    K1mat = annots['ctrl1'][0,0]['K']
    L2mat = annots['ctrl2'][0,0]['L']
    K2mat = annots['ctrl2'][0,0]['K']
    
    print(K1, K1mat)
    print(L1, L1mat)
    print(K2, K2mat)
    print(L2, L2mat)