from control_cascade import ControlCascade
import numpy as np
import time


class ReferenceTrajectory(object):
    def __init__(self, curve="figure8", ts=0.01, N=50000, space=1.5, tscale=0.2):
        """Create object of reference trajetcory

        Args:
            curve (str, optional): figure8 or spiral. Defaults to "figure8".
            ts (float, optional): sampling period of reference trajectory. Defaults to 0.01.
            N (int, optional): number of samples. Defaults to 50000.
            space (float, optional): size of flight space in m. Defaults to 2.
        """
        self.t = np.cumsum(ts*np.ones(N))
        self.tscale = tscale
        if hasattr(space, "__len__"):
            self.space = space
        else:
            self.space = np.array([space, space, space])
        if hasattr(tscale, "__len__"):
            self.tscale = tscale
        else:
            self.tscale = np.array([tscale, tscale, tscale])
        self.curve = curve
        if self.curve == "figure8":
            self.figure8()
        else:
            self.spiral()

    def figure8(self):
        """Lemniscate of Gerono/Figure8 trajectory"""
        self.x = self.space[0]*np.cos(self.tscale[0]*self.t)
        self.y = self.space[1]*np.sin(self.tscale[1]*2*self.t) / 2
        self.z = -self.space[2]*np.cos(self.tscale[2]*3*self.t)*0.8 + self.space[2]


    def spiral(self):
        """Spiral trajectory"""
        self.x = self.space[0]*np.cos(self.tscale[0]*self.t)
        self.y = self.space[1]*np.sin(self.tscale[1]*self.t)
        self.z = -self.space[2]*np.cos(self.tscale[2]*3*self.t)*0.8 + self.space[2]

def control_loop(ctrl_type="ic", Nsteps=6000, reference=None, use_lq=False, noise=True, print_plot=True, save_results=False, save_folder=""):
    """ Run control cascade for every controller type 
        on the simulated nonlinear planar model of quadrotor UAV
    """

    from nonlinear_dynamics import discreteUavCF, discreteUavNoiseCF
    import pickle
    import numpy as np
    import matplotlib.pyplot as plt
    from uav_params import UAVParamsCF

    # load model data
    model_data_folder = "params"
    if ctrl_type == "eic":
        models_file = {"lateral_crazyflie_eic", "altitude_crazyflie_eic"}
    else:
        models_file = {"lateral_crazyflie", "altitude_crazyflie"}

    # Load parameters of dynamics and cost function
    models = []
    for model_file in models_file:
        with open(model_data_folder+"/"+"ctrl_"+model_file+".pkl", 'rb') as infile:
            models.append(pickle.load(infile))

    with open(model_data_folder+"/"+"ctrl_"+"attitude_crazyflie_eic"+".pkl", 'rb') as infile:
        model_att = pickle.load(infile)
    pass

    # Initialization and data for saving the solution
    rk = np.zeros((6, reference.t.shape[0]))
    rk[0, :] = reference.x
    rk[1, :] = reference.y
    x0 = np.zeros((6, 1))
    # x0[0, 0] = reference.x[0]
    # x0[1, 0] = reference.y[0]
    N = 800
    xk = np.zeros((6, reference.t.shape[0]))

    xk[:, 0] = x0[:, 0]

    t0 = 0.0
    # init attitude controller
    if use_lq:
        from controllers import ControllerLQ
        ctrl_att = ControllerLQ(model_att.K_h, model_att.L_h)
    else:
        from controllers import ControllerPID
        Kp = 0.3
        Kd = 0.003
        Ki = 0.0001
        ctrl_att = ControllerPID(Kp, Kd, Ki, model_att.dt, t0)

    ctrl = ControlCascade(ctrl_type, models, True, post_process=True)

    time_cascade = []
    u_att_ref = 0
    u_thrust = 0
    for i in range(1, Nsteps):
        t_start = time.time()
        u_att_ref_temp, u_thrust_temp = ctrl.spin(
            rk[:, i-1:i+N-1], xk[:, i-1])
        time_cascade.append(time.time() - t_start)
        if u_att_ref_temp is not None:
            u_att_ref = -u_att_ref_temp/UAVParamsCF.g
        else:
            Warning("Y: Solution not found!")
        if u_thrust_temp is not None:
            u_thrust = UAVParamsCF.m*(u_thrust_temp+UAVParamsCF.g)
        else:
            Warning("Z: Solution not found!")
        xk[:, i] = xk[:, i-1]
        for j in range(10):
            t0 = t0 + model_att.dt
            u_torque = ctrl_att.spin(xk[[2, 5],
                                        i], [u_att_ref, 0])
            u_torque = np.clip(u_torque, UAVParamsCF.tau_min, UAVParamsCF.tau_max)
            if noise:
                xk[:, i] = discreteUavNoiseCF(xk[:, i], np.array(
                    [u_thrust, u_torque]), model_att.dt)
            else:
                xk[:, i] = discreteUavCF(xk[:, i], np.array(
                    [u_thrust, u_torque]), model_att.dt)
                
    ctrl.pool.close()
    if save_results:
        path = save_folder + "log_controller_"+reference.curve+"_"
        path_time = save_folder + "time_"+reference.curve+"_"
        if noise:
            path = path+"noise_"
            path_time = path_time+"noise_"
        ctrl.save(path)
        np.save(path_time+ctrl_type, np.array(time_cascade))
    if print_plot:
        ctrl.plot()
        ctrl.plot_path()
        plt.show()
    if noise:
        print(ctrl_type+"_"+reference.curve+"_noise"+" test finished.")
    else:
        print(ctrl_type+"_"+reference.curve+" test finished.")


def test_controllers():
    save_folder = "results/2d_nonlin/data/"
    ctrl_types = ["ic", "mpcmb","mpc", "eic"]
    ctrl_types = ["ic", "eic"]
    ref_types = ["spiral", "figure8"]
    # ctrl_types = ["eic"]
    # ref_types = ["figure8"]
    noise_types = [False]
    omega = 0.4
    Nsteps=6000
    for noise_type in noise_types:
        for ref_type in ref_types:
            if ref_type == "spiral":
                refTraj = ReferenceTrajectory(curve=ref_type, space = [1.8, 1.0, 1.0], tscale = omega)
            else:
                refTraj = ReferenceTrajectory(curve=ref_type, space = [1.8, 1.5, 1.0], tscale = omega)
            for ctrl_type in ctrl_types:
                control_loop(reference=refTraj, Nsteps=Nsteps, ctrl_type=ctrl_type, use_lq=False,
                            noise=noise_type, print_plot=False, save_results=True, save_folder=save_folder)

if __name__ == "__main__":
    # control_loop(ctrl_type="ic")
    test_controllers()
