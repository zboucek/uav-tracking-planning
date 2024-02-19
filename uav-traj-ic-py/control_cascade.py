from logging import error

import cvxpy as cp
import matplotlib.pyplot as plt
import numpy as np
from multiprocessing import Pool


class ControlCascade:
    """Control cascade for planar UAV attitude control"""

    def __init__(self, ctrl_type='ic', models=None, log_data=False, post_process = False):
        self.pool = Pool(len(models))
        self.ctrl_type = ctrl_type
        self.models = models
        self.log_data = log_data
        self.post_process = post_process
        if self.log_data:
            self.log_state = []
            self.log_traj = []
            self.log_ctrl = []
            self.log_cost = []
            self.log_time = []

        if models is None:
            return None
        # init controllers
        if self.ctrl_type == 'ic':
            from controllers import ControllerIC
            self.controllers = []
            for model in models:
                self.controllers.append(ControllerIC(
                    model.set_h, model.set_l, model.K_h, model.L_h, model.K_l, model.L_l, cp.GUROBI))
        elif self.ctrl_type == 'eic':
            from controllers import ControllerICExtended
            self.controllers = []
            for model in models:
                self.controllers.append(ControllerICExtended(
                    model.set_h, model.set_m, model.set_l, model.K_h, model.L_h, model.K_m, model.L_m, model.K_l, model.L_l, cp.GUROBI))
        elif self.ctrl_type == 'mpc':
            from controllers import ControllerMPC
            self.controllers = []
            for model in models:
                self.controllers.append(ControllerMPC(
                    model.xlim, model.ulim, model.sys.A, model.sys.B, model.R_h, model.Q_h, model.N, cp.GUROBI))
        elif self.ctrl_type == 'mpcmb':
            from controllers import ControllerMPCMB
            self.controllers = []
            for model in models:
                self.controllers.append(ControllerMPCMB(model.xlim, model.ulim, model.sys.A, model.sys.B,
                                        model.R_h, model.Q_h, model.dt, model.dt2, model.Ublock, model.sys2.A,
                                        model.sys2.B, model.Q_h2, model.R_h2, model.N, cp.GUROBI))
        else:
            error("Wrong controller type!")
            return None

    def spin_par(self, traj, state):
        """ Get adequate control action with respect to reference trajectory and current state

            state variable:
                x = [y, z, phi, dy, dz, dphi]'
            Input:
                traj: reference trajectory
                state: current state of UAV
            Return:
                u_att_ref
        """
        udim = 0
        for model in self.models:
            udim = udim + model.sys.B.shape[1]
        u = [None]*udim
        cost = [None]*len(self.models)
        time = [None]*len(self.models)
        
        results = []
        for i, model in enumerate(self.models):
            results.append(self.pool.apply_async(self.controllers[i].spin, [traj[[i, i+state.shape[0]//2], :model.N+1], state[[i, i+state.shape[0]//2]]]))
        
        for i, result in enumerate(results):
            u[i], cost[i], time[i] = result.get()
            if self.post_process:
                if u[i] is None:
                    Warning("Solution of {i}-th controller not found.")
                    if self.log_data and len(self.log_ctrl) > 0:
                        u[i] = self.log_ctrl[-1][i]
                    else:
                        u[i] = 0.0
                u[i] = np.clip(u[i], -model.ulim.b[0], model.ulim.b[1])

        if self.log_data:
            self.log_ctrl.append(u)
            self.log_state.append(state)
            self.log_cost.append(cost)
            self.log_traj.append(traj[:, 0])
            self.log_time.append(time)

        return u
    
    def spin(self, traj, state):
        """ Get adequate control action with respect to reference trajectory and current state

            state variable:
                x = [y, z, phi, dy, dz, dphi]'
            Input:
                traj: reference trajectory
                state: current state of UAV
            Return:
                u_att_ref
        """
        udim = 0
        for model in self.models:
            udim = udim + model.sys.B.shape[1]
        u = [None]*udim
        cost = [None]*len(self.models)
        time = [None]*len(self.models)
        
        for i, model in enumerate(self.models):
            u[i], cost[i], time[i] = self.controllers[i].spin(
                traj[[i, i+state.shape[0]//2], :model.N+1], state[[i, i+state.shape[0]//2]])
            if self.post_process:
                if u[i] is None:
                    Warning("Solution of {i}-th controller not found.")
                    if self.log_data and len(self.log_ctrl) > 0:
                        u[i] = self.log_ctrl[-1][i]
                    else:
                        u[i] = 0.0
                u[i] = np.clip(u[i], -model.ulim.b[0], model.ulim.b[1])
            
        if self.log_data:
            self.log_ctrl.append(u)
            self.log_state.append(state)
            self.log_cost.append(cost)
            self.log_traj.append(traj[:, 0])
            self.log_time.append(time)

        return u

    def plot(self):
        """Print plot of state and control trajectory"""

        log_r = np.array(self.log_traj).T
        log_x = np.array(self.log_state).T
        log_u = np.array(self.log_ctrl).T
        log_c = np.array(self.log_cost).T
        dt = 1e-2
        t = np.linspace(0, log_x.shape[1]*dt, log_x.shape[1])
        plt.figure()

        plt.step(t, log_r[0, :])
        plt.step(t, log_r[1, :])
        if log_x.shape[0] > 6:
            plt.step(t, log_r[2:])

        for log in log_x:
            plt.step(t, log)

        if log_x.shape[0] > 6:
            plt.legend([r'$x_r$', r'$y_r$', r'$z_r$', r'$x$', r'$y$', r'$z$', r'$\phi$', r'$\phi$', r'$\theta$',
                        r'$v_y$', r'$v_z$', r'$\omega_x$', r'$\omega_y$'])
        else:
            plt.legend([r'$y_r$', r'$z_r$', r'$y$', r'$z$',  r'$\phi$',
                        r'$v_y$', r'$v_z$', r'$\omega$'])
        plt.title("UAV State")
        # plt.show()
        plt.figure()
        for log in log_u:
            plt.step(t, log)

        if log_x.shape[0] > 6:
            plt.legend([r'$a_{x,r}$', r'$a_{y,r}$', r'$a_{z,r}$'])
        else:
            plt.legend([r'$a_{y,r}$', r'$a_{z,r}$'])
        plt.title("UAV Control")
        
        
        plt.figure()
        for log in log_c:
            if log.ndim > 1:
                for l in log:
                    plt.step(t, l)
            else:
                plt.step(t, log)

        if log_x.shape[0] > 6:
            if log_c.ndim == 3:
                plt.legend([r'$c_{1,x}$', r'$c_{1,y}$', r'$c_{1,z}$', r'$c_{2,x}$', r'$c_{2,y}$', r'$c_{2,z}$'])
            else:
                plt.legend([r'$c_x$', r'$c_y$', r'$c_z$'])
        else:
            if log_c.ndim == 3:
                plt.legend([r'$c_{1,y}$', r'$c_{1,z}$', r'$c_{2,y}$', r'$c_{2,z}$'])
            else:
                plt.legend([r'$c_y$', r'$c_z$'])
        plt.title("UAV Cost")
        # plt.show()

    def save(self, outfile="log_controller_", name = ""):
        """Save log data as NumPy arrays to npy file"""
        if not self.log_data:
            return
        else:
            state = np.array(self.log_state).T
            ref = np.array(self.log_traj).T
            ctrl = np.array(self.log_ctrl).T
            cost = np.array(self.log_cost).T
            time = np.array(self.log_time).T
            np.save(outfile+self.ctrl_type + name, [state, ref, ctrl, cost, time])

    def plot_path(self):
        """Print 2d/3d plot with reference and real path of the UAV"""
        log_x = np.array(self.log_state).T
        log_traj = np.array(self.log_traj).T
        plt.figure()
        if log_x.shape[0] > 6:
            # from mpl_toolkits import mplot3d
            ax = plt.axes(projection="3d")
            ax.plot3D(log_x[0, :], log_x[1, :], log_x[2, :])
            ax.plot3d(log_traj[0, :], log_traj[1,
                      :], log_traj[2, :])
            ax.set_xlabel(r'$x[m]')
            ax.set_ylabel(r'$y[m]$')
            ax.set_zlabel(r'$z[m]$')
        else:
            plt.plot(log_x[0, :], log_x[1, :])
            plt.plot(log_traj[0, :],
                     log_traj[1, :])
            plt.xlabel(r'$y[m]$')
            plt.ylabel(r'$z[m]$')
        plt.title("UAV position")
        plt.legend([r'position', r'reference'])
        # plt.show()


def example():
    """ Run control cascade for every controller type once
    """

    import pickle

    # load model data
    model_data_folder = "params"
    models_file = {"lateral","lateral", "altitude"}

    # Load parameters of dynamics and cost function
    models = []
    for model_file in models_file:
        with open(model_data_folder+"/"+"ctrl_"+model_file+".pkl", 'rb') as infile:
            models.append(pickle.load(infile))

    # Initialization and data for saving the solution
    rk = np.zeros((12, 5000))
    rk[(0, 1), 100:] = 1
    x0 = np.zeros(12)
    N = 800
    xk_ic = np.zeros((12, N+1))
    xk_ic[:, 0] = x0

    types = ['mpc', 'mpcmb', 'ic']
    for type in types:
        ctrl_ic = ControlCascade(type, models, True)
        u = ctrl_ic.spin(rk[:, :N], xk_ic[:, 0])
        print(type, u)


def example_loop():
    """ Run control cascade for every controller type 
        on the simulated nonlinear planar model of quadrotor UAV
    """

    import pickle

    from nonlinear_dynamics import discreteUav, discreteUavNoise
    from uav_params import UAVParams
    use_lq = False

    # load model data
    model_data_folder = "params"
    # models_file = {"lateral_hummingbird", "altitude_hummingbird"}
    models_file = {"lateral_crazyflie", "altitude_crazyflie"}

    # Load parameters of dynamics and cost function
    models = []
    for model_file in models_file:
        with open(model_data_folder+"/"+"ctrl_"+model_file+".pkl", 'rb') as infile:
            models.append(pickle.load(infile))

    with open(model_data_folder+"/"+"ctrl_"+"attitude_hummingbird"+".pkl", 'rb') as infile:
        model_att = pickle.load(infile)
    pass

    # Initialization and data for saving the solution
    rk = np.zeros((6, 5000))
    rk[0, 100:] = 1.5
    # rk[1,200:] = 0
    x0 = np.zeros((6, 1))
    x0[0, 0] = -0.0
    N = 800
    xk = np.zeros((6, N+1))

    ctrl_types = ['mpc', 'mpcmb', 'ic']
    ctrl_types = ['ic']
    # t = types[1]
    for ctrl_type in reversed(ctrl_types):
        xk[:, 0] = x0[:, 0]

        t0 = 0.0
        # init attitude controller
        if use_lq:
            from controllers import ControllerLQ
            ctrl_att = ControllerLQ(model_att.K_h, model_att.L_h)
        else:
            from controllers import ControllerPID
            Kp = .3
            Kd = 0.3
            Ki = 0.0001
            ctrl_att = ControllerPID(Kp, Kd, Ki, model_att.dt, t0)

        ctrl = ControlCascade(ctrl_type, models, True)

        u_att_ref = 0
        u_thrust = 0
        for i in range(1, 300):
            u_att_ref_temp, u_thrust_temp = ctrl.spin(
                rk[:, i-1:i+N-1], xk[:, i-1])
            if u_att_ref_temp is not None:
                u_att_ref = u_att_ref_temp
            else:
                print("Y: Solution not found!")
            if u_thrust_temp is not None:
                u_thrust = u_thrust_temp
            else:
                print("Z: Solution not found!")
            print(ctrl_type,  u_att_ref, u_thrust)
            xk[:, i] = xk[:, i-1]
            for j in range(10):
                t0 = t0 + model_att.dt
                u_torque = ctrl_att.spin(xk[[2, 5],
                                            i], [u_att_ref, 0])
                if u_torque > UAVParams.tau_max:
                    u_torque = UAVParams.tau_max
                elif u_torque < UAVParams.tau_min:
                    u_torque = UAVParams.tau_min
                xk[:, i] = discreteUav(xk[:, i], np.array(
                    [u_thrust+UAVParams.m*UAVParams.g, u_torque]), model_att.dt)
        ctrl.save()
        ctrl.plot()
        ctrl.plot_path()
        plt.show()
        print("end")

def example_loop_cf():
    """ Run control cascade for every controller type 
        on the simulated nonlinear planar model of Crazyflie quadrotor UAV
    """

    import pickle

    from nonlinear_dynamics import discreteUavCF, discreteUavNoiseCF
    from uav_params import UAVParamsCF
    use_lq = False

    # load model data
    model_data_folder = "params"
    models_file = {"lateral_crazyflie", "altitude_crazyflie"}

    # Load parameters of dynamics and cost function
    models = []
    for model_file in models_file:
        with open(model_data_folder+"/"+"ctrl_"+model_file+".pkl", 'rb') as infile:
            models.append(pickle.load(infile))

    with open(model_data_folder+"/"+"ctrl_"+"attitude_crazyflie"+".pkl", 'rb') as infile:
        model_att = pickle.load(infile)
    pass

    # Initialization and data for saving the solution
    rk = np.zeros((6, 5000))
    rk[0, 100:] = 1.5
    rk[1, 200:] = -0.5
    x0 = np.zeros((6, 1))
    x0[0, 0] = -0.0
    N = 800
    xk = np.zeros((6, N+1))

    ctrl_types = ['mpc', 'mpcmb', 'ic','ica']
    ctrl_types = ['ica']
    # t = types[1]
    for ctrl_type in reversed(ctrl_types):
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

        ctrl = ControlCascade(ctrl_type, models, True)

        u_att_ref = 0
        u_thrust = 0
        for i in range(1, 300):
            u_att_ref_temp, u_thrust_temp = ctrl.spin(
                rk[:, i-1:i+N-1], xk[:, i-1])
            if u_att_ref_temp is not None:
                u_att_ref = -u_att_ref_temp/UAVParamsCF.g
            else:
                print("Y: Solution not found!")
            if u_thrust_temp is not None:
                u_thrust = u_thrust_temp
            else:
                print("Z: Solution not found!")
            print(ctrl_type,  u_att_ref, u_thrust)
            xk[:, i] = xk[:, i-1]
            for j in range(10):
                t0 = t0 + model_att.dt
                u_torque = ctrl_att.spin(xk[[2, 5],
                                            i], [u_att_ref, 0])
                if u_torque > UAVParamsCF.tau_max:
                    u_torque = UAVParamsCF.tau_max
                elif u_torque < UAVParamsCF.tau_min:
                    u_torque = UAVParamsCF.tau_min
                xk[:, i] = discreteUavCF(xk[:, i], np.array(
                    [UAVParamsCF.m*(u_thrust+UAVParamsCF.g), u_torque]), model_att.dt)
        ctrl.save()
        ctrl.plot()
        ctrl.plot_path()
        plt.show()
        print("end")
        
def example_loop_cf_eic():
    """ Run control cascade for eIC controller  
        on the simulated nonlinear planar model of Crazyflie quadrotor UAV
    """

    import pickle

    from nonlinear_dynamics import discreteUavCF, discreteUavNoiseCF
    from uav_params import UAVParamsCF
    use_lq = False

    # load model data
    model_data_folder = "params"
    models_file = {"lateral_crazyflie_eic", "altitude_crazyflie_eic"}

    # Load parameters of dynamics and cost function
    models = []
    for model_file in models_file:
        with open(model_data_folder+"/"+"ctrl_"+model_file+".pkl", 'rb') as infile:
            models.append(pickle.load(infile))

    with open(model_data_folder+"/"+"ctrl_"+"attitude_crazyflie_eic"+".pkl", 'rb') as infile:
        model_att = pickle.load(infile)
    pass

    # Initialization and data for saving the solution
    rk = np.zeros((6, 5000))
    rk[0, 100:] = 1.5
    rk[1, 200:] = -0.5
    x0 = np.zeros((6, 1))
    x0[0, 0] = -0.0
    N = 800
    xk = np.zeros((6, N+1))

    ctrl_types = ['mpc', 'mpcmb', 'ic','eic']
    ctrl_types = ['eic']
    # t = types[1]
    for ctrl_type in reversed(ctrl_types):
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

        ctrl = ControlCascade(ctrl_type, models, True)

        u_att_ref = 0
        u_thrust = 0
        for i in range(1, 300):
            u_att_ref_temp, u_thrust_temp = ctrl.spin(
                rk[:, i-1:i+N-1], xk[:, i-1])
            if u_att_ref_temp is not None:
                u_att_ref = -u_att_ref_temp/UAVParamsCF.g
            else:
                print("Y: Solution not found!")
            if u_thrust_temp is not None:
                u_thrust = u_thrust_temp
            else:
                print("Z: Solution not found!")
            print(ctrl_type,  u_att_ref, u_thrust)
            xk[:, i] = xk[:, i-1]
            for j in range(10):
                t0 = t0 + model_att.dt
                u_torque = ctrl_att.spin(xk[[2, 5],
                                            i], [u_att_ref, 0])
                if u_torque > UAVParamsCF.tau_max:
                    u_torque = UAVParamsCF.tau_max
                elif u_torque < UAVParamsCF.tau_min:
                    u_torque = UAVParamsCF.tau_min
                xk[:, i] = discreteUavCF(xk[:, i], np.array(
                    [UAVParamsCF.m*(u_thrust+UAVParamsCF.g), u_torque]), model_att.dt)
        # ctrl.save()
        ctrl.plot()
        ctrl.plot_path()
        plt.show()
        print("end")

if __name__ == "__main__":
    example_loop_cf_eic()
    # example_loop()
