import numpy
import sys
import math
import time
from matplotlib import pyplot

import speed_profile
import old_mpc_solver
import path
import frenetpid


class FrenetUnicycle(object):

    def __init__(self, pt, x=None, u=None, kp=0., ki=0., kd=0., ID='vehicle'):

        if x is None:
            x = [0., 0., 0., 0.]
        self.x = x              # Vehicle state.

        if u is None:
            u = [0., 0.]
        self.u = u              # Vehicle input.

        self.ID = ID

        self.frenet = frenetpid.FrenetPID(pt, kp, ki, kd)
        self.path_pos = path.PathPosition(pt, [x[0], x[1]])

    def update(self, delta_t):
        """Gets a new control input and moves the vehicle. """
        self.u[1] = self._get_omega()
        self._move(delta_t)
        self.path_pos.update_position([self.x[0], self.x[1]])

    def _move(self, delta_t):
        """Moves the vehicle according to the system dynamics. """
        self.x[0] = self.x[0] + delta_t * self.x[3] * math.cos(self.x[2])
        self.x[1] = self.x[1] + delta_t * self.x[3] * math.sin(self.x[2])
        self.x[2] = (self.x[2] + delta_t * self.u[1]) % (2 * math.pi)
        self.x[3] = self.x[3] + delta_t*self.u[0]

    def _get_omega(self):
        """Returns the omega control signal calculated by frenet controller. """
        return self.frenet.get_omega(self.x[0], self.x[1], self.x[2], self.x[3])

    def get_path_pos(self):
        """Returns the position on the path. """
        return self.path_pos.get_position()

    def set_pid(self, kp, ki, kd):
        """Sets the PID parameters of the frenet controller. """
        self.frenet.set_pid(kp, ki, kd)

    def get_error(self):
        """Returns the distance to the path. """
        return self.frenet.get_y_error()

    def get_velocity(self):
        """Returns the velocity of the vehicle. """
        return self.x[3]

    def get_x(self):
        """Returns the current state. """
        return self.x

    def set_x(self, x):
        """Sets the state. """
        self.x = x

    def get_u(self):
        """Returns the current input. """
        return self.u

    def set_u(self, u):
        """Sets the input. """
        self.u = u

    def set_omega(self, omega):
        self.u[1] = omega

    def set_acceleration(self, acc):
        self.u[0] = acc

    def __str__(self):
        """Returns the current state in string format. """
        s = 'ID = {}: x = ['.format(self.ID)
        for x in self.x:
            s += '{:.2f}, '.format(x)

        s = s[:-2]
        s += ']'

        return s


def print_numpy(a):
    str = '['
    for v in a.flatten():
        str += ' {:.2f}'.format(v)
    str += ' ]'

    print(str)


class SimMPC(object):

    def __init__(self, vehicles,
                 Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None,
                 runs=20, saved_h=2, startup_time=1.):

        self.vehicles = vehicles
        self.dt = delta_t
        self.h = horizon
        self.vopt = speed_profile.Speed()
        self.runs = runs
        self.truck_length = truck_length
        self.saved_h = saved_h

        self.time_average = 0
        self.total_elapsed_time = 0

        self.verbose = False

        self.timegap = timegap

        self.startup_time = startup_time

        self.positions = numpy.zeros((len(self.vehicles), runs))
        self.velocities = numpy.zeros((len(self.vehicles), runs))
        self.accelerations = numpy.zeros((len(self.vehicles), runs))
        self.timegaps = numpy.zeros((len(self.vehicles), runs))
        self.path_errors = numpy.zeros((len(self.vehicles), runs))
        self.velocity_errors = numpy.zeros((len(self.vehicles), runs))

        self.mpcs = [None for i in self.vehicles]
        for i, vehicle in enumerate(self.vehicles):
            if i == 0:
                leader = True
            else:
                leader = False
            self.mpcs[i] = old_mpc_solver.TruckMPC(Ad, Bd, delta_t, horizon, zeta, Q,
                                                   R, truck_length, safety_distance,
                                                   timegap, xmin=xmin, xmax=xmax,
                                                   umin=umin, umax=umax, x0=x0,
                                                   vehicle_id=vehicle.ID,
                                                   saved_h=saved_h, is_leader=leader)

        print('\nSimulation initialized. ')

    def run(self):
        """Runs the simulation. """
        print('Simulation running...')
        self.startup()
        start_time = time.time()
        self.run_controller()
        self.total_elapsed_time = time.time() - start_time
        self.finish()

    def finish(self):
        s = 'Simulation finished.'
        s += '\nSimulated time = {:.2f}s. delta_t = {:.2f}. Total elapsed time = {:.2f}s.'.format(
            self.runs*self.dt, self.dt, self.total_elapsed_time)
        s += '\nIterations = {}, average iteration time = {:.4f}s'.format(
            self.runs, self.total_elapsed_time/self.runs)
        s += ' ({:.4f}s / vehicle).'.format(
            self.total_elapsed_time/(self.runs*len(self.vehicles)))
        s += '\nMean path error = {:.2f}, mean timegap = {:.2f}, mean v error = {:.2f}'.format(
            numpy.mean(self.path_errors), numpy.mean(self.timegaps[1:]),
            numpy.mean(self.velocity_errors))

        print(s)

    def startup(self):
        """Starts by driving a bit without feedback. """
        samples = int(round(self.startup_time/self.dt))
        vopt_avg = self.vopt.get_average()
        startup_acc = vopt_avg/((samples - 1)*self.dt)

        for i in range(samples):
            for j, vehicle in enumerate(self.vehicles):
                if i < samples - 1:
                    acc = startup_acc
                else:
                    acc = 0

                vehicle.set_acceleration(acc)
                vehicle.update(self.dt)

                vel = vehicle.get_velocity()
                path_pos = vehicle.get_path_pos()
                self.mpcs[j].set_new_x0(numpy.array([vel, path_pos]))

    def run_controller(self):
        """Runs the control system. """
        s = ''
        for k in range(self.runs):
            if self.verbose:
                s += '\nk = {} - - - - - - - - - - - - - - - - - - -'.format(k)

            for j, vehicle in enumerate(self.vehicles):
                vel = vehicle.get_velocity()
                path_pos = vehicle.get_path_pos()
                self.mpcs[j].set_new_x0(numpy.array([vel, path_pos]))

                if j == 0:
                    self.mpcs[j].compute_optimal_trajectories(self.vopt)
                else:
                    preceding_x = self.mpcs[j - 1].get_assumed_state()

                    self.mpcs[j].compute_optimal_trajectories(
                        self.vopt, preceding_x)

                acc = self.mpcs[j].get_instantaneous_acceleration()

                vehicle.set_acceleration(acc)

                opt_v = self.vopt.get_speed_at(path_pos)

                if self.verbose:
                    s += '\nID = {}, s = {:.2f}, v = {:.2f} ({:.2f}), a = {:.2f}'.format(
                        vehicle.ID, path_pos, vel, opt_v, acc)

                timegap = 0
                if j > 0:
                    distance = self.vehicles[j - 1].get_path_pos() - path_pos
                    vel = vehicle.get_velocity()
                    if vel != 0:
                        timegap = distance/vel
                    else:
                        timegap = 0

                    if self.verbose:
                        s += ', timegap = {:.2f}'.format(timegap)

                self.timegaps[j][k] = timegap
                self.positions[j][k] = path_pos
                self.velocities[j][k] = vel
                self.accelerations[j][k] = acc
                self.path_errors[j][k] = vehicle.get_error()
                self.velocity_errors[j][k] = vel - opt_v

                vehicle.update(self.dt)

        if self.verbose:
            print(s)

    def set_vopt(self, vopt):
        """Sets the optimal speed profile. """
        self.vopt = vopt

    def set_verbose(self, verbose):
        self.verbose = verbose

    def print_graps(self):
        kk = numpy.arange(self.runs)
        tt = kk*self.dt

        pyplot.figure(figsize=(10, 10))

        ax = pyplot.subplot(411)
        ax.set_title('Speed')
        ax.set_xlabel('longitudinal position, m')
        ax.set_ylabel('speed, m/s')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.positions[i], self.velocities[i],
                        label=self.vehicles[i].ID)
        voptend = numpy.argmax(self.vopt.pos > numpy.max(self.positions))
        if voptend == 0:
            voptend = len(self.vopt.pos)
        pyplot.plot(self.vopt.pos[:voptend], self.vopt.vel[:voptend],
                    label='reference')
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(412)
        ax.set_title('Acceleration')
        ax.set_xlabel('longitudinal position, m')
        ax.set_ylabel('acceleration, m/s/s')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.positions[i], self.accelerations[i],
                        label=self.vehicles[i].ID)
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(413)
        ax.set_title('Timegap to preceding vehicle')
        ax.set_xlabel('longitudinal position, m')
        ax.set_ylabel('time, s')
        ax.set_color_cycle(['orange', 'green', 'red'])
        for i in range(1, len(self.vehicles)):
            pyplot.plot(self.positions[i], self.timegaps[i],
                        label=self.vehicles[i].ID)
        pyplot.plot(tt, self.timegap * numpy.ones(len(tt)),
                    label='reference')
        pyplot.legend(loc='upper right')

        ax = pyplot.subplot(414)
        ax.set_title('Path lateral distance error')
        ax.set_xlabel('longitudinal position, m')
        ax.set_ylabel('distance, m')
        for i in range(len(self.vehicles)):
            pyplot.plot(self.positions[i], self.path_errors[i],
                        label=self.vehicles[i].ID)
        pyplot.legend(loc='upper right')

        pyplot.tight_layout(pad=0.5, w_pad=0.5, h_pad=2)
        pyplot.show()


def main(args):
    # ID of the vehicle.
    if len(args) < 2:
        print('Need to enter at least one vehicle ID.')
        sys.exit()

    vehicle_ids = ['/' + v_id for v_id in args[1:]]

    horizon = 10
    delta_t = 0.1
    Ad = numpy.array([1., 0., delta_t, 1.]).reshape(2, 2)
    Bd = numpy.array([delta_t, 0.]).reshape(2, 1)
    zeta = 0.75
    Q_v = 1     # Part of Q matrix for velocity tracking.
    Q_s = 0.5   # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0., 0., Q_s]).reshape(2, 2) # State tracking.
    R_acc = 0.1
    R = numpy.array([1]) * R_acc  # Input tracking.
    v_min = 0.
    v_max = 2.
    s_min = 0.
    s_max = 1000000.
    acc_min = -0.5
    acc_max = 0.5
    truck_length = 0.2
    safety_distance = 0.1
    timegap = 1
    saved_h = 2

    runs = 200

    xmin = numpy.array([v_min, s_min])
    xmax = numpy.array([v_max, s_max])
    umin = numpy.array([acc_min])
    umax = numpy.array([acc_max])

    x_radius = 1.7
    y_radius = 1.2
    center = [0, -y_radius]
    pts = 400

    # Reference speed profile.
    opt_v_pts = 1000
    opt_v_max = 1.3
    opt_v_min = 0.7
    opt_v_period_length = 20
    vopt = speed_profile.Speed()
    vopt.generate_sin(opt_v_min, opt_v_max, opt_v_period_length, opt_v_pts)
    vopt.repeating = True

    pt = path.Path()
    pt.gen_circle_path([x_radius, y_radius], points=pts, center=center)

    kp = 0.5
    ki = -0.02
    kd = 3

    verbose = False
    print_graphs = True

    start_distance = 0.75
    path_len = pt.get_path_length()

    startup_time = 0.5

    vehicles = [None for i in vehicle_ids]

    for i in range(len(vehicle_ids)):
        theta = (len(vehicle_ids) - i - 1)*2*math.pi*start_distance/path_len + 0.1

        theta = 0.1

        x = center[0] + x_radius*math.cos(theta)
        y = center[1] + y_radius*math.sin(theta)
        v = 0

        vehicles[i] = FrenetUnicycle(
            pt, x=[x, y, theta + math.pi/2, v], u=[0, 0], kp=kp, ki=ki, kd=kd, ID=vehicle_ids[i])

    sim = SimMPC(vehicles, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length, safety_distance,
                 timegap, xmin=xmin, xmax=xmax, umin=umin, umax=umax, runs=runs, saved_h=saved_h,
                 startup_time=startup_time)

    sim.set_vopt(vopt)

    sim.set_verbose(verbose)

    sim.run()

    if print_graphs:
        sim.print_graps()


if __name__ == '__main__':
    main(sys.argv)























