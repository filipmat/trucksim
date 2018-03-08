import numpy
import sys
import math
import time

import speed
import truckmpc
import frenet_unicycle
import path


class SimMPC(object):

    def __init__(self, vehicles,
                 Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None,
                 runs=20, saved_h=2):

        self.vehicles = vehicles
        self.dt = delta_t
        self.h = horizon
        self.vopt = speed.Speed()
        self.runs = runs
        self.truck_length = truck_length
        self.saved_h = saved_h

        self.time_average = 0

        self.mpcs = [None for i in self.vehicles]
        for i, vehicle in enumerate(self.vehicles):
            self.mpcs[i] = truckmpc.TruckMPC(Ad, Bd, delta_t, horizon, zeta, Q,
                                             R, truck_length, safety_distance,
                                             timegap, xmin=xmin, xmax=xmax,
                                             umin=umin, umax=umax, x0=x0,
                                             vehicle_id=vehicle.ID,
                                             saved_h=saved_h)

        self.mpcs[0].set_leader(True)

        print('\nSimulation initialized. ')

    def run(self):
        """Runs the simulation. """
        print('Simulation running...')
        self.startup()
        start_time = time.time()
        self.run_controller()
        self.time_average = (time.time() - start_time)/self.runs
        self.finish()

    def finish(self):
        s = 'Simulation finished'
        s += '\nSimulated time = {:.2f}s'.format(self.runs*self.dt)
        s += ', iterations = {}, average iteration time = {:.4f}s'.format(
            self.runs, self.time_average)

        print(s)

    def startup(self):
        """Starts by driving a bit without feedback. """
        samples = int(round(1./self.dt))
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
            s += '\nk = {} - - - - - - - - - - - - - - - - - - - - -'.format(k)

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

                s += '\nID = {}, s = {:.2f}, v = {:.2f} ({:.2f}), a = {:.2f}'.format(
                    vehicle.ID, path_pos, vel, opt_v, acc)

                if j > 0:
                    distance = self.vehicles[j - 1].get_path_pos() - path_pos
                    try:
                        timegap = distance/vehicle.get_velocity()
                    except ZeroDivisionError:
                        timegap = 0

                    s += ', timegap = {:.2f}'.format(timegap)

                vehicle.update(self.dt)

        print(s)

    def set_vopt(self, vopt):
        """Sets the optimal speed profile. """
        self.vopt = vopt


def main(args):
    # ID of the vehicle.
    if len(args) < 2:
        print('Need to enter at least one vehicle ID.')
        sys.exit()

    vehicle_ids = ['/' + v_id for v_id in args[1:]]

    horizon = 3
    delta_t = 0.1
    Ad = numpy.array([1., 0., delta_t, 1.]).reshape(2, 2)
    Bd = numpy.array([delta_t, 0.]).reshape(2, 1)
    zeta = 0.1
    s0 = 0.
    v0 = 1.5
    Q_v = 0.5     # Part of Q matrix for velocity tracking.
    Q_s = 0.1   # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0., 0., Q_s]).reshape(2, 2) # State tracking.
    R_acc = 0.25
    R = numpy.array([1]) * R_acc  # Input tracking.
    v_min = 0.
    v_max = 2.
    s_min = -1000000.
    s_max = 1000000.
    acc_min = -0.5
    acc_max = 0.5
    truck_length = 0.5
    safety_distance = 0.3
    timegap = 0.5
    saved_h = 2

    runs = 100

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
    opt_v_period_length = 100
    vopt = speed.Speed()
    vopt.generate_sin(opt_v_min, opt_v_max, opt_v_period_length, opt_v_pts)
    vopt.repeating = True

    pt = path.Path()
    pt.gen_circle_path([x_radius, y_radius], points=pts, center=center)

    kp = 0.5
    ki = 0
    kd = 3

    start_distance = 0.75
    path_len = pt.get_path_length()

    vehicles = [None for i in vehicle_ids]

    for i in range(len(vehicle_ids)):
        theta = (len(vehicle_ids) - i - 1)*2*math.pi*start_distance/path_len + \
            0.1

        x = center[0] + x_radius*math.cos(theta)
        y = center[1] + y_radius*math.sin(theta)
        v = 0

        vehicles[i] = frenet_unicycle.FrenetUnicycle(
            pt, x=[x, y, theta + math.pi/2, v], u=[0, 0], kp=kp, ki=ki, kd=kd,
            ID=vehicle_ids[i])

    sim = SimMPC(vehicles, Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap, xmin=xmin, xmax=xmax, umin=umin,
                 umax=umax, runs=runs, saved_h=saved_h)

    sim.set_vopt(vopt)

    sim.run()


if __name__ == '__main__':
    main(sys.argv)























