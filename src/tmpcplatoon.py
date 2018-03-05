#!/usr/bin/env python

import sys
import time
import numpy

import truckmpc
import speed


# TEST
def print_numpy(a):
    str = '['
    for v in a.flatten():
        str += ' {:.2f}'.format(v)
    str += ' ]'

    print(str)
# TEST


class Controller(object):
    """Class for subscribing to vehicle positions, calculate control input, and
    send commands to the vehicle. """

    def __init__(self, vehicle_ids,
                 Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
                 safety_distance, timegap,
                 xmin=None, xmax=None, umin=None, umax=None, x0=None, QN=None,
                 runs=20):

        self.vehicle_ids = ['/' + v_id for v_id in vehicle_ids]  # IDs.
        self.running = False  # If controller is running or not.

        self.dt = delta_t
        self.h = horizon

        self.vopt = speed.Speed()

        self.runs = runs

        self.truck_length = truck_length

        self.mpcs = dict()
        self.iopt_s = dict()
        self.iopt_v = dict()
        for i, vehicle_id in enumerate(self.vehicle_ids):
            x0[1] = 2*((len(self.vehicle_ids) - 1) - i)

            self.iopt_v[vehicle_id] = x0[0]
            self.iopt_s[vehicle_id] = x0[1]
            self.mpcs[vehicle_id] = truckmpc.TruckMPC(Ad, Bd, delta_t, horizon,
                                                      zeta, Q, R, truck_length,
                                                      safety_distance, timegap,
                                                      xmin=xmin, xmax=xmax,
                                                      umin=umin, umax=umax,
                                                      x0=x0, QN=QN,
                                                      t_id=vehicle_id)
            if i == 0:
                self.mpcs[vehicle_id].set_leader(True)


        print('\nController initialized. Vehicles {}.\n'.format(
            self.vehicle_ids))

    def set_vopt(self, vopt):
        """Sets the optimal speed profile. """
        self.vopt = vopt

    def _control(self):
        """Perform control actions from received data. Sends new values to
        truck. """
        for j in range(self.runs):
            if self.running:
                print('\nj = {} - - - - - - - - - - - - - - - - - - '.format(j))

                for vehicle_id in self.vehicle_ids:

                    acc = self._get_acc(vehicle_id)

    def _get_acc(self, vehicle_id):
        """Returns the velocity control signal for the given vehicle. Calculates
        the distance error to the vehicle in front and gets the velocity from
        the PID controller. """
        print('id = {} -------'.format(vehicle_id))
        m = self.mpcs[vehicle_id]

        ass = m.get_assumed_state()

        v = self.iopt_v[vehicle_id]
        s = self.iopt_s[vehicle_id]

        self.mpcs[vehicle_id].set_new_x0(numpy.array([v, s]))

        if self.vehicle_ids.index(vehicle_id) == 0:

            m.compute_optimal_trajectories(self.vopt)
            acc_trajectory = m.get_input_trajectory()
            acc = acc_trajectory[0]

        else:
            id_prec = self.vehicle_ids[self.vehicle_ids.index(vehicle_id) - 1]
            preceding_x = self.mpcs[id_prec].get_assumed_state()

            m.compute_optimal_trajectories(self.vopt, preceding_x)

            s_prec = self.iopt_s[id_prec]
            gaplength = s_prec - self.truck_length - s
            try:
                timegap = gaplength/v
            except:
                timegap = 999

            print('timegap = {:.2f}'.format(timegap))

        acc_trajectory = m.get_input_trajectory()
        acc = acc_trajectory[0]

        # TEST
        print('s = {:.2f}, v = {:.2f}, a = {:.2f}'.format(s, v, acc))

        print('s_ref: '),
        print_numpy(m.pos_ref)
        print('v_ref: '),
        print_numpy(m.vel_ref)
        print('a_ref: '),
        print_numpy(m.acc_ref)
        print('old_x: '),
        print_numpy(ass[:self.h * 2])
        print('ass_x: '),
        print_numpy(ass[self.h * 2:])
        print('a_tra: '),
        print_numpy(acc_trajectory)
        # TEST

        self.iopt_v[vehicle_id] = ass[self.h*2 + 2]
        self.iopt_s[vehicle_id] = ass[self.h*2 + 3]

        return acc


    def stop(self):

        if self.running:
            self.running = False
            print('Controller stopped.\n')

    def start(self):
        """Starts the controller. """
        if not self.running:
            self.running = True
            print('Controller started.')

    def run(self):
        """Runs the controller. """
        self.start()
        self._control()
        self.stop()


def main(args):
    # ID of the vehicle.
    if len(args) < 2:
        print('Need to enter at least one vehicle ID.')
        sys.exit()

    vehicle_ids = args[1:]

    horizon = 3
    delta_t = 0.2
    Ad = numpy.array([1., 0., delta_t, 1.]).reshape(2, 2)
    Bd = numpy.array([delta_t, 0.]).reshape(2, 1)
    zeta = 0.1
    s0 = 0.
    v0 = 1.5
    Q_v = 0.5     # Part of Q matrix for velocity tracking.
    Q_s = 0.1   # Part of Q matrix for position tracking.
    Q = numpy.array([Q_v, 0., 0., Q_s]).reshape(2, 2) # State tracking.
    QN = numpy.zeros((2, 2))
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

    runs = 5

    x0 = numpy.array([v0, s0])
    xmin = numpy.array([v_min, s_min])
    xmax = numpy.array([v_max, s_max])
    umin = numpy.array([acc_min])
    umax = numpy.array([acc_max])

    # Test optimal speed profile.
    opt_step = 20
    opt_v = 1.
    pos = opt_v * delta_t * numpy.arange(opt_step)
    vel = opt_v * numpy.ones(opt_step)
    vopt = speed.Speed(pos, vel)

    # Initialize controller.
    controller = Controller(vehicle_ids,
        Ad, Bd, delta_t, horizon, zeta, Q, R, truck_length,
        safety_distance, timegap,
        xmin=xmin, xmax=xmax,
        umin=umin, umax=umax, x0=x0, QN=QN, runs=runs
    )

    controller.set_vopt(vopt)

    # Start controller.
    controller.run()


if __name__ == '__main__':
    main(sys.argv)
