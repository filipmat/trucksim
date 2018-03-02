#!/usr/bin/env python

# Class for describing a path, and class for defining a new path.

import Tkinter as Tk
import os
import math


class Path:
    """Class for a path. Path is described by a series of coordinate pairs."""

    def __init__(self):
        self.path = []
        self.gamma = []
        self.gammap = []
        self.gammapp = []
        self._lp = True  # Used for graphical application.

        self.xr = 0
        self.yr = 0
        self.xc = 0
        self.yc = 0

    def save(self, filename):
        """Saves path to file. Writes each line on the format x,y"""
        try:
            __location__ = os.path.realpath(os.path.join(os.getcwd(),
                                                         os.path.dirname(
                                                             __file__)))
            fl = open(os.path.join(__location__, filename), 'w')

            for xy in self.path:
                fl.write('{},{}\n'.format(xy[0], xy[1]))

            fl.close()
            print('Saved path as {}'.format(filename))

        except Exception as e:
            print('\nError when saving path to file: {}'.format(e))

    def load(self, filename):
        """Loads path from file. Assumes lines to be on the format x,y"""
        self.path = []
        try:
            __location__ = os.path.realpath(os.path.join(os.getcwd(),
                                                         os.path.dirname(
                                                             __file__)))
            fl = open(os.path.join(__location__, filename), 'r')

            for line in fl:
                line_list = line.strip().split(',')

                if len(line_list) > 1:
                    x = float(line_list[0])
                    y = float(line_list[1])
                    self.path.append((x, y))

            self._calc_gammas()

            fl.close()

        except Exception as e:
            print('\nError when loading path from file: {}'.format(e))

    def gen_circle_path(self, radius, points=300, center=None):
        """Generates a circle path with specified radius and number of
        points."""
        if center is None:
            center = [0, 0]

        newpath = []
        if isinstance(radius, list):
            if len(radius) > 1:
                x_mag = radius[0]
                y_mag = radius[1]
            else:
                x_mag = radius[0]
                y_mag = radius[0]
        else:
            x_mag = radius
            y_mag = radius

        for i in range(points):
            x = center[0] + x_mag * math.cos(2 * math.pi * i / points)
            y = center[1] + y_mag * math.sin(2 * math.pi * i / points)
            newpath.append([x, y])

        self.path = newpath
        self._calc_gammas()

        self.xr = x_mag
        self.yr = y_mag
        self.xc = center[0]
        self.yc = center[1]

        return x_mag, y_mag, center[0], center[1]

    def reverse(self):
        """Reverses the path. Recalculates gamma values."""
        self.path.reverse()
        self._calc_gammas()

    def interpolate(self):
        """Interpolates the path. Returns a denser list that has added one
        set of intermediate points to the original path."""
        length = len(self.path)

        temp = [[0, 0] for j in range(length)]
        for i in range(length - 1):
            temp[i][0] = (self.path[i][0] + self.path[i + 1][0]) / 2
            temp[i][1] = (self.path[i][1] + self.path[i + 1][1]) / 2
        temp[length - 1][0] = (self.path[length - 1][0] + self.path[0][0]) / 2
        temp[length - 1][1] = (self.path[length - 1][1] + self.path[0][1]) / 2

        newlist = [[0, 0] for j in range(2 * length)]
        for i in range(length - 1):
            newlist[i * 2] = self.path[i]
            newlist[i * 2 + 1] = temp[i]
        newlist[2 * length - 1] = temp[length - 1]
        newlist[2 * length - 2] = self.path[length - 1]

        self.path = newlist
        self._calc_gammas()

    def split(self):
        """Returns two lists, one containing x and one containing y."""
        xlist = [a for a, b in self.path]
        ylist = [b for a, b in self.path]

        return xlist, ylist

    def printp(self):
        """Prints the path in the terminal."""
        for xy in self.path:
            print('{:07.4f}, {:07.4f}'.format(xy[0], xy[1]))

    def get_xy(self, index):
        """# Returns x and y at the given index."""
        try:
            index = index % len(self.path)
            x = self.path[index][0]
            y = self.path[index][1]
            return [x, y]

        except Exception as e:
            print('\nError when retrieving x and y: {}'.format(e))
            return [0, 0]

    def get_closest(self, xy):
        """Return the closest x and y of the path to the given coordinates,
        as well as the index of the path list it is found on."""
        try:
            closest = min(self.path,
                          key=lambda a: (a[0] - xy[0]) ** 2 + (
                                  a[1] - xy[1]) ** 2)
            index = self.path.index(closest)
            return index, closest

        except Exception as e:
            print('\nError when retrieving closest point on path: {}'.format(e))
            return 0, [0, 0]

    def get_tangent(self, index):
        """Returns a unit vector approximating the tangent direction at the
        given index. The tangent points in the direction of the path. """
        try:
            index = index % len(self.path)

            if index == len(self.path) - 1:
                x2 = self.path[0][0]
                y2 = self.path[0][1]
            else:
                x2 = self.path[index + 1][0]
                y2 = self.path[index + 1][1]

            vec = [x2 - self.path[index - 1][0],
                   y2 - self.path[index - 1][1]]
            vec_norm = math.sqrt(vec[0] ** 2 + vec[1] ** 2)
            vec[0] = vec[0] / vec_norm
            vec[1] = vec[1] / vec_norm

            return vec

        except Exception as e:
            print('\nError when calculating tangent: {}'.format(e))
            return [0, 0]

    def get_normal(self, index):
        """Returns the normal vector to the path at the given index. The
        normal points to the right of the path."""
        index = index % len(self.path)
        v = self.get_tangent(index)
        return [v[1], -v[0]]

    def is_left(self, xy):
        """Returns True if point (x, y) is to the left of the path. """
        index, closest = self.get_closest(xy)

        n = self.get_normal(index)

        val = n[0] * (xy[0] - closest[0]) + n[1] * (xy[1] - closest[1])

        if val >= 0:
            return False
        else:
            return True

    def get_ey(self, xy):
        """Returns the y error of the given point (x, y). The error is positive
        if the point is to the left of the path. """
        index, closest = self.get_closest(xy)
        ey = math.sqrt((xy[0] - closest[0]) ** 2 + (xy[1] - closest[1]) ** 2)
        if self.is_left(xy):
            ey = -ey

        return ey

    def get_gamma(self, index):
        """Returns gamma at the given index."""
        try:
            index = index % len(self.path)
            return self.gamma[index]
        except Exception as e:
            print('\nError when getting gamma: {}'.format(e))
            return 0

    def get_gammap(self, index):
        """Returns gamma prime at the given index."""
        try:
            index = index % len(self.path)
            return self.gammap[index]
        except Exception as e:
            print('\nError when getting gamma prime: {}'.format(e))
            return 0

    def get_gammapp(self, index):
        """Returns gamma prime prime at the given index."""
        try:
            index = index % len(self.path)
            return self.gammapp[index]
        except Exception as e:
            print('\nError when getting gamma prime prime: {}'.format(e))
            return 0

    def _calc_gammas(self):
        """Used internally to calculate gammas."""
        self._calc_gamma()
        self._calc_gammap()
        self._calc_gammapp()

    def _calc_gamma(self):
        """Used internally to calculate gamma values and save them."""
        self.gamma = [0 for j in range(len(self.path))]

        for i in range(len(self.path)):
            x1 = self.path[i - 1][0]
            y1 = self.path[i - 1][1]
            if i == len(self.path) - 1:  # At end of list, grab first of list.
                x2 = self.path[0][0]
                y2 = self.path[0][1]
            else:
                x2 = self.path[i + 1][0]
                y2 = self.path[i + 1][1]

            try:
                angle = math.atan((y2 - y1) / (x2 - x1))

                if x2 < x1:  # Calculate 3rd and 4th quadrant correctly.
                    angle = angle + math.pi
                elif y2 < y1:  # Make angles in range 0 to 2 pi.
                    angle = angle + 2 * math.pi

                self.gamma[i] = angle

            except ZeroDivisionError:  # Angle is pi/2 or 3pi/2
                if y2 > y1:
                    self.gamma[i] = math.pi / 2
                else:
                    self.gamma[i] = 3 * math.pi / 2

    def _calc_gammap(self):
        """Used internally to calculate gamma prime."""
        self.gammap = [0 for j in range(len(self.path))]

        for i in range(len(self.gamma)):
            x1 = self.path[i - 1][0]
            y1 = self.path[i - 1][1]
            x2 = self.path[i][0]
            y2 = self.path[i][1]
            if i == len(self.path) - 1:  # At end of list, grab first of list.
                x3 = self.path[0][0]
                y3 = self.path[0][1]
                gammap3 = self.gamma[0]
            else:
                x3 = self.path[i + 1][0]
                y3 = self.path[i + 1][1]
                gammap3 = self.gamma[i + 1]

            gamma_diff = gammap3 - self.gamma[i - 1]  # Make sure difference
            while gamma_diff > 2 * math.pi:  # is between 0 and 2 pi.
                gamma_diff = gamma_diff - 2 * math.pi
            while gamma_diff < 0:
                gamma_diff = gamma_diff + 2 * math.pi

            self.gammap[i] = gamma_diff / (
                    math.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2) + math.sqrt(
                        (x2 - x1) ** 2 + (y2 - y1) ** 2))

    def _calc_gammapp(self):
        """Used internally to calculate gamma prime prime."""
        self.gammapp = [0 for j in range(len(self.path))]

        for i in range(len(self.gamma)):
            x1 = self.path[i - 1][0]
            y1 = self.path[i - 1][1]
            x2 = self.path[i][0]
            y2 = self.path[i][1]
            if i == len(self.path) - 1:
                x3 = self.path[0][0]
                y3 = self.path[0][1]
                gammap3 = self.gammap[0]

            else:
                x3 = self.path[i + 1][0]
                y3 = self.path[i + 1][1]
                gammap3 = self.gammap[i + 1]

            self.gammapp[i] = (gammap3 - self.gammap[i - 1]) / (
                    math.sqrt((x3 - x2) ** 2 + (y3 - y2) ** 2) + math.sqrt(
                        (x2 - x1) ** 2 + (y2 - y1) ** 2))

    def get_distance(self, xy1, xy2):
        """Returns the distance between the two points xy1 and xy2. xy1 is
        considered to be in front of xy2. The distance is calculated from
        xy1's projection on the path back along the path to xy2. """
        index1, _ = self.get_closest(xy1)
        index2, _ = self.get_closest(xy2)

        return self.get_distance_from_indices(index1, index2)

    def get_distance_from_indices(self, i1, i2):
        """Returns the distance along the path between the two indices. The
        distance is calculate from i1 back along the path to i2. """
        dist_sum = 0

        i = i1
        i_lower = i2 if i2 < i1 else i2 - len(self.path)

        while i >= i_lower:
            xy_l = self.get_xy(i - 1)
            xy_u = self.get_xy(i)
            dist_sum += math.sqrt(
                (xy_u[0] - xy_l[0]) ** 2 + (xy_u[1] - xy_l[1]) ** 2)

            i -= 1

        return dist_sum

    def get_position_on_path(self, xy):
        """Returns the position on the path calculated from the beginning of the
        path."""
        index, _ = self.get_closest(xy)

        position = self.get_distance_from_indices(index, 0)

        return position

    def get_path_length(self):
        """Returns the length of the path. Calculated from the distance between
        the last and the first indices. """
        return self.get_distance_from_indices(len(self.path) - 1, 0)


    def plot(self, realh=5, realw=5):
        """Plots the path in a Tkinter window. Arguments are the width and
        height of the real path area in meters."""
        self._lp = True  # Plot while true
        realh = float(realh)
        realw = float(realw)
        root = Tk.Tk()
        h = 600  # Tkinter canvas height.
        w = int(h * realw / realh)

        s_frame = Tk.Frame(root, background='aquamarine')
        s_frame.pack()

        canv_frame = Tk.Frame(root)
        canv_frame.pack(in_=s_frame, side=Tk.LEFT)

        # Canvas for drawing the path in.
        canv = Tk.Canvas(root, width=w, height=h, background='#FFFFFF',
                         borderwidth=0, relief=Tk.RAISED)
        canv.pack(in_=canv_frame)
        lst = [h, w, realh, realw]
        canv.bind('<Button-1>',
                  lambda event, arg=lst: self._print_click_info(event, arg))

        # Frame containing quit button.
        right_frame = Tk.Frame(root, background='aquamarine')
        right_frame.pack(in_=s_frame, side=Tk.RIGHT, anchor=Tk.N)

        # Quit button for closing the GUI.
        quit_button = Tk.Button(root, text='Close', command=self._quitm,
                                width=10, background='coral',
                                activebackground='red')
        quit_button.pack(in_=right_frame)

        # Create origin coordinate arrows.
        canv.create_line(int(w / 2), int(h / 2), int(w / 2), int(h / 2) - 50,
                         width=2, arrow='last')
        canv.create_line(int(w / 2), int(h / 2), int(w / 2) + 50, int(h / 2),
                         width=2, arrow='last')

        # Add coordinates to the corners.
        canv.create_text(2, 2, text='({}, {})'.format(-realw / 2, realh / 2),
                         anchor='nw')
        canv.create_text(2, h - 2,
                         text='({}, {})'.format(-realw / 2, -realh / 2),
                         anchor='sw')
        canv.create_text(w - 2, h - 2, text='({}, {})'.format(realw / 2,
                                                              -realh / 2),
                         anchor='se')
        canv.create_text(w - 2, 2, text='({}, {})'.format(realw / 2, realh / 2),
                         anchor='ne')

        root.protocol("WM_DELETE_WINDOW", self._quitm)  # Window close action.

        try:
            # Print arrow in the direction of the path at the start of the path.
            length = 100.0  # Length of arrow.
            d = 20.0  # Distance from path do draw arrow at.
            tang = self.get_tangent(0)
            norm = self.get_normal(0)
            xy0 = self._pixelv(0, realh, realw, h, w)
            xy0[0] = int(xy0[0] + norm[0] * d - tang[0] * length / 2)
            xy0[1] = int(xy0[1] - norm[1] * d + tang[1] * length / 2)
            xyf = [int(xy0[0] + tang[0] * length),
                   int(xy0[1] - tang[1] * length)]
            canv.create_line(xy0[0], xy0[1], xyf[0], xyf[1],
                             width=2, arrow='last')

            # Print lines between points and mark each point with a dot.
            for i in range(len(self.path)):
                xy1 = self._pixelv(i - 1, realh, realw, h, w)
                xy2 = self._pixelv(i, realh, realw, h, w)
                canv.create_line(xy1[0], xy1[1], xy2[0], xy2[1],
                                 fill='blue', width=2)
                canv.create_oval(xy2[0] - 3, xy2[1] - 3,
                                 xy2[0] + 3, xy2[1] + 3, fill='green')
        except Exception as e:
            print(e)

        while self._lp:
            try:
                if 'normal' == root.state():
                    root.update()
            except:
                pass

        root.destroy()

    def _print_click_info(self, event, arg):
        """Used internally to print the coordinates clicked on."""
        x = float((event.x - arg[1] / 2) * arg[3] / arg[1])
        y = float((arg[0] / 2 - event.y) * arg[2] / arg[0])
        index, closest = self.get_closest([x, y])
        print('({:07.4f}, {:07.4f}), {}error: {:07.4f}, gamma: {:05.4f}'.format(
            x, y,
            'left,  ' if self.is_left([x, y]) else 'right, ',
            self.get_ey([x, y]), self.get_gamma(index)))

    def _pixelv(self, index, hreal, wreal, hpixel, wpixel):
        """Used internally to transform the path from real coordinates
        to pixel coordinates."""
        try:
            index = index % len(self.path)
            xpixel = int(wpixel / wreal * self.path[index][0] + wpixel / 2)
            ypixel = int(- hpixel / hreal * self.path[index][1] + hpixel / 2)
            return [xpixel, ypixel]
        except:
            return [0, 0]

    def _quitm(self):
        """Used internally for quitting graphical application."""
        self._lp = False


class NewPath:
    """Class for defining a custom path. """

    def __init__(self, filename, width=6, height=6):
        self.filename = filename

        self.root = Tk.Tk()

        self.newpath = []
        self.graphicpath = []
        self.height = float(height)  # Height of area in meters.
        self.width = float(width)
        self.win_height = 800  # Graphical window height
        self.win_width = int(self.win_height * self.width / self.height)

        s_frame = Tk.Frame(self.root, background='aquamarine')
        s_frame.pack()

        canv_frame = Tk.Frame(self.root)
        canv_frame.pack(in_=s_frame, side=Tk.LEFT)

        self.canv = Tk.Canvas(self.root, width=self.win_width,
                              height=self.win_height, background='#FFFFFF',
                              borderwidth=0, relief=Tk.RAISED)
        self.canv.pack(in_=canv_frame)
        self.canv.bind('<Button-1>', self.append_new_path)

        right_frame = Tk.Frame(self.root, background='aquamarine')
        right_frame.pack(in_=s_frame, side=Tk.RIGHT, anchor=Tk.N)

        quit_button = Tk.Button(self.root, text='Quit and \nsave path',
                                command=self.quit_save,
                                width=10, height=2, background='green3',
                                activebackground='green4')
        quit_button.pack(in_=right_frame)

        quit_button2 = Tk.Button(self.root, text='Quit without \nsaving path',
                                 command=self.quit_no_save,
                                 width=10, height=2, background='red3',
                                 activebackground='red4')
        quit_button2.pack(in_=right_frame)

        self.canv.create_line(int(self.win_width / 2), int(self.win_height / 2),
                              int(self.win_width / 2),
                              int(self.win_height / 2) - 50, width=2,
                              arrow='last')
        self.canv.create_line(int(self.win_width / 2), int(self.win_height / 2),
                              int(self.win_width / 2) + 50,
                              int(self.win_height / 2), width=2,
                              arrow='last')

        self.canv.create_text(2, 2,
                              text='({}, {})'.format(
                                  -self.width / 2, self.height / 2),
                              anchor='nw')
        self.canv.create_text(2, self.win_height - 2,
                              text='({}, {})'.format(-self.width / 2,
                                                     -self.height / 2),
                              anchor='sw')
        self.canv.create_text(self.win_width - 2, self.win_height - 2,
                              text='({}, {})'.format(self.width / 2,
                                                     -self.height / 2),
                              anchor='se')
        self.canv.create_text(self.win_width - 2, 2,
                              text='({}, {})'.format(self.width / 2,
                                                     self.height / 2),
                              anchor='ne')

        self.root.title('Record path')
        self.root.mainloop()

    def quit_save(self):
        """Save and quit the application. """
        self.save(self.filename)
        self.root.quit()

    def quit_no_save(self):
        """Quit without saving. """
        self.root.quit()

    def save(self, filename):
        """Saves path to file. Writes each line on the format x,y"""
        try:
            __location__ = os.path.realpath(os.path.join(os.getcwd(),
                                                         os.path.dirname(
                                                             __file__)))
            fl = open(os.path.join(__location__, self.filename), 'w')

            for xy in self.newpath:
                fl.write('{},{}\n'.format(xy[0], xy[1]))

            fl.close()
            print('Saved path as {}'.format(self.filename))

        except Exception as e:
            print('\nError when saving path to file: '),
            print(e)

    def append_new_path(self, event):
        """Appends the coordinates of a mouse click to the path. """
        if (0 <= event.x <= self.win_width) and (
                0 <= event.y <= self.win_height):
            xy = self.transform([event.x, event.y])

            print('{:07.4f}, {:07.4f}'.format(xy[0], xy[1]))

            self.draw(event.x, event.y)
            self.newpath.append(xy)
            self.graphicpath.append([event.x, event.y])

    def draw(self, x, y):
        """Draws a line from the previous point to the most recent. Mark with
        circle. """
        try:
            self.canv.create_line(self.graphicpath[-1][0],
                                  self.graphicpath[-1][1],
                                  x, y, fill='blue', width=2)
        except:
            pass
        self.canv.create_oval(x - 3, y - 3, x + 3, y + 3, fill='green')

    def transform(self, xy):
        """Transform pixel coordinates to real coordinates. """
        x = float((xy[0] - self.win_width / 2) * self.width / self.win_width)
        y = float((self.win_height / 2 - xy[1]) * self.height / self.win_height)
        return [x, y]

    def printp(self):
        """Prints the path in the terminal."""
        for xy in self.newpath:
            print('{:07.4f}, {:07.4f}'.format(xy[0], xy[1]))
