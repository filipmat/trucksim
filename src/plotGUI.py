#!/usr/bin/env python

# Class for GUI that plots the truck trajectories. Subscribes to the topic where
# the truck positions are published.

# TODO
# Add/fix recording of data.

import rospy
import time
import Tkinter as Tk
import math
import os
import random

from trucksim.msg import vehicleposition
import path


class TruckPlot(object):
    """Class for GUI that plots the truck trajectories. """

    # noinspection PyPep8
    def __init__(self, root, node_name, topic_type, topic_name,
                 filename='record', width=5, height=5, win_size=600,
                 clear_seconds=180, inactivity_time_limit=20):
        """
        Parameters
        filename: string, prefix for saved files.
        width, height: floats, width and height of the plot area in meters.
        win_size: integer, height of the GUI in pixels.
        clear_seconds: integer, how often the trajectories should be cleared
            automatically. Set to 0 to never clear trajectories automatically.
        inactivity_time_limit: integer, how long a vehicle can stay inactive
            before it is removed. Set to 0 to never remove inactive vehicles.
        """
        self.root = root
        self.width = float(width)  # Real width (meters) of the window.
        self.height = float(height)
        self.win_height = win_size  # Graphical window height.
        self.win_width = int(self.win_height * self.width / self.height)
        self.filename_prefix = filename
        self.display_tail = False
        self.display_path = False
        self.node_name = node_name  # Subscriber node name.
        self.topic_name = topic_name  # Subscriber topic name.
        self.topic_type = topic_type  # Subscriber topic type.

        # Reference path.
        self.pt = path.Path()
        self.PATH_TAG = 'path'
        self.PATH_COLOR = 'blue'
        self.xr = 0  # Reference path ellipse radii.
        self.yr = 0
        self.xc = 0  # Reference path ellipse center.
        self.yc = 0

        # Parameters for recording data.
        self.recording = False
        self.timestamp = 0
        self.rec_start_time = 0

        # Parameters for periodically clearing trajectories.
        self.clear_seconds = clear_seconds
        if self.clear_seconds == 0:
            self.clear_periodically = False
        else:
            self.clear_periodically = True

        # Parameters for periodiclly checking for inactive vehicles.
        self.inactivity_time_limit = inactivity_time_limit
        if self.inactivity_time_limit == 0:
            self.remove_inactive_vehicles = False
        else:
            self.remove_inactive_vehicles = True
        self.inactivity_check_delay = 2000  # How often method calls itself, ms.

        # Dicts for keeping track of vehicle information.
        self.vehicles = dict()
        self.vehicle_colors = dict()
        self.old_positions = dict()
        self.last_published_time = dict()

        # Stuff for canvas.
        bg_color = 'SlateGray2'
        w1 = 15
        ypad = 10
        self.truckl = 0.27  # Real vehicle length.
        self.truckw = 0.15

        # Setup subscriber node.
        rospy.init_node(self.node_name, anonymous=True)
        rospy.Subscriber(self.topic_name, self.topic_type, self._callback)

        # Base frame.
        s_frame = Tk.Frame(self.root, background=bg_color)
        s_frame.pack()

        # Create canvas frame with a canvas for drawing in.
        canv_frame = Tk.Frame(self.root)
        canv_frame.pack(in_=s_frame, side=Tk.LEFT)
        self.canv = Tk.Canvas(self.root, width=self.win_width,
                              height=self.win_height, background='#FFFFFF',
                              borderwidth=0, relief=Tk.RAISED)
        self.canv.pack(in_=canv_frame)
        self.canv.bind('<Button-1>', self._left_click)

        # Available colors to be used by the vehicles.
        self.available_colors = ['blue', 'green', 'yellow', 'orange', 'red',
                                 'pink', 'purple', 'cyan', 'dark green',
                                 'coral', 'purple4',
                                 'brown1']
        # Currently colors that are free to be used.
        self.free_colors = self.available_colors[:]

        # Create frame next to the canvas for buttons, labels etc.
        right_frame = Tk.Frame(self.root, background=bg_color)
        right_frame.pack(in_=s_frame, side=Tk.RIGHT, anchor=Tk.N)

        # Create button frame.
        button_frame = Tk.Frame(self.root, background=bg_color)
        button_frame.pack(in_=right_frame, side=Tk.TOP, anchor=Tk.N,
                          pady=(0, 2 * ypad))

        traj_frame = Tk.Frame(self.root, background=bg_color)
        traj_frame.pack(in_=right_frame, side=Tk.TOP, anchor=Tk.N,
                        pady=(0, 2 * ypad))

        # Create frame for recording widgets.
        record_frame = Tk.Frame(self.root, background=bg_color)
        # record_frame.pack(in_ = right_frame, side = tk.TOP,
        #                    anchor = tk.N, pady = (0, 2*ypad))

        path_frame = Tk.Frame(self.root, background=bg_color)
        path_frame.pack(in_=right_frame, side=Tk.TOP, pady=(0, 2 * ypad))

        # Create bottom frame for other stuff.
        bottom_frame = Tk.Frame(self.root, background=bg_color)
        bottom_frame.pack(in_=right_frame, side=Tk.TOP, anchor=Tk.N,
                          pady=(2 * ypad, 0))

        # Button for quitting the program.
        quit_button = Tk.Button(self.root, text='Quit',
                                command=self._quit1,
                                width=w1, height=2, background='red2',
                                activebackground='red3')
        quit_button.pack(in_=button_frame)

        # Checkbox for displaying trajectories.
        self.traj_button_var = Tk.IntVar()
        self.traj_button = Tk.Checkbutton(self.root,
                                          text='Display\ntrajectories',
                                          variable=self.traj_button_var,
                                          command=self._traj_btn_callback,
                                          width=w1, height=2,
                                          background=bg_color)
        if self.display_tail:
            self.traj_button.toggle()
        self.traj_button.pack(in_=traj_frame, side=Tk.TOP)

        # Button for clearing trajectories.
        self.clear_button = Tk.Button(self.root, text='Clear trajectories',
                                      command=self._clear_trajectories,
                                      width=w1, height=2, background='orange',
                                      activebackground='dark orange')
        self.clear_button.pack(in_=traj_frame, side=Tk.TOP)

        # Buttons for recording trajectories.
        self.start_record_button = Tk.Button(
            self.root, text='Start new\nrecording',
            command=self._start_record, width=w1, height=2)
        self.start_record_button.pack(in_=record_frame)
        self.stop_record_button = Tk.Button(self.root, text='Stop\nrecording',
                                            command=self._stop_record, width=w1,
                                            height=2,
                                            state=Tk.DISABLED)
        self.stop_record_button.pack(in_=record_frame)

        # Label displaying elapsed recording time.
        self.rec_time_text_var = Tk.StringVar()
        self.rec_time_label = Tk.Label(self.root,
                                       textvariable=self.rec_time_text_var,
                                       anchor=Tk.W,
                                       justify=Tk.LEFT, width=13, height=2,
                                       background=bg_color,
                                       foreground='grey')
        self.rec_time_text_var.set('Not recording\n')
        self.rec_time_label.pack(in_=record_frame)

        # Widgets for changing reference path.
        self.path_label = Tk.Label(self.root, text='REFERENCE PATH',
                                   background=bg_color)
        self.path_label.pack(in_=path_frame, side=Tk.TOP)

        # Checkbox for displaying trajectories.
        self.path_button_var = Tk.IntVar()
        self.path_button = Tk.Checkbutton(self.root,
                                          text='Display\nreference path',
                                          variable=self.path_button_var,
                                          command=self._path_btn_callback,
                                          width=w1, height=2,
                                          background=bg_color)
        if self.display_path:
            self.path_button.toggle()
        self.path_button.pack(in_=path_frame, side=Tk.TOP)

        # Variables for changing the reference path.
        self.xr_var = Tk.StringVar()
        self.xr_var.set(0)
        self._make_entry(path_frame, bg_color, 'x_radius',
                         textvariable=self.xr_var)

        self.yr_var = Tk.StringVar()
        self.yr_var.set(0)
        self._make_entry(path_frame, bg_color, 'y_radius',
                         textvariable=self.yr_var)

        self.xc_var = Tk.StringVar()
        self.xc_var.set(0)
        self._make_entry(path_frame, bg_color, 'x_offset',
                         textvariable=self.xc_var)

        self.yc_var = Tk.StringVar()
        self.yc_var.set(0)
        self._make_entry(path_frame, bg_color, 'y_offset',
                         textvariable=self.yc_var)

        self.apply_path_button = Tk.Button(self.root,
                                           text='Apply new\nreference path',
                                           command=self._apply_path,
                                           width=w1, height=2,
                                           background='PaleGreen3',
                                           activebackground='PaleGreen4')
        self.apply_path_button.pack(in_=path_frame, side=Tk.TOP)

        # Label displaying the elapsed server time.
        self.time_text_var = Tk.StringVar()
        self.time_label = Tk.Label(self.root, textvariable=self.time_text_var,
                                   anchor=Tk.W, justify=Tk.LEFT,
                                   width=13, height=2,
                                   background=bg_color)
        self.time_text_var.set('')
        self.time_label.pack(in_=bottom_frame)

        # Actions for closing the window and pressing ctrl-C on the window.
        self.root.protocol('WM_DELETE_WINDOW', self._quit1)
        self.root.bind('<Control-c>', self._quit2)

        # Draw the coordinate arrows and coordinate labels in corners.
        self._draw_coordinate_frame()

        # Start repeated clearing of trajectories.
        self._clear_trajectories_periodically()

        # Start repeated removal of inactive trucks.
        self._remove_inactive_vehicles()

    def _make_entry(self, framep, background, caption, **options):
        """Creates an entry widget. """
        frame = Tk.Frame(self.root, background=background)
        frame.pack(in_=framep, side=Tk.TOP)
        lbl = Tk.Label(self.root, text=caption, background=background,
                       width=8, anchor=Tk.W)
        lbl.pack(in_=frame, side=Tk.LEFT)
        entry = Tk.Entry(self.root, width=9, **options)
        entry.pack(in_=frame, side=Tk.LEFT)

    def _apply_path(self):
        """Apply changes made to the reference path in the entry widgets. """
        try:
            xr = float(self.xr_var.get())
            yr = float(self.yr_var.get())
            xc = float(self.xc_var.get())
            yc = float(self.yc_var.get())
            if xr <= 0 or yr <= 0:
                print('Invalid values entered')
            else:
                self.gen_circle_path([xr, yr], 400, [xc, yc])
                print('New reference path applied.')
        except Exception as e:
            print('Invalid values entered.')

        self.xr_var.set(self.xr)
        self.yr_var.set(self.yr)
        self.xc_var.set(self.xc)
        self.yc_var.set(self.yc)

        self.apply_path_button.focus()

    def _left_click(self, event):
        """Action when left clicking on the canvas. """
        xreal, yreal = self._pixel_to_real(event.x, event.y)
        print('Clicked at ({:07.4f}, {:07.4f})'.format(xreal, yreal))

    def _quit1(self):
        """Quits the GUI. """
        print('Quitting.')
        self.root.quit()

    def _quit2(self, event):
        """Quits the GUI. """
        print('Quitting.')
        self.root.quit()

    def _callback(self, data):
        """Subscriber callback method. Called when receiving data on the topic.
        Moves the vehicle corresponding to the id in the data or creates a new
        vehicle if it does not exist. """
        vehicle_id = data.id
        x = data.x
        y = data.y
        theta = data.theta

        try:
            self._move_truck(vehicle_id, x, y, theta)
            self._draw_tail(vehicle_id, x, y, theta)
        except KeyError:
            self._create_new_truck(vehicle_id, x, y, theta)

        # Store last time data was published in order to check inactivity.
        self.last_published_time[vehicle_id] = time.time()

    def _move_truck(self, vehicle_id, x, y, theta):
        """Moves a truck triangle to the new position. """
        xf, yf, xr, yr, xl, yl = self._get_triangle_corners(x, y, theta)

        # Move polygon.
        self.canv.coords(self.vehicles[vehicle_id], xf, yf, xr, yr, xl, yl)

    def _create_new_truck(self, vehicle_id, x, y, theta):
        """Creates a new vehicle triangle on the canvas. Adds the vehicle to
        the dictionaries. """
        print('Vehicle {} added.'.format(vehicle_id))

        xf, yf, xr, yr, xl, yl = self._get_triangle_corners(x, y, theta)
        color = self._get_random_color()

        # Create triangle.
        self.vehicles[vehicle_id] = self.canv.create_polygon(
            xf, yf, xr, yr, xl, yl, fill=color, outline='black')

        self.vehicle_colors[vehicle_id] = color  # Store vehicle color.

        self.old_positions[vehicle_id] = [x, y]  # Store previous position.

    def _get_random_color(self):
        """Returns a random color from the list of available colors. """
        sr = random.SystemRandom()
        color = sr.choice(self.free_colors)
        index = self.free_colors.index(color)

        self.free_colors = [
            c for i, c in enumerate(self.free_colors) if i != index]

        if len(self.free_colors) == 0:
            self.free_colors = self.available_colors[:]

        return color

    def _draw_tail(self, vehicle_id, x, y, theta):
        """Draws the last movement of the vehicle. """
        if self.display_tail:
            state = Tk.NORMAL
        else:
            state = Tk.HIDDEN

        # Draw a line from the old position to the new.
        self._plot_sequence(
            [[self.old_positions[vehicle_id][0],
              self.old_positions[vehicle_id][1]],
             [x, y]],
            join=False, clr=self.vehicle_colors[vehicle_id],
            tag=vehicle_id, state=state)

        # Update old vehicle position.
        self.old_positions[vehicle_id] = [x, y]

    def _remove_inactive_vehicles(self):
        """Removes vehicles that have not published a position for a certain
        amount of time. Method calls itself periodically. """
        inactive_ids = []

        if self.remove_inactive_vehicles:
            # Find which vehicles are inactive.
            for vehicle_id in self.last_published_time:
                if time.time() - self.last_published_time[vehicle_id] > \
                        self.inactivity_time_limit:
                    inactive_ids.append(vehicle_id)

            # Remove all inactive vehicles.
            for vehicle_id in inactive_ids:
                self._remove_vehicle(vehicle_id)

        # Call method again after a delay.
        self.root.after(
            self.inactivity_check_delay, self._remove_inactive_vehicles)

    def _remove_vehicle(self, vehicle_id):
        """Removes the vehicle. """
        print('Vehicle {} removed because of inactivity.'.format(vehicle_id))

        self._remove_vehicle_drawings(vehicle_id)
        self._remove_vehicle_information(vehicle_id)

    def _remove_vehicle_drawings(self, vehicle_id):
        """Removes all traces of the vehicle from the canvas. """
        self.canv.delete(self.vehicles[vehicle_id])
        self.canv.delete(vehicle_id)

    def _remove_vehicle_information(self, vehicle_id):
        """Removes all stored information about the vehicle. """
        del self.vehicles[vehicle_id]
        del self.vehicle_colors[vehicle_id]
        del self.old_positions[vehicle_id]
        del self.last_published_time[vehicle_id]

    def _raise_vehicles(self):
        """Raises all vehicle objects to the top of the canvas. """
        for vehicle_id in self.vehicles:
            self.canv.tag_raise(self.vehicles[vehicle_id])

    def _draw_coordinate_frame(self):
        """Draw lines for the origin and create text displaying the coordinates
        in the corners. """
        cftag = 'cf'
        # Create origin coordinate arrows.
        self.canv.create_line(int(self.win_width / 2), int(self.win_height / 2),
                              int(self.win_width / 2),
                              int(self.win_height / 2) - 50,
                              width=2, arrow='last', tag=cftag)
        self.canv.create_line(int(self.win_width / 2), int(self.win_height / 2),
                              int(self.win_width / 2) + 50,
                              int(self.win_height / 2),
                              width=2, arrow='last', tag=cftag)

        # Add coordinates to the corners.
        d = 6
        self.canv.create_text(
            d, d,
            text='({:.1f}, {:.1f})'.format(
                -self.width / 2, self.height / 2),
            anchor='nw', tag=cftag)

        self.canv.create_text(
            d, self.win_height - d,
            text='({:.1f}, {:.1f})'.format(
                -self.width / 2, -self.height / 2),
            anchor='sw', tag=cftag)

        self.canv.create_text(
            self.win_width - d, self.win_height - d,
            text='({:.1f}, {:.1f})'.format(
                self.width / 2, -self.height / 2),
            anchor='se', tag=cftag)

        self.canv.create_text(
            self.win_width - d, d,
            text='({:.1f}, {:.1f})'.format(
                self.width / 2, self.height / 2),
            anchor='ne', tag=cftag)

    def _draw_closest_point(self, xy, clr='blue'):
        """Draw the closest point on the path for coordinates xy. """
        cross_length = 10
        _, closest = self.pt.get_closest(xy)
        xp, yp = self._real_to_pixel(closest[0], closest[1])

        self.canv.create_line(xp - cross_length, yp - cross_length,
                              xp + cross_length, yp + cross_length,
                              fill=clr, tag='closest', width=2)
        self.canv.create_line(xp - cross_length, yp + cross_length,
                              xp + cross_length, yp - cross_length,
                              fill=clr, tag='closest', width=2)

    def _plot_sequence(self, seq, join=False, clr='blue', width=2,
                       tag='line', state=Tk.NORMAL):
        """Plots a sequence, a list on the form [[x0, y0], [x1, y1], ...],
        where x1 and y1 are real coordinates. Joins the beginning and end
        if join = True. """
        if len(seq) > 0:
            if join:  # Join the first and the last points.
                starti = 0
            else:
                starti = 1
            try:
                for i in range(starti, len(seq)):
                    x1, y1 = self._real_to_pixel(seq[i - 1][0], seq[i - 1][1])
                    x2, y2 = self._real_to_pixel(seq[i][0], seq[i][1])
                    self.canv.create_line(x1, y1, x2, y2,
                                          fill=clr, width=width, tag=tag,
                                          state=state)
            except Exception as e:
                print('Error when plotting sequence: {}'.format(e))

    def _get_triangle_corners(self, xreal, yreal, yaw):
        """Returns the three coordinate pairs in pixel for the triangle
        corresponding to the vehicle position. """
        length = self.truckl  # Truck length in meters.
        width = self.truckw

        # Coordinates for frontal corner.
        xf, yf = self._real_to_pixel(
            xreal + length / 2 * math.cos(yaw),
            yreal + length / 2 * math.sin(yaw))
        # Coordinates for rear right corner.
        xr, yr = self._real_to_pixel(
            xreal - length / 2 * math.cos(yaw) + width / 2 * math.cos(
                yaw - math.pi / 2),
            yreal - length / 2 * math.sin(yaw) + width / 2 * math.sin(
                yaw - math.pi / 2))

        # Coordinates for rear left corner.
        xl, yl = self._real_to_pixel(
            xreal - length / 2 * math.cos(yaw) + width / 2 * math.cos(
                yaw + math.pi / 2),
            yreal - length / 2 * math.sin(yaw) + width / 2 * math.sin(
                yaw + math.pi / 2))

        return xf, yf, xr, yr, xl, yl

    def _traj_btn_callback(self):
        """Callback for trajectory check button. Enable/disaple plotting of
        trajectories. """
        if self.traj_button_var.get() == 1:
            self.display_tail = True
            self._show_trajectories()
        else:
            self.display_tail = False
            self._hide_trajectories()

    def _hide_trajectories(self):
        """Hides all vehicle trajectories. """
        for vehicle_id in self.vehicles:
            self._hide_canvas_tag(vehicle_id)

    def _show_trajectories(self):
        """Shows all vehicle trajectories. """
        for vehicle_id in self.vehicles:
            self._show_canvas_tag(vehicle_id)

    def _path_btn_callback(self):
        """Callback for path check button. Enable/disaple plotting of
        reference path. """
        if self.path_button_var.get() == 1:
            self.display_path = True
            self._draw_path()
            self.clear_button.config(state='normal')
            self._raise_vehicles()
        else:
            self.display_path = False
            self.canv.delete(self.PATH_TAG)

    def _record_data(self):
        """Writes data to file if recording is on. Sets recording label. """
        if self.recording:
            # Gather data to be written into a list.
            values = []

            self._write_data(values)  # Write list to file.

            # Set the label displaying the recording time.
            self.rec_time_text_var.set(
                'Recording: \n{:.1f}'.format(
                    time.time() - self.rec_start_time))

    def _start_record(self):
        """Starts a new recording of trajectories. Saves files to a file named
        'self.filename_prefix + i + .txt', where i is the first available
        index for which such a file not already exists. """
        if self.recording:
            print('Already recording.')
            return

        self._clear_trajectories()
        self.recording = True
        self.start_record_button.config(state='disabled')
        self.stop_record_button.config(state='normal')
        self.rec_start_time = time.time()
        self.rec_time_label.config(foreground='black')

        try:
            __location__ = os.path.realpath(os.path.join(os.getcwd(),
                                                         os.path.dirname(
                                                             __file__)))

            i = 0
            while os.path.exists(os.path.join(__location__, '{}{}{}'.format(
                    self.filename_prefix, i, '.txt'))):
                i += 1

            self.filename = self.filename_prefix + str(i) + '.txt'

            self.record_file = open(
                os.path.join(__location__, self.filename), 'w')

        except Exception as e:
            print('\nError when opening file for writing: {}'.format(e))
            return

        print('Recording started.')

    def _stop_record(self):
        """Stops current recording of trajectories. """
        if not self.recording:
            print('No recording is running.')
            return

        self.recording = False
        self.start_record_button.config(state='normal')
        self.stop_record_button.config(state='disabled')
        self.rec_time_text_var.set('Not recording\n')
        self.rec_time_label.config(foreground='grey')

        try:
            self.record_file.close()
            print('Saved as {}'.format(self.filename))

        except Exception as e:
            print('\nError when closing file: {}'.format(e))

        print('Recording stopped.')

    def _write_data(self, values):
        """Writes values to file. values is a list. """
        if self.recording:
            try:
                for i, x in enumerate(values):
                    if i == 0:
                        self.record_file.write('{}'.format(x))
                    else:
                        self.record_file.write(',{}'.format(x))

                self.record_file.write('\n')

            except Exception as e:
                print('\nError when writing to file: {}'.format(e))

    def _real_to_pixel(self, xreal, yreal):
        """Transform from real to pixel coordinates. """
        xpixel = int(self.win_width / self.width * xreal + self.win_width / 2)
        ypixel = int(
            -self.win_height / self.height * yreal + self.win_height / 2)
        return xpixel, ypixel

    def _pixel_to_real(self, xp, yp):
        """Transform from pixel to real coordinates. """
        xreal = float((xp - self.win_width / 2) * self.width / self.win_width)
        yreal = float(
            (self.win_height / 2 - yp) * self.height / self.win_height)
        return xreal, yreal

    @staticmethod
    def _tailn(seq, n):
        """Returns the last n of a list. If n < 0 return whole list. """
        tail_len = len(seq)
        if n < 0 or n > tail_len:
            return seq
        elif n == 0:
            return []
        else:
            return seq[tail_len - n:tail_len]

    def _hide_canvas_tag(self, tag):
        """Hide all canvas items with the specified tag. """
        handles = self.canv.find_withtag(tag)
        for x in handles:
            self.canv.itemconfig(x, state=Tk.HIDDEN)

    def _show_canvas_tag(self, tag):
        """Show all canvas items with the specified tag. """
        handles = self.canv.find_withtag(tag)
        for x in handles:
            self.canv.itemconfig(x, state=Tk.NORMAL)

    def _clear_trajectories_periodically(self):
        """Clears all vehicle trajectories. Method calls itself after a certain
        time delay. """
        if self.clear_periodically:
            self._clear_trajectories()
        self.root.after(
            self.clear_seconds * 1000, self._clear_trajectories_periodically)

    def _clear_trajectories(self):
        """Clear the saved truck trajectories. """
        print('Trajectories cleared. ')
        for vehicle_id in self.vehicles:
            self.canv.delete(vehicle_id)

    def _draw_path(self):
        """Draws the reference path. """
        if self.display_path:
            self._plot_sequence(self.pt.path, join=True, tag=self.PATH_TAG,
                                clr=self.PATH_COLOR, width=2)

    def load_path(self, filename):
        """Loads a path from a file. """
        self.pt.load(filename)
        self._draw_path()

    def gen_circle_path(self, radius, points, center=None):
        """Generates a circle/ellipse path. """
        if center is None:
            center = [0, 0]

        if isinstance(radius, list):
            if len(radius) > 1:
                self.xr = radius[0]
                self.yr = radius[1]
            else:
                self.xr = radius[0]
                self.yr = radius[0]
        else:
            self.xr = radius
            self.yr = radius

        self.xc = center[0]
        self.yc = center[1]

        self.xr_var.set(self.xr)
        self.yr_var.set(self.yr)
        self.xc_var.set(self.xc)
        self.yc_var.set(self.yc)

        self.pt.gen_circle_path([self.xr, self.yr], points, [self.xc, self.yc])
        self.canv.delete(self.PATH_TAG)
        self._draw_path()


def main():
    node_name = 'plot'  # Name of node.
    topic_name = 'vehicle_position'  # Name of topic the node subscribes to.
    topic_type = vehicleposition  # The type of the topic.

    width = 6  # Width in meters of displayed area.
    height = 6  # Height in meters.
    x_radius = 1.7  # Ellipse x-radius.
    y_radius = 1.2  # Ellipse y-radius.
    center = [0.3, -1.3]  # The coordinates of the center of the ellipse.
    pts = 200  # Number of points on displayed reference path.

    root = Tk.Tk()
    try:
        truckplot = TruckPlot(root, node_name, topic_type, topic_name,
                              width=width, height=height)

        truckplot.gen_circle_path([x_radius, y_radius], pts, center=center)

        root.mainloop()

    except RuntimeError as e:
        print('Error when running GUI: {}'.format(e))


if __name__ == '__main__':
    main()
