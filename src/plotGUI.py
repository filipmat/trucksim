#!/usr/bin/env python

"""
Usage: rosrun trucksim plotGUI.py

Class for a GUI that plots the truck trajectories. Subscribes to the topic where
the truck positions are published.

Assumes that the message contains the fields: ID, x, y, yaw. If the fields are named differently
this needs to be changed in _callback() so that the methods called in _callback() receives the
correct parameters.
"""

import rospy
import time
import Tkinter as Tk
import math
import random

from trucksim.msg import MocapState  # ROS topic for receiving data.
import path     # For displaying a reference path.


class TruckPlot(object):
    """Class for GUI that plots the truck trajectories. """

    def __init__(self, root, topic_type, topic_name,
                 width=5, height=5, win_size=600,
                 clear_seconds=180, inactivity_time_limit=20, vehicle_length=0.4):
        """

        :param root: Tkinter root.
        :param topic_type: type of the ROS topic where vehicle positions are published.
        :param topic_name: name of the ROS topic where vehicle positions are published.
        :param width: width of the initial drawing area in meters.
        :param height: height of the initial drawing area in meters.
        :param win_size: width of the drawing area in pixels.
        :param clear_seconds: how often to periodically clear the past vehicle trajectories. Set to
        0 to never clear.
        :param inactivity_time_limit: how long a vehicle will be retained without receiving new
        position information. Set to 0 to retain forever.
        :param vehicle_length: length of the triangle representing vehicles.
        """

        self.root = root
        self.width = float(width)  # Real width (meters) of the window.
        self.height = float(height)
        self.window_height = win_size  # Graphical window height.
        self.window_width = int(self.window_height * self.width / self.height)
        self.display_vehicle_tails = False
        self.display_reference_path = False

        # Reference path.
        self.pt = path.Path()
        self.PATH_TAG = 'path'
        self.PATH_COLOR = 'blue'
        self.x_radius = 0  # Reference path ellipse radii.
        self.y_radius = 0
        self.x_center = 0  # Reference path ellipse center.
        self.y_center = 0

        # Parameters for periodically clearing trajectories.
        self.clear_seconds = clear_seconds
        if self.clear_seconds == 0:
            self.periodically_clear_trajectories = False
        else:
            self.periodically_clear_trajectories = True

        # Parameters for periodiclly checking for inactive vehicles.
        self.inactivity_time_limit = inactivity_time_limit
        if self.inactivity_time_limit == 0:
            self.remove_inactive_vehicles = False
        else:
            self.remove_inactive_vehicles = True
        self.inactivity_check_delay = 2000  # How often method calls itself, ms.

        # Dicts for keeping track of vehicle information.
        self.vehicle_polygons = dict()
        self.vehicle_colors = dict()
        self.previous_vehicle_positions = dict()
        self.last_vehicle_published_time = dict()

        # Stuff for canvas.
        bg_color = 'SlateGray2'
        w1 = 15
        ypad = 10
        self.vehicle_length = vehicle_length  # Real vehicle length.
        self.vehicle_width = vehicle_length / 2
        self.coordinate_frame_tag = 'cf'
        self.zoom_scale = 1.25
        self.truck_zoom_scale = 1.25

        # Setup subscriber node.
        rospy.init_node('plotGUI', anonymous=True)
        rospy.Subscriber(topic_name, topic_type, self._callback)

        # Base frame.
        s_frame = Tk.Frame(self.root, background=bg_color)
        s_frame.pack()

        # Create canvas frame with a canvas for drawing in.
        canv_frame = Tk.Frame(self.root)
        canv_frame.pack(in_=s_frame, side=Tk.LEFT)
        self.canvas = Tk.Canvas(self.root, width=self.window_width,
                                height=self.window_height, background='#FFFFFF',
                                borderwidth=0, relief=Tk.RAISED)
        self.canvas.pack(in_=canv_frame)
        self.canvas.bind('<Button-1>', self._left_click)

        # Available colors to be used by the vehicles.
        self.colors_list = ['blue', 'green', 'yellow', 'orange', 'red', 'pink', 'purple', 'cyan',
                            'dark green', 'coral', 'purple4', 'brown1']
        # Currently colors that are free to be used.
        self.free_colors = self.colors_list[:]

        # Create frame next to the canvas for buttons, labels etc.
        right_frame = Tk.Frame(self.root, background=bg_color)
        right_frame.pack(in_=s_frame, side=Tk.RIGHT, anchor=Tk.N)

        # Create button frame.
        button_frame = Tk.Frame(self.root, background=bg_color)
        button_frame.pack(in_=right_frame, side=Tk.TOP, anchor=Tk.N, pady=(0, 2 * ypad))

        trajectory_frame = Tk.Frame(self.root, background=bg_color)
        trajectory_frame.pack(in_=right_frame, side=Tk.TOP, anchor=Tk.N, pady=(0, 2 * ypad))

        # Create frame for changing reference path.
        path_frame = Tk.Frame(self.root, background=bg_color)
        path_frame.pack(in_=right_frame, side=Tk.TOP, pady=(0, 2 * ypad))

        # Create bottom frame for other stuff.
        bottom_frame = Tk.Frame(self.root, background=bg_color)
        bottom_frame.pack(in_=right_frame, side=Tk.TOP, anchor=Tk.N, pady=(2 * ypad, 0))

        # Button for quitting the program.
        quit_button = Tk.Button(self.root, text='Quit', command=self._quit1,
                                width=w1, height=2, background='red2', activebackground='red3')
        quit_button.pack(in_=button_frame)

        # Checkbox for displaying trajectories.
        self.trajectory_button_variable = Tk.IntVar()
        self.trajectory_button = Tk.Checkbutton(self.root, text='Display\ntrajectories',
                                                variable=self.trajectory_button_variable,
                                                command=self._trajectory_button_callback,
                                                width=w1, height=2, background=bg_color)
        if self.display_vehicle_tails:
            self.trajectory_button.toggle()
        self.trajectory_button.pack(in_=trajectory_frame, side=Tk.TOP)

        # Button for clearing trajectories.
        self.clear_button = Tk.Button(self.root, text='Clear trajectories',
                                      command=self._clear_trajectories, width=w1, height=2,
                                      background='orange', activebackground='dark orange')
        self.clear_button.pack(in_=trajectory_frame, side=Tk.TOP)

        # Widgets for changing reference path.
        self.path_label = Tk.Label(self.root, text='REFERENCE PATH', background=bg_color)
        self.path_label.pack(in_=path_frame, side=Tk.TOP)

        # Checkbox for displaying trajectories.
        self.path_button_variable = Tk.IntVar()
        self.path_button = Tk.Checkbutton(self.root, text='Display\nreference path',
                                          variable=self.path_button_variable,
                                          command=self._path_button_callback,
                                          width=w1, height=2, background=bg_color)
        if self.display_reference_path:
            self.path_button.toggle()
        self.path_button.pack(in_=path_frame, side=Tk.TOP)

        # Variables for changing the reference path.
        self.x_radius_variable = Tk.StringVar()
        self.x_radius_variable.set(0)
        self._make_entry(path_frame, bg_color, 'x_radius', textvariable=self.x_radius_variable)

        self.y_radius_variable = Tk.StringVar()
        self.y_radius_variable.set(0)
        self._make_entry(path_frame, bg_color, 'y_radius', textvariable=self.y_radius_variable)

        self.x_center_variable = Tk.StringVar()
        self.x_center_variable.set(0)
        self._make_entry(path_frame, bg_color, 'x_offset', textvariable=self.x_center_variable)

        self.y_center_variable = Tk.StringVar()
        self.y_center_variable.set(0)
        self._make_entry(path_frame, bg_color, 'y_offset', textvariable=self.y_center_variable)

        self.apply_path_button = Tk.Button(self.root, text='Apply new\nreference path',
                                           command=self._apply_path, width=w1, height=2,
                                           background='PaleGreen3', activebackground='PaleGreen4')
        self.apply_path_button.pack(in_=path_frame, side=Tk.TOP)

        # Buttons for zooming the area.
        zoom_frame = Tk.Frame(self.root, background=bg_color)
        zoom_frame.pack(in_=bottom_frame, side=Tk.TOP, anchor=Tk.N, pady=(2 * ypad, 0))
        zoom_in_button = Tk.Button(self.root, text='+', command = self._zoom_in)
        zoom_out_button = Tk.Button(self.root, text='-', command=self._zoom_out)
        zoom_out_button.pack(in_=zoom_frame, side=Tk.LEFT)
        zoom_in_button.pack(in_=zoom_frame, side=Tk.LEFT)

        # Buttons for truck size.
        truck_size_frame = Tk.Frame(self.root, background=bg_color)
        truck_size_frame.pack(in_=bottom_frame, side=Tk.TOP, anchor=Tk.N, pady=(ypad, 0))
        enlarge_trucks_button = Tk.Button(self.root, text='+', command=self._enlarge_vehicles)
        reduce_trucks_button = Tk.Button(self.root, text='-', command=self._reduce_vehicles)
        reduce_trucks_button.pack(in_=truck_size_frame, side = Tk.LEFT)
        enlarge_trucks_button.pack(in_=truck_size_frame, side = Tk.LEFT)

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

        lbl = Tk.Label(self.root, text=caption, background=background, width=8, anchor=Tk.W)
        lbl.pack(in_=frame, side=Tk.LEFT)

        entry = Tk.Entry(self.root, width=9, **options)
        entry.pack(in_=frame, side=Tk.LEFT)

    def _set_area_size(self, width=None, height=None):
        """Sets the width and height of the drawable area in meters. """
        if width is not None:
            self.width = width
        if height is not None:
            self.height = height

        self._draw_coordinate_frame()
        self._clear_trajectories()
        self._draw_path()

    def _zoom_in(self):
        """Zooms in on the canvas. """
        self._set_area_size(self.width/self.zoom_scale, self.height/self.zoom_scale)

    def _zoom_out(self):
        """Zooms out on the canvas. """
        self._set_area_size(self.width*self.zoom_scale, self.height*self.zoom_scale)

    def _enlarge_vehicles(self):
        """Makes the trucks bigger. """
        self.vehicle_length = self.vehicle_length * self.truck_zoom_scale
        self.vehicle_width = self.vehicle_width * self.truck_zoom_scale

    def _reduce_vehicles(self):
        """Makes the trucks smaller. """
        self.vehicle_length = self.vehicle_length / self.truck_zoom_scale
        self.vehicle_width = self.vehicle_width / self.truck_zoom_scale

    def _apply_path(self):
        """Apply changes made to the reference path in the entry widgets. """
        try:
            xr = float(self.x_radius_variable.get())
            yr = float(self.y_radius_variable.get())
            xc = float(self.x_center_variable.get())
            yc = float(self.y_center_variable.get())
            if xr <= 0 or yr <= 0:
                print('Invalid values entered')
            else:
                self.gen_circle_path([xr, yr], 400, [xc, yc])
                print('New reference path applied.')
        except ValueError:
            print('Invalid values entered.')

        self.x_radius_variable.set(self.x_radius)
        self.y_radius_variable.set(self.y_radius)
        self.x_center_variable.set(self.x_center)
        self.y_center_variable.set(self.y_center)

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
        yaw = data.yaw

        try:
            self._move_vehicle(vehicle_id, x, y, yaw)
            self._draw_tail(vehicle_id, x, y)
        except KeyError:
            self._create_new_vehicle(vehicle_id, x, y, yaw)

        # Store last time data was published in order to check inactivity.
        self.last_vehicle_published_time[vehicle_id] = time.time()

    def _move_vehicle(self, vehicle_id, x, y, theta):
        """Moves a truck triangle to the new position. """
        xf, yf, xr, yr, xl, yl = self._get_triangle_corners(x, y, theta)

        # Move polygon.
        self.canvas.coords(self.vehicle_polygons[vehicle_id], xf, yf, xr, yr, xl, yl)

    def _create_new_vehicle(self, vehicle_id, x, y, theta):
        """Creates a new vehicle triangle on the canvas. Adds the vehicle to the dictionaries. """
        print('Vehicle {} added.'.format(vehicle_id))

        xf, yf, xr, yr, xl, yl = self._get_triangle_corners(x, y, theta)
        color = self._get_random_color()

        # Create triangle.
        self.vehicle_polygons[vehicle_id] = self.canvas.create_polygon(xf, yf, xr, yr, xl, yl,
                                                                       fill=color, outline='black')

        self.vehicle_colors[vehicle_id] = color  # Store vehicle color.

        self.previous_vehicle_positions[vehicle_id] = [x, y]  # Store previous position.

    def _get_random_color(self):
        """Returns a random color from the list of available colors. """
        sr = random.SystemRandom()
        color = sr.choice(self.free_colors)
        index = self.free_colors.index(color)

        self.free_colors = [c for i, c in enumerate(self.free_colors) if i != index]

        if len(self.free_colors) == 0:
            self.free_colors = self.colors_list[:]

        return color

    def _draw_tail(self, vehicle_id, x, y):
        """Draws the last movement of the vehicle. """
        if self.display_vehicle_tails:
            state = Tk.NORMAL
        else:
            state = Tk.HIDDEN

        # Draw a line from the old position to the new.
        self._plot_sequence(
            [[self.previous_vehicle_positions[vehicle_id][0],
              self.previous_vehicle_positions[vehicle_id][1]], [x, y]],
            join=False, clr=self.vehicle_colors[vehicle_id], tag=vehicle_id, state=state)

        # Update old vehicle position.
        self.previous_vehicle_positions[vehicle_id] = [x, y]

    def _remove_inactive_vehicles(self):
        """Removes vehicles that have not published a position for a certain
        amount of time. Method calls itself periodically. """
        inactive_ids = []

        if self.remove_inactive_vehicles:
            # Find which vehicles are inactive.
            for vehicle_id in self.last_vehicle_published_time:
                if (time.time() - self.last_vehicle_published_time[vehicle_id] >
                        self.inactivity_time_limit):
                    inactive_ids.append(vehicle_id)

            # Remove all inactive vehicles.
            for vehicle_id in inactive_ids:
                self._remove_vehicle(vehicle_id)

        # Call method again after a delay.
        self.root.after(self.inactivity_check_delay, self._remove_inactive_vehicles)

    def _remove_vehicle(self, vehicle_id):
        """Removes the vehicle. """
        print('Vehicle {} removed because of inactivity.'.format(vehicle_id))

        self._remove_vehicle_drawings(vehicle_id)
        self._remove_vehicle_information(vehicle_id)

    def _remove_vehicle_drawings(self, vehicle_id):
        """Removes all traces of the vehicle from the canvas. """
        self.canvas.delete(self.vehicle_polygons[vehicle_id])
        self.canvas.delete(vehicle_id)

    def _remove_vehicle_information(self, vehicle_id):
        """Removes all stored information about the vehicle. """
        del self.vehicle_polygons[vehicle_id]
        del self.vehicle_colors[vehicle_id]
        del self.previous_vehicle_positions[vehicle_id]
        del self.last_vehicle_published_time[vehicle_id]

    def _raise_vehicles(self):
        """Raises all vehicle objects to the top of the canvas. """
        for vehicle_id in self.vehicle_polygons:
            self.canvas.tag_raise(self.vehicle_polygons[vehicle_id])

    def _draw_coordinate_frame(self):
        """Draw lines for the origin and create text displaying the coordinates
        in the corners. """
        self.canvas.delete(self.coordinate_frame_tag)

        # Create origin coordinate arrows.
        self.canvas.create_line(int(self.window_width / 2), int(self.window_height / 2),
                                int(self.window_width / 2),
                                int(self.window_height / 2) - 50,
                                width=2, arrow='last', tag=self.coordinate_frame_tag)
        self.canvas.create_line(int(self.window_width / 2), int(self.window_height / 2),
                                int(self.window_width / 2) + 50,
                                int(self.window_height / 2),
                                width=2, arrow='last', tag=self.coordinate_frame_tag)

        # Add coordinates to the corners.
        offset = 6
        self.canvas.create_text(offset, offset,
                                text='({:.1f}, {:.1f})'.format(-self.width / 2, self.height / 2),
                                anchor='nw', tag=self.coordinate_frame_tag)

        self.canvas.create_text(offset, self.window_height - offset,
                                text='({:.1f}, {:.1f})'.format(-self.width / 2, -self.height / 2),
                                anchor='sw', tag=self.coordinate_frame_tag)

        self.canvas.create_text(self.window_width - offset, self.window_height - offset,
                                text='({:.1f}, {:.1f})'.format(self.width / 2, -self.height / 2),
                                anchor='se', tag=self.coordinate_frame_tag)

        self.canvas.create_text(self.window_width - offset, offset,
                                text='({:.1f}, {:.1f})'.format(self.width / 2, self.height / 2),
                                anchor='ne', tag=self.coordinate_frame_tag)

    def _draw_closest_point(self, xy, clr='blue'):
        """Draw the closest point on the path for coordinates xy. """
        cross_length = 10
        _, closest = self.pt.get_closest(xy)
        xp, yp = self._real_to_pixel(closest[0], closest[1])

        self.canvas.create_line(xp - cross_length, yp - cross_length,
                                xp + cross_length, yp + cross_length,
                                fill=clr, tag='closest', width=2)
        self.canvas.create_line(xp - cross_length, yp + cross_length,
                                xp + cross_length, yp - cross_length,
                                fill=clr, tag='closest', width=2)

    def _plot_sequence(self, seq, join=False, clr='blue', width=2, tag='line', state=Tk.NORMAL):
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
                    self.canvas.create_line(x1, y1, x2, y2, fill=clr, width=width, tag=tag,
                                            state=state)
            except Exception as e:
                print('Error when plotting sequence: {}'.format(e))

    def _get_triangle_corners(self, xreal, yreal, yaw):
        """Returns the three coordinate pairs in pixel for the triangle
        corresponding to the vehicle position. """
        length = self.vehicle_length  # Truck length in meters.
        width = self.vehicle_width

        # Coordinates for frontal corner.
        xf, yf = self._real_to_pixel(xreal + length / 2 * math.cos(yaw),
                                     yreal + length / 2 * math.sin(yaw))
        # Coordinates for rear right corner.
        xr, yr = self._real_to_pixel(
            xreal - length / 2 * math.cos(yaw) + width / 2 * math.cos(yaw - math.pi / 2),
            yreal - length / 2 * math.sin(yaw) + width / 2 * math.sin(yaw - math.pi / 2))

        # Coordinates for rear left corner.
        xl, yl = self._real_to_pixel(
            xreal - length / 2 * math.cos(yaw) + width / 2 * math.cos(yaw + math.pi / 2),
            yreal - length / 2 * math.sin(yaw) + width / 2 * math.sin(yaw + math.pi / 2))

        return xf, yf, xr, yr, xl, yl

    def _trajectory_button_callback(self):
        """Callback for trajectory check button. Enable/disable plotting of trajectories. """
        if self.trajectory_button_variable.get() == 1:
            self.display_vehicle_tails = True
            self._show_trajectories()
        else:
            self.display_vehicle_tails = False
            self._hide_trajectories()

    def _hide_trajectories(self):
        """Hides all vehicle trajectories. """
        for vehicle_id in self.vehicle_polygons:
            self._hide_canvas_tag(vehicle_id)

    def _show_trajectories(self):
        """Shows all vehicle trajectories. """
        for vehicle_id in self.vehicle_polygons:
            self._show_canvas_tag(vehicle_id)

    def _path_button_callback(self):
        """Callback for path check button. Enable/disable plotting of reference path. """
        if self.path_button_variable.get() == 1:
            self.display_reference_path = True
            self._draw_path()
            self.clear_button.config(state='normal')
        else:
            self.display_reference_path = False
            self.canvas.delete(self.PATH_TAG)

    def _real_to_pixel(self, xreal, yreal):
        """Transform from real to pixel coordinates. """
        xpixel = int(self.window_width / self.width * xreal + self.window_width / 2)
        ypixel = int(
            -self.window_height / self.height * yreal + self.window_height / 2)
        return xpixel, ypixel

    def _pixel_to_real(self, xp, yp):
        """Transform from pixel to real coordinates. """
        xreal = float((xp - self.window_width / 2) * self.width / self.window_width)
        yreal = float((self.window_height / 2 - yp) * self.height / self.window_height)

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
        handles = self.canvas.find_withtag(tag)
        for x in handles:
            self.canvas.itemconfig(x, state=Tk.HIDDEN)

    def _show_canvas_tag(self, tag):
        """Show all canvas items with the specified tag. """
        handles = self.canvas.find_withtag(tag)
        for x in handles:
            self.canvas.itemconfig(x, state=Tk.NORMAL)

    def _clear_trajectories_periodically(self):
        """Clears all vehicle trajectories. Method calls itself after a certain
        time delay. """
        if self.periodically_clear_trajectories:
            self._clear_trajectories()
        self.root.after(
            self.clear_seconds * 1000, self._clear_trajectories_periodically)

    def _clear_trajectories(self):
        """Clear the saved truck trajectories. """
        print('Trajectories cleared. ')
        for vehicle_id in self.vehicle_polygons:
            self.canvas.delete(vehicle_id)

    def _draw_path(self):
        """Draws the reference path. """
        self.canvas.delete(self.PATH_TAG)
        if self.display_reference_path:
            self._plot_sequence(self.pt.path, join=True, tag=self.PATH_TAG,
                                clr=self.PATH_COLOR, width=2)
            self._raise_vehicles()

    def gen_circle_path(self, radius, points, center):
        """Generates a circle/ellipse path. """
        self.x_radius = radius[0]
        self.y_radius = radius[1]
        self.x_center = center[0]
        self.y_center = center[1]

        self.x_radius_variable.set(self.x_radius)
        self.y_radius_variable.set(self.y_radius)
        self.x_center_variable.set(self.x_center)
        self.y_center_variable.set(self.y_center)

        self.pt.gen_circle_path(radius, points, center)
        self.canvas.delete(self.PATH_TAG)
        self._draw_path()


def main():
    topic_name = 'mocap_state'  # Name of topic that the node subscribes to.
    topic_type = MocapState     # The type of the topic.

    width = 6                   # Width in meters of displayed area.
    height = 6                  # Height in meters.
    win_size = 500              # Size of window in pixels.

    # Variables for reference path.
    x_radius = 1.40                 # Ellipse x-radius.
    y_radius = 1.20                 # Ellipse y-radius.
    center = [0.2, -y_radius/2]     # The coordinates of the center of the ellipse.
    pts = 200                       # Number of points on displayed reference path.

    root = Tk.Tk()
    try:
        # Create class.
        truckplot = TruckPlot(root, topic_type, topic_name,
                              width=width, height=height, vehicle_length=0.5, win_size=win_size)

        # Add a reference path.
        truckplot.gen_circle_path([x_radius, y_radius], pts, center=center)

        # Run the GUI.
        root.mainloop()

    except RuntimeError as e:
        print('Error when running GUI: {}'.format(e))


if __name__ == '__main__':
    main()
