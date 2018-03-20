#!/usr/bin/env python

import Tkinter as Tk
import controllerGUI


class ControllerGUI():
    """GUI for controller. """
    def __init__(self, controller):
        self.root = Tk.Tk()
        self.controller = controller

        self.root.title('Control panel')

        bg_color0 = 'SlateGray2'
        bg_color1 = 'SlateGray3'
        bg_color2 = 'SlateGray3'
        bg_color3 = 'SlateGray3'
        w1 = 15
        ypad = 10

        # Base frame.
        base_frame = Tk.Frame(self.root, background = bg_color0)
        base_frame.pack()

        # Frame in base frame.
        left_frame = Tk.Frame(self.root, background = bg_color0)
        left_frame.pack(in_ = base_frame, side = Tk.LEFT)

        # Frame containing quit button.
        btnframe1 = Tk.Frame(self.root, background = bg_color1)
        btnframe1.pack(in_ = left_frame, side = Tk.TOP)

        # Frame containing widgets for starting and stopping the controller.
        startstopframe = Tk.Frame(self.root, background = bg_color1)
        startstopframe.pack(in_ = left_frame, side = Tk.TOP, pady = (2*ypad, 0))

        # Frame containging widgets for changing control parameters.
        changes_frame = Tk.Frame(self.root, background = bg_color2)
        changes_frame.pack(in_ = left_frame, side = Tk.TOP, pady = (2 * ypad, 0))

        # Frame containing widgets for changing reference path.
        path_frame = Tk.Frame(self.root, background = bg_color3)
        path_frame.pack(in_ = left_frame, side = Tk.TOP, pady = (2 * ypad, 0))

        # Button for quitting the program.
        quit_button = Tk.Button(self.root, text ='Quit',
                                command = self.quit1,
                                width = w1, height = 2, background = 'red2',
                                activebackground = 'red3')
        quit_button.pack(in_ = btnframe1, side = Tk.TOP)

        # Button for starting the program.
        start_button = Tk.Button(self.root, text ='Start controller',
                                 command = self.start,
                                 width = w1, height = 2, background = 'green2',
                                 activebackground = 'green3')
        start_button.pack(in_ = startstopframe)

        # Button for stopping the controller.
        stop_button = Tk.Button(self.root, text ='Stop controller',
                                command = self.stop,
                                width = w1, height = 2, background = 'light coral',
                                activebackground = 'coral')
        stop_button.pack(in_ = startstopframe)

        # Label saying if the controller is running or not.
        self.running_text_var = Tk.StringVar()
        self.running_label = Tk.Label(self.root,
                                      textvariable = self.running_text_var, anchor = Tk.W,
                                      justify = Tk.LEFT, width = w1, height = 2,
                                      background = bg_color1, foreground = 'grey')
        self.running_text_var.set('Controller stopped\n')
        self.running_label.pack(in_ = startstopframe)

        # Widgets for changing controller parameters.

        # Get a list with descriptive names of the adjustable parameters and
        # one list of the current values of those parameters.
        self.adjustables, self.adj_values = controller.get_adjustables()
        self.adj_vars = [] # List of textvariables for displaying the values.

        if len(self.adjustables) > 0:
            # Create label for the frame.
            tuning_label = Tk.Label(self.root, text ='CONTROLLER TUNING',
                                    background = bg_color2)
            tuning_label.pack(in_ = changes_frame, side = Tk.TOP)

            try:
                # Create an entry for each adjustable value. Add the StringVars
                # to a list so that they can be accessed later.
                for adj, val in zip(self.adjustables, self.adj_values):
                    strvar = Tk.StringVar()
                    strvar.set(val)
                    self.adj_vars.append(strvar)
                    self.make_entry(changes_frame, bg_color2, adj, textvariable = strvar)

            except Exception as e:
                print('Error when creating entries for adjustables. :'.format(
                    e))

            # Create button for applying the changes entered in the entries.
            self.apply_button = Tk.Button(self.root, text ='Apply changes',
                                          command = self.apply, width = w1, height = 2,
                                          background = 'medium sea green',
                                          activebackground = 'sea green')
            self.apply_button.pack(in_ = changes_frame, side = Tk.TOP)

        if False:
            # Widgets for changing reference path.
            try:
                self.path_label = Tk.Label(self.root, text ='REFERENCE PATH', background=bg_color3)
                self.path_label.pack(in_ = path_frame, side = Tk.TOP)

                self.xr_var = Tk.StringVar()    # x-radius
                self.xr_var.set(str(self.controller.pt.xr))
                self.make_entry(path_frame, bg_color3, 'x_radius', textvariable = self.xr_var)

                self.yr_var = Tk.StringVar()    # y-radius
                self.yr_var.set(str(self.controller.pt.yr))
                self.make_entry(path_frame, bg_color3, 'y_radius', textvariable = self.yr_var)

                self.xc_var = Tk.StringVar()    # x-coordinate of center
                self.xc_var.set(str(self.controller.pt.xc))
                self.make_entry(path_frame, bg_color3, 'x_offset', textvariable = self.xc_var)

                self.yc_var = Tk.StringVar()    # y-coordinate of center.
                self.yc_var.set(str(self.controller.pt.yc))
                self.make_entry(path_frame, bg_color3, 'y_offset', textvariable = self.yc_var)

                # Create button for applying the changes to the reference path.
                self.apply_path_button = Tk.Button(self.root,
                                                   text = 'Apply new\nreference path',
                                                   command = self.apply_path, width = w1, height = 2,
                                                   background = 'medium sea green',
                                                   activebackground = 'sea green')
                self.apply_path_button.pack(in_ = path_frame, side = Tk.TOP)
            except Exception as e:
                print('Could not display reference path panel in GUI: {}'.format(e))

        # Actions for closing the window and pressing ctrl-C on the window.
        self.root.protocol('WM_DELETE_WINDOW', self.quit1)
        self.root.bind('<Control-c>', self.quit2)
        self.root.bind('<Escape>', self.quit2)

        # Bind buttons for stopping the controller.
        self.root.bind('e', self.keypress_stop)
        self.root.bind('q', self.keypress_stop)
        self.root.bind('s', self.keypress_stop)
        self.root.bind('w', self.keypress_start)

        self.control_loop()

        self.root.mainloop()

    def control_loop(self):
        self.controller.control()
        self.root.after(int(self.controller.dt*1000), self.control_loop)

    def make_entry(self, framep, background, caption, **options):
        """Makes a tk entry with label to the left of it. """
        frame = Tk.Frame(self.root, background = background)
        frame.pack(in_ = framep, side = Tk.TOP)
        lbl = Tk.Label(self.root, text = caption, background = background,
                       width = 8, anchor = Tk.W)
        lbl.pack(in_ = frame, side = Tk.LEFT)
        entry = Tk.Entry(self.root, width = 9, **options)
        entry.pack(in_ = frame, side = Tk.LEFT)


    def quit1(self):
        """Callback for quit_button and window closing. Quits the GUI. """
        print('Quitting.')
        self.controller.stop()
        self.root.quit()


    def quit2(self, event):
        """Callback from pressing Ctrl-C. Quits the GUI. """
        print('Quitting.')
        self.controller.stop()
        self.root.quit()


    def start(self):
        """Callback for start_button. Starts controller. """
        self.controller.start()
        self.running_text_var.set('Controller running\n')
        self.running_label.config(foreground = 'black')


    def keypress_stop(self, event):
        """Called when pressing certain keys on the window. Stops the truck. """
        self.stop()


    def keypress_start(self, event):
        """Called when pressing keys to start controller. """
        self.start()


    def stop(self):
        """Callback for stop_button. Stops the controller. """
        self.controller.stop()
        self.running_text_var.set('Controller stopped\n')
        self.running_label.config(foreground = 'grey')
        self.root.after(100, self.stop_again)

    def stop_again(self):
        """Method for calling the controller's stop method a second time to
        make sure that the truck stops. """
        self.controller.stop()


    def apply(self):
        """Callback for apply_button. Tries to apply the specified values in the
        entries to the controller. """
        # Gather the values in the entries into a list.
        values = []
        for textvar in self.adj_vars:
            values.append(textvar.get())

        # Try to set the controller parameters to the entered values.
        self.controller.set_adjustables(values)

        # Get the new (or old if failed) values of the parameters.
        _, values = self.controller.get_adjustables()
        for val, textvar in zip(values, self.adj_vars):
            textvar.set(val)

        self.apply_button.focus()


    def apply_path(self):
        """Callback for apply path button. Apply new reference path. """
        try:
            xr = float(self.xr_var.get())
            yr = float(self.yr_var.get())
            xc = float(self.xc_var.get())
            yc = float(self.yc_var.get())
            if xr < 0 or yr < 0:
                print('Invalid values entered')
            else:
                self.controller.set_reference_path([xr, yr], [xc, yc])
                print('New reference path applied.')
        except Exception as e:
            print('Invalid values entered.')

        try:
            self.xr_var.set(self.controller.pt.xr)
            self.yr_var.set(self.controller.pt.yr)
            self.xc_var.set(self.controller.pt.xc)
            self.yc_var.set(self.controller.pt.yc)
        except Exception as e:
            print('Failed to display reference path values: {}'.format(e))
            pass

        self.apply_path_button.focus()


def main():
    pass


if __name__ == '__main__':
    main()
