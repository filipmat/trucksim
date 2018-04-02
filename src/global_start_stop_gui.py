#!/usr/bin/env python


# Class with a GUI for starting and stopping controllers.
# Publishes start/stop (1/0) to a topic. Controllers subscribing to that topic should start and
# stop accordingly.


import Tkinter as Tk

import rospy
from trucksim.msg import ControllerRun


class GlobalRunGUI():
    """GUI for controller. """
    def __init__(self, topic_name):
        self.root = Tk.Tk()

        self.root.title('Global run')

        rospy.init_node('global_run', anonymous=False)

        self.publisher = rospy.Publisher(topic_name, ControllerRun, queue_size=1)

        bg_color0 = 'SlateGray2'
        bg_color1 = 'SlateGray3'
        w1 = 15
        ypad = 10

        # Base frame.
        base_frame = Tk.Frame(self.root, background = bg_color0)
        base_frame.pack()

        # Frame in base frame.
        left_frame = Tk.Frame(self.root, background = bg_color0)
        left_frame.pack(in_ = base_frame, side = Tk.LEFT)

        # Frame containing quit button.
        quit_button_frame = Tk.Frame(self.root, background = bg_color1)
        quit_button_frame.pack(in_ = left_frame, side = Tk.TOP)

        # Frame containing widgets for starting and stopping the controller.
        start_stop_frame = Tk.Frame(self.root, background = bg_color1)
        start_stop_frame.pack(in_ = left_frame, side = Tk.TOP, pady = (2*ypad, 0))

        # Button for quitting the program.
        quit_button = Tk.Button(self.root, text ='Quit',
                                command = self.quit1,
                                width = w1, height = 2, background = 'red2',
                                activebackground = 'red3')
        quit_button.pack(in_ = quit_button_frame, side = Tk.TOP)

        # Button for starting the controller.
        start_button = Tk.Button(self.root, text ='Start controller',
                                 command = self.start,
                                 width = w1, height = 2, background = 'green2',
                                 activebackground = 'green3')
        start_button.pack(in_ = start_stop_frame)

        # Button for stopping the controller.
        stop_button = Tk.Button(self.root, text ='Stop controller',
                                command = self.stop,
                                width = w1, height = 2, background = 'light coral',
                                activebackground = 'coral')
        stop_button.pack(in_ = start_stop_frame)

        # Label saying whether the controller is running or not.
        self.running_text_var = Tk.StringVar()
        self.running_label = Tk.Label(self.root,
                                      textvariable = self.running_text_var, anchor = Tk.W,
                                      justify = Tk.LEFT, width = w1, height = 2,
                                      background = bg_color1, foreground = 'grey')
        self.running_text_var.set('Controller stopped\n')
        self.running_label.pack(in_ = start_stop_frame)

        # Actions for closing the window and pressing ctrl-C on the window.
        self.root.protocol('WM_DELETE_WINDOW', self.quit1)
        self.root.bind('<Escape>', self.quit2)

        # Bind buttons for starting and stopping the controller.
        self.root.bind('e', self.keypress_stop)
        self.root.bind('q', self.keypress_stop)
        self.root.bind('s', self.keypress_stop)
        self.root.bind('<Control-c>', self.keypress_stop)
        self.root.bind('w', self.keypress_start)

        self.root.mainloop()

    def quit1(self):
        """Callback for quit_button and window closing. Quits the GUI. """
        print('Quitting.')
        self.stop()
        self.root.quit()

    def quit2(self, event):
        """Callback from pressing Ctrl-C. Quits the GUI. """
        self.quit1()

    def keypress_stop(self, event):
        """Called when pressing certain keys on the window. Stops the truck. """
        self.stop()

    def keypress_start(self, event):
        """Called when pressing keys to start controller. """
        self.start()

    def start(self):
        """Callback for start_button. Starts controller. """
        self.publish_start()
        self.running_text_var.set('Running...\n')
        self.running_label.config(foreground = 'black')

    def stop(self):
        """Callback for stop_button. Stops controller. """
        self.publish_stop()
        self.running_text_var.set('Stopped.\n')
        self.running_label.config(foreground='grey')
        self.root.after(100, self.stop_again)

    def stop_again(self):
        """Method for calling stop a second time to make sure that the vehicle stops. """
        self.publish_stop()

    def publish_start(self):
        self.publisher.publish(1)

    def publish_stop(self):
        self.publisher.publish(0)


def main():
    topic_name = 'global/run'

    GlobalRunGUI(topic_name)


if __name__ == '__main__':
    main()
