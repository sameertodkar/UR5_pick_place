
# The code is used to generate the Graphical User Interface for interacting with the robot. The code
# makes use of Tkinter library to access functionaity of sliders, Entry, and buttons.

# Simplest way to create a GUI
# Step1: Create a canvas
# Step2: Create a frame on the canvas and place with relative values with the canvas
# Step3: Add buttons, textboxes, sliders etc in the frames and then provide relative values wrt to frames

# Aligning widgets, buttons with relative values eases the task of making GUI

# Author: Sameer Todkar
# Date: 30 Jan 2021

import tkinter as tk
from tkinter import *
import trial


class Custom_slider:
    """
    Class to design a custom slider which will return the value based on the slider input
    """

    def get_value(self, var):
        """
        Method to extract value from the slider

        Returns:
            float: [description]
        """
        self.z = float(var)
        return var

    def create_sliders(self, distance_top, width, height, upper_limit, lower_limit):
        """
        Method to create a slider based on:
        lower limit of slider
        upper limit of slider
        position of the slider in the frame

        """

        self.slider_frame = tk.Frame(root, bg='gray5', bd=0.5,
                                   highlightcolor='black', highlightthickness=0, borderwidth=0)

        self.slider_frame.place(relx=0.225, rely=distance_top,
                              relwidth=width, relheight=height, anchor='n')

        self.w = Scale(self.slider_frame, from_=lower_limit, to=upper_limit, font=('Comic Sans Ms', 11),
                       orient=HORIZONTAL, highlightthickness=0, borderwidth=0, resolution=0.02,
                       troughcolor="gray2", bg='gray5', cursor='dot', fg='deep sky blue', command=self.get_value)

        self.w.place(relwidth=1.0, relheight=1.0)
        self.z = self.w.get()
        return


class Custom_text_inputs:

    """
    Class to create text inputs in tkinter, and extract the value provided by the user in the GUI
    """

    def create_title_frame(self, frame, position_x, position_y, title):
        """
        Method to create a title boxes, these text boxes does not take user input
        """

        self.joint_text_frame = tk.Frame(
            frame, bg='gray5', bd=0, highlightthickness=0)
        self.joint_text_frame.place(relx=position_x, rely=position_y, relwidth=0.25,
                                    relheight=0.08, anchor='n')

        self.joint_text = tk.Entry(self.joint_text_frame, bg='gray5',
                                   fg='deep sky blue', font=('Comic Sans Ms', 12, 'bold'), justify='center',
                                   highlightthickness=0, borderwidth=0)
        self.joint_text.insert(0, title)
        self.joint_text.place(relwidth=1, relheight=1)

    def create_joint_frames(self, frame, position_x, position_y):
        """
        Method to create boxes that take user input and displays the value on the screen

        Args:
            frame: parent in which the textbox will be placed 
            position_x (float): placement of the textbox in x axis 
            position_y (float): placement of the textbox in y axis 
        """

        # Code for creating a parent frame on the root

        self.box_values_frame = tk.Frame(frame, bg='gray5', bd=1,
                                         highlightthickness=0, borderwidth=0)
        self.box_values_frame.place(relx=position_x, rely=position_y,
                                    relwidth=0.25, relheight=0.275, anchor='n')

        # Code for creating x, y, z textboxes

        x_text = tk.Entry(self.box_values_frame, bg='gray5',
                          fg='white', font=('Comic Sans Ms', 12, 'bold'), justify='center',
                          highlightthickness=0, borderwidth=0)
        x_text.insert(0, "x:")
        x_text.place(relx=0, rely=0, relwidth=0.25, relheight=0.34)

        y_text = tk.Entry(self.box_values_frame, bg='gray5',
                          fg='white', font=('Comic Sans Ms', 12, 'bold'), justify='center', highlightthickness=0, borderwidth=0)
        y_text.insert(0, "y:")
        y_text.place(relx=0, rely=0.34, relwidth=0.25, relheight=0.33)

        z_text = tk.Entry(self.box_values_frame, bg='gray5',
                          fg='white', font=('Comic Sans Ms', 12, 'bold'), justify='center', highlightthickness=0, borderwidth=0)
        z_text.insert(0, "z:")
        z_text.place(relx=0, rely=0.67, relwidth=0.25, relheight=0.33)

        # Code for creating textboxes which will take input from the user

        self.box_value_x = tk.Entry(self.box_values_frame, bg='gray2', fg='white', justify='center', font=(
            'Comic Sans MS', 11), highlightthickness=0, borderwidth=0, insertbackground='white')
        self.box_value_x.insert(1, "0")
        self.box_value_x.place(relx=0.25, rely=0, relwidth=0.5, relheight=0.34)
        float(self.box_value_x.get())

        self.box_value_y = tk.Entry(self.box_values_frame, bg='gray2', fg='white', justify='center', font=(
            'Comic Sans MS', 11), highlightthickness=0, borderwidth=0, insertbackground='white')
        self.box_value_y.insert(1, "0")
        self.box_value_y.place(relx=0.25, rely=0.34,
                               relwidth=0.5, relheight=0.33)
        float(self.box_value_y.get())

        self.box_value_z = tk.Entry(self.box_values_frame, bg='gray2', fg='white', justify='center', font=(
            'Comic Sans MS', 11), highlightthickness=0, borderwidth=0, insertbackground='white')
        self.box_value_z.insert(1, "0")
        self.box_value_z.place(relx=0.25, rely=0.67,
                               relwidth=0.5, relheight=0.33)
        float(self.box_value_z.get())

        # Code for creating a button which executes the command to extract values from the GUI

        self.button_main = tk.Button(self.box_values_frame, text="Set", bg='gray15', justify='center',
                                     fg='white', font=('Comic Sans MS', 10), highlightthickness=0, borderwidth=0, command=self.get_values_from_box)
        self.button_main.place(relx=0.75, rely=0.34,
                               relwidth=0.25, relheight=0.25)

    def get_values_from_box(self):
        """
        Method extracting the values from textboxes

        Returns:
            [list]: Position variables of goal and box pose
        """
        self.list_values = [float(self.box_value_x.get()), float(
            self.box_value_y.get()), float(self.box_value_z.get())]
        print("Position specified: {0}".format(self.list_values))
        return self.list_values


def execution_task():
    trial.execute_pick_place([sliders[0].z, sliders[1].z, sliders[2].z,
                              sliders[3].z, sliders[4].z, sliders[5].z], box_values.list_values, goal_pose_values.list_values)


root = tk.Tk()
root.title("Vention UR5")
root.resizable(FALSE, FALSE)
height_of_canvas = 500
width_of_canvas = 810
canvas = tk.Canvas(height=height_of_canvas,
                   width=width_of_canvas, bg='white').pack()


############### BACKGROUND IMAGE CODE ###############

background_image = tk.PhotoImage(file='robot.png')
background_label = tk.Label(root,  image=background_image)
background_label.place(relwidth=1, relheight=1)


###############  SLIDERS SPECIFIC CODE ###############

title_joint_angles = Custom_text_inputs()

title_joint_angles.create_title_frame(root, 0.225, 0.10, "Enter Joint Angles")

distances_top = [0.225, 0.325, 0.425, 0.525, 0.625, 0.725]
upper_limit_joint = [3.14, 0, 3.14, 3.14, 3.14, 3.14]
lower_limit_joint = [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14]


sliders = [Custom_slider(), Custom_slider(), Custom_slider(),
           Custom_slider(), Custom_slider(), Custom_slider()]

for i in range(len(distances_top)):
    sliders[i].create_sliders(distances_top[i], 0.3, 0.1, upper_limit_joint[i],
                              lower_limit_joint[i])

###############  BOX POSE AND GOAL POSE CODE ###############


box_values = Custom_text_inputs()
box_values.create_title_frame(root, 0.775, 0.10, "Enter Box Position")
box_values.create_joint_frames(root, 0.775, 0.2)


goal_pose_values = Custom_text_inputs()
goal_pose_values.create_title_frame(root, 0.775, 0.5, "Enter Goal Position")
goal_pose_values.create_joint_frames(root, 0.775, 0.575)


############### BUTTON CODES TO SET AND EXECUTE ###############

execute_frame = tk.Frame(root, bg='gray5', highlightthickness=0, borderwidth=0)
execute_frame.place(
    relx=0.48, rely=0.85, relwidth=0.25, relheight=0.1, anchor='n')


button_execute = tk.Button(execute_frame, text="Execute", bg='SpringGreen3',
                           fg='black', font=('Comic Sans MS', 12), highlightthickness=0, borderwidth=0,
                           command=lambda: execution_task())
button_execute.place(relx=0.25, relwidth=0.75, relheight=1)


mainloop()
root.mainloop()
