import tkinter as tk
from tkinter import *
import trial


class Custom_slider:

    def get_value(self, var):
        self.z = float(var)
        return var

    def create_sliders(self, distance_top, width, height, upper_limit, lower_limit):

        self.city_frame = tk.Frame(root, bg='gray5', bd=0.5,
                                   highlightcolor='black', highlightthickness=0, borderwidth=0)

        self.city_frame.place(relx=0.225, rely=distance_top,
                              relwidth=width, relheight=height, anchor='n')

        self.w = Scale(self.city_frame, from_=lower_limit, to=upper_limit, font=('Comic Sans Ms', 11),
                       orient=HORIZONTAL, highlightthickness=0, borderwidth=0, resolution=0.02,
                       troughcolor="gray2", bg='gray5', cursor='dot', fg='deep sky blue', command=self.get_value)

        self.w.place(relwidth=1.0, relheight=1.0)
        self.z = self.w.get()
        return


class Custom_text_inputs:

    def create_title_frame(self, frame, position_x, position_y, title):

        self.joint_text_frame = tk.Frame(frame, bg='gray5', bd=0, highlightthickness=0)
        self.joint_text_frame.place(relx=position_x, rely=position_y, relwidth=0.25,
                            relheight=0.08, anchor='n')

        self.joint_text = tk.Entry(self.joint_text_frame, bg='gray5',
                            fg='deep sky blue', font=('Comic Sans Ms', 12, 'bold'), justify='center',
                            highlightthickness=0, borderwidth=0)
        self.joint_text.insert(0, title)
        self.joint_text.place(relwidth=1, relheight=1)

    def create_joint_frames(self, frame, position_x, position_y):

        self.box_values_frame = tk.Frame(frame, bg='gray5', bd=1,
                                    highlightthickness=0, borderwidth=0)
        self.box_values_frame.place(relx=position_x, rely=position_y,
                            relwidth=0.25, relheight=0.275, anchor='n')

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

        self.box_value_x = tk.Entry(self.box_values_frame, bg='gray2', fg='white', justify='center', font=(
            'Comic Sans MS', 11), highlightthickness=0, borderwidth=0, insertbackground='white')
        self.box_value_x.insert(1, "0")
        self.box_value_x.place(relx=0.25, rely=0, relwidth=0.5, relheight=0.34)
        float(self.box_value_x.get())

        self.box_value_y = tk.Entry(self.box_values_frame, bg='gray2', fg='white', justify='center', font=(
            'Comic Sans MS', 11), highlightthickness=0, borderwidth=0, insertbackground='white')
        self.box_value_y.insert(1, "0")
        self.box_value_y.place(relx=0.25, rely=0.34, relwidth=0.5, relheight=0.33)
        float(self.box_value_y.get())

        self.box_value_z = tk.Entry(self.box_values_frame, bg='gray2', fg='white', justify='center', font=(
            'Comic Sans MS', 11), highlightthickness=0, borderwidth=0, insertbackground='white')
        self.box_value_z.insert(1, "0")
        self.box_value_z.place(relx=0.25, rely=0.67, relwidth=0.5, relheight=0.33)
        float(self.box_value_z.get())

        self.button_main = tk.Button(self.box_values_frame, text="Set", bg='gray15', justify='center',
                            fg='white', font=('Comic Sans MS', 10), highlightthickness=0, borderwidth=0, command=self.get_values_from_box)
        self.button_main.place(relx=0.75, rely=0.34, relwidth=0.25, relheight=0.25)

    
    def get_values_from_box(self):
        self.list_values = [float(self.box_value_x.get()), float(self.box_value_y.get()), float(self.box_value_z.get())]
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

# BACKGROUND IMAGE CODE
background_image = tk.PhotoImage(file='robot.png')
background_label = tk.Label(root,  image=background_image)
background_label.place(relwidth=1, relheight=1)


# SLIDER SPECIFIC CODE


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


# BOX POSE AND GOAL POSE CODE


box_values = Custom_text_inputs()
box_values.create_title_frame(root, 0.775, 0.10, "Enter Box Position")
box_values.create_joint_frames(root, 0.775, 0.2)
# box_xyz = box_position_values.get_values_from_box()


goal_pose_values = Custom_text_inputs()
goal_pose_values.create_title_frame(root, 0.775, 0.5, "Enter Goal Position")
goal_pose_values.create_joint_frames(root, 0.775, 0.575)


# BUTTON CODES TO SET AND EXECUTE

execute_frame = tk.Frame(root, bg='gray5', highlightthickness=0, borderwidth=0)
execute_frame.place(
    relx=0.48, rely=0.85, relwidth=0.25, relheight=0.1, anchor='n')


button_execute = tk.Button(execute_frame, text="Execute", bg='SpringGreen3',
                           fg='black', font=('Comic Sans MS', 12), highlightthickness=0, borderwidth=0,
                           command=lambda: execution_task())
button_execute.place(relx=0.25, relwidth=0.75, relheight=1)


mainloop()
root.mainloop()
