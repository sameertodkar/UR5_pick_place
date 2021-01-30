import tkinter as tk
from tkinter import *


def delete_text(event):
    """
    This function is used to delete the default text in the entry textbox
    Arguments:
        event {string} -- left click of the mouse
    """
    entry.delete(0, "end")


def append_values(val):
    with open("slider_values.txt".format(i), "w+") as f:
        f.writelines(val) 


def create_sliders(distance_top, width, height, limits):
    city_frame = tk.Frame(root, bg='white', bd=1)
    city_frame.place(relx=0.2, rely=distance_top,
                    relwidth=width, relheight=height, anchor='n')

    w = Scale(city_frame, from_=limits*(-1), to=limits, orient=HORIZONTAL,
            resolution=0.2, troughcolor="white", command=append_values)
    w.place(relwidth=1, relheight=1)

# def add_push_button:



root = tk.Tk()
root.title("Vention_workspace")

title = tk.Label(text="Robot Manipulation")

height_of_canvas = 500
width_of_canvas = 810
canvas = tk.Canvas(height=height_of_canvas, width=width_of_canvas).pack()

# city_frame = tk.Frame(root, bg='sky blue', bd=1)
# city_frame.place(relx=0.1, rely=0.1,
#                  relwidth=0.08, relheight=0.08, anchor='n')


# entry = tk.Entry(city_frame, bg='white', font=('Comic Sans MS', 12))
# entry.insert(0, "")
# entry.place(relwidth=1, relheight=1)
# entry.bind("<Button-1>", delete_text)

list = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
limits = [1, 2, 3, 4, 5, 6]

for i in range(6):
    create_sliders(list[i], 0.3, 0.1, limits[i])

mainloop()
root.mainloop()
