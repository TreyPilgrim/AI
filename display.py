import tkinter as tk
import numpy as np
import simToMap as stm
import matplotlib.pyplot as plt
import DStarLite

RESOLUTION = 25
myArray = []
map = np.zeros((25, 25))
dstar = DStarLite.DStarLite((0,0), (RESOLUTION, RESOLUTION), map)

def showGrid():
    global myArray
    npArray = np.array(myArray)
    map = stm.generate_map(npArray, 500, RESOLUTION)
    plt.imshow(map)
    plt.show()


def circle(event):
    x = event.x
    y = event.y
    global myArray
    myArray.append((x, y))
    canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="red")

def drawPath():
    global myArray
    global dstar

    npArray = np.array(myArray)
    map = stm.generate_map(npArray, 500, RESOLUTION)


    dstar.replan()
    path = dstar.extract_path()
    print(path)
    # canvas.create_line()





window = tk.Tk()
canvas = tk.Canvas(window, width=500, height=500, background="white")

canvas.bind("<B1-Motion>", circle)
canvas.pack()

stepButton = tk.Button(window, text="Show Gridworld", command=showGrid)
stepButton.pack()

pathbutton = tk.Button(window, text = "Show Best Path", command=drawPath)
pathbutton.pack()

window.mainloop()

