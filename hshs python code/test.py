#red = 255
#green = 255
#color_string = f'{128:x}{255:x}00'
#print (f'{255:x}')
#print(color_string)
#print(f'{14:02x}')
#red_level = 20
#red_level = min(red_level, 255)
#red_level = int(max(red_level, 0))
#color_string = f'{red_level:x}ff00'
#print(f'red level: {red_level}, color string: {color_string}')


import tkinter as tk


# Define the data points
data = [20, 15, 10, 7, 5, 4, 3, 2, 1, 1, 0]

root = tk.Tk()
root.title("Bar Graph")

c_width = 400  # Define it's width
c_height = 350  # Define it's height
c = tk.Canvas(root, width=c_width, height=c_height, bg='white')
c.pack()

# The variables below size the bar graph
y_stretch = 15  # The highest y = max_data_value * y_stretch
y_gap = 20  # The gap between lower canvas edge and x axis
x_stretch = 10  # Stretch x wide enough to fit the variables
x_width = 20  # The width of the x-axis
x_gap = 20  # The gap between left canvas edge and y axis

# A quick for loop to calculate the rectangle
for x, y in enumerate(data):

    # coordinates of each bar

    # Bottom left coordinate
    x0 = x * x_stretch + x * x_width + x_gap

    # Top left coordinates
    y0 = c_height - (y * y_stretch + y_gap)

    # Bottom right coordinates
    x1 = x * x_stretch + x * x_width + x_width + x_gap

    # Top right coordinates
    y1 = c_height - y_gap

    # Draw the bar
    c.create_rectangle(x0, y0, x1, y1, fill="red")

    # Put the y value above the bar
    c.create_text(x0 + 2, y0, anchor=tk.SW, text=str(y))

root.mainloop()