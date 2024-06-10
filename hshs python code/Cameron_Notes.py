numbers = "111 222 333"
numbers_list = numbers.split(" ")
x = numbers_list[0]
y = numbers_list[1]
z = numbers_list[2]

if x.isnumeric() and y.isnumeric() and z.isnumeric():
    print(f'x: {x}, y: {y}, z: {z}')
else:
    print(f'no')



self.controller_z_display = gui.EntryAsDisplay(parent = ms2k_frame,controller = self.controller, row = 1, column = 2, label_name= "Z-Axis:")
self.controller_x_display = gui.EntryAsDisplay(parent = ms2k_frame,controller = self.controller, row = 1, column = 3, label_name= "X-Axis:")
self.controller_y_display = gui.EntryAsDisplay(parent = ms2k_frame,controller = self.controller, row = 1, column = 4, label_name= "Y-Axis:")
self.controller_x_display.string_var.set(self.update_response_history())



self.controller_x_display.string_var.set(str(self.update_incrementy()))
self.controller_x_display.after(5000, self.update_increment)

next_string = self.text_q.get(block=True, timeout=None)
next_string_list = next_string.split(" ")
if len(next_string_list) >= 3:
    x = next_string_list[0]
    y = next_string_list[1]
    z = next_string_list[2]
    if x.isdigit() and y.isdigit() and z.isdigit():
        return x
    




    self.controller_x_display.after_cancel(self.send_to_port("W XYZ"))