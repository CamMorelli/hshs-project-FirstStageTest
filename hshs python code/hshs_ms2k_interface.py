import tkinter as tk
from tkinter import ttk
import queue
import threading        #   Serial read_from_port operates in a different thread
import time
import serial
import serial.tools.list_ports
import numpy as np
import hshs_constants
import hshs_gui_interface as gui
import platform
import hshs_codec
#import hshs_file_save
import datetime


class MS2KClass():
    """Class for serial port funtions"""

    def __init__(self, controller, gui_obj, ms2k_frame):

        self.controller = controller
        self.gui_obj = gui_obj

        self.ms2k_frame = ms2k_frame

        self.text_q = queue.Queue()

        self.port_initiate = True
        self.port_successfully_started = False
        self.port_menu_selected = None
        self.serial_port = None

        self.controller_display = tk.Listbox(ms2k_frame,height = 18, width = 30)
        self.controller_display.grid(row = 2, column = 2, columnspan = 3, sticky=tk.EW, padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)

        self.controller_z_display = gui.EntryAsDisplay(parent = ms2k_frame,controller = self.controller, row = 1, column = 2, label_name= "Z-Axis:")
        self.controller_x_display = gui.EntryAsDisplay(parent = ms2k_frame,controller = self.controller, row = 1, column = 3, label_name= "X-Axis:")
        self.controller_y_display = gui.EntryAsDisplay(parent = ms2k_frame,controller = self.controller, row = 1, column = 4, label_name= "Y-Axis:")

        self.ms2k_port = None
        self.ports_list = None

        self.port_open = False

        self.port_option_menu = None

        self.initial_command_box_text = "Enter Command Here, press Enter"

        self.response_history = tk.Listbox(ms2k_frame, height=18, width=80)
        self.response_history.grid(row=2, column=0, columnspan = 2, sticky=tk.EW, padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.very_verbose = True
        self.name='MS-2000-500-CP',

    def comm_control_widgets(self, port_frame):
        """Populate widgets in ms2k communication control frame including serial port list"""
        ports_detail_list = [tuple(p) for p in list(serial.tools.list_ports.comports())]   #   Get list of serial ports
        self.ports_list = [p[0] for p in ports_detail_list]
        if hshs_constants.DEBUG: print(f'Found serial ports: {self.ports_list}')
#        settings_heading = gui.SimpleButton(port_frame, self.controller, 0, 0, 'Comm\nControls', 'header font', '', '')
        start_button = gui.SimpleButton(port_frame, self.controller, 0, 1, 'Connect\nMS2k[m]', 'basic font', self.start_port, '')
#        stop_button = gui.SimpleButton(port_frame, self.controller, 0, 2, 'Quit\n[q]', 'basic font', self.controller.quit, '')
        tk.Label(port_frame, text = "Serial Port", font=self.controller.widget_basic_font).grid(row=0, column = 3)
        self.port_menu_selected = tk.StringVar(port_frame)      #   menu variable for port list
        self.port_option_menu = ttk.Combobox(port_frame, textvariable=self.port_menu_selected, postcommand=self.update_option_menu)  #   postcommand updates list before it is shown
        self.port_option_menu.grid(row=1, column = 3, sticky=tk.NSEW, padx=hshs_constants.ENTRY_PAD, pady=hshs_constants.ENTRY_PAD)
        self.port_option_menu.config(width=63)
        self.port_option_menu['values'] = [port for port in self.ports_list]
        self.port_option_menu['state'] = 'readonly'

        cmd_entry = gui.TextEntry(port_frame, self.controller, 0, 6, 35, self.initial_command_box_text, "MS2K Command Entry", "ms2k_command_entry", "MS2K Command Entry")
        cmd_entry.bind('<FocusIn>',lambda x: self.clear_command_box(cmd_entry))
        cmd_entry.bind('<Return>',lambda x: self.send_cmd_entry(cmd_entry.get()))

    def z_stage_control_widgets(self,frame):
        title = tk.Label(frame, text="Z\nControls", font=self.controller.widget_basic_font)
        title.grid(row=0, column=0, rowspan = 1, sticky='nsew')

        # Fine controls
        self.f_up_button = gui.SimpleButton(frame, self.controller, 1, 1, chr(0x2191), 'arrow', self.move_rel, 'zf+')
        self.f_down_button = gui.SimpleButton(frame, self.controller, 3, 1, chr(0x2193), 'arrow', self.move_rel, 'zf-')

        # Rough controls
        self.c_up_button = gui.SimpleButton(frame, self.controller, 0, 1, chr(0x21C8), 'arrow', self.move_rel, 'zc+')
        self.c_down_button = gui.SimpleButton(frame, self.controller, 4, 1, chr(0x21CA), 'arrow', self.move_rel, 'zc-')

        # Curr location
        self.increment_val=gui.NumberEntry(frame, self.controller, 2, 1, 5., 5., 2., 50000., "", "", self.update_increment, 'self')    #TODO solidify parameters. Copied from X range

        # Increment vals
        self.f_z_increment_val = gui.NumberEntry(frame, self.controller, 5, 0, 5., 5., 2., 50000., "Fine\nadjust", "", self.update_increment, 'self')
        self.c_z_increment_val = gui.NumberEntry(frame, self.controller, 5, 1, 5., 5., 2., 50000., "Rough\nadjust", "", self.update_increment, 'self')


    def xy_stage_control_widgets(self, frame):
        title = tk.Label(frame, text="XY\nControls", font=self.controller.widget_basic_font)
        title.grid(row=0, column=0, rowspan = 1, sticky='nsew')
        title2 = tk.Label(frame, text= "                 ", font=self.controller.widget_basic_font)
        title2.grid(row=0, column= 4, rowspan=1, sticky='nsew')
        title3 = tk.Label(frame, text= "                 ", font=self.controller.widget_basic_font)
        title3.grid(row=0, column= 3, rowspan=1, sticky='nsew')
        title4 = tk.Label(frame, text= "                 ", font=self.controller.widget_basic_font)
        title4.grid(row=0, column= 1, rowspan=1, sticky='nsew')

        # Fine controls
        self.f_up_button = gui.SimpleButton(frame, self.controller, 1, 2, chr(0x2191), 'arrow', self.move_rel, 'yf+')
        self.f_down_button = gui.SimpleButton(frame, self.controller, 3, 2, chr(0x2193), 'arrow', self.move_rel, 'yf-')
        self.f_left_button = gui.SimpleButton(frame, self.controller, 2, 1, chr(0x2190), 'arrow', self.move_rel, 'xf-')
        self.f_right_button = gui.SimpleButton(frame, self.controller, 2, 3, chr(0x2192), 'arrow', self.move_rel, 'xf+')

        # Rough controls
        self.c_up_button = gui.SimpleButton(frame, self.controller, 0, 2, chr(0x21C8), 'arrow', self.move_rel, 'yc+')
        self.c_down_button = gui.SimpleButton(frame, self.controller, 4, 2, chr(0x21CA), 'arrow', self.move_rel, 'yc-')
        self.c_left_button = gui.SimpleButton(frame, self.controller, 2, 0, chr(0x21C7), 'arrow', self.move_rel, 'xf-')
        self.c_right_button = gui.SimpleButton(frame, self.controller, 2, 4, chr(0x21C9), 'arrow', self.move_rel, 'xf+')

        # Curr location
        self.increment_val=gui.NumberEntry(frame, self.controller, 2, 2, 5., 5., 2., 50000., "", "", self.update_increment, 'self')    #TODO solidify parameters. Copied from X range

        # Increment vals
        self.f_xy_increment_val = gui.NumberEntry(frame, self.controller, 5, 0, 5., 5., 2., 50000., "Fine\nadjust", "", self.update_increment, 'self')
        self.c_xy_increment_val = gui.NumberEntry(frame, self.controller, 5, 1, 5., 5., 2., 50000., "Rough\nadjust", "", self.update_increment, 'self')

        # Stop loop button.
        self.stop_loop_button = gui.SimpleButton(frame, self.controller,5, 4, text = "Stop loop", font = self.controller.widget_basic_font, command = self.stop_loop, argument= '')

    def update_increment(self):
        pass
#        x = 0
#        y = 0
#        z = 0
#        next_string_list = next_string.split(" ")
#        if len(next_string_list) == 3:
#            new_x = int(next_string_list[0])
#            new_y = int(next_string_list[1])
#            new_z = int(next_string_list[2])
#            if new_x != x or new_y != y or new_z != z:
#                self.controller_x_display.string_var.set((f'{new_x:,}'))
#                self.controller_y_display.string_var.set((f'{new_y:,}'))
#                self.controller_z_display.string_var.set((f'{new_z:,}'))
#                self.controller_x_display.after(1, self.send_to_port("W XYZ"))
#                x = int(next_string_list[0])
#                y = int(next_string_list[1])
#                z = int(next_string_list[2])
#                self.controller_x_display.after_cancel(self.send_to_port)
#            else:
#                pass
#        else:
#            pass

    def clear_command_box(self, obj):
        """Clear command box if contains initial text"""
        if obj.get() == self.initial_command_box_text:
            obj.delete (0, tk.END)

    def update_option_menu(self):
        """Updates list of port options on press of refresh button"""
        ports_detail_list = [tuple(p) for p in list(serial.tools.list_ports.comports())]   #   Get list of serial ports
        self.ports_list = [p[0] for p in ports_detail_list]
        if hshs_constants.DEBUG: print(f'Found new ports list: {self.ports_list}')
        self.port_option_menu['values'] = self.ports_list

    def select_ms2k_port(self, port_name):
        """Select ms2k port from list"""
        if port_name in self.ports_list:
            self.port_menu_selected.set(port_name)
        else:
            other_port = str(next((s for s in self.ports_list if "usbmodem" in s), self.ports_list[-1]))
                            # On mac, port will contain "usbmodem"; last item in list is a good default value
            self.port_menu_selected.set(str(other_port))
    def stop_loop(self):
        if self.stop_loop:
            next_string = self.text_q.get(block=True, timeout=None)
            next_string_list = next_string.split(" ")
            self.stop_loop = False
            if len(next_string_list) == 3:
                x = 0
                y = 0
                z = 0
                new_x = int(next_string_list[0])
                new_y = int(next_string_list[1])
                new_z = int(next_string_list[2])
                if x == new_x and y == new_y and z == new_z:
                    self.controller_x_display.after_cancel(self.send_to_port)

    def update_response_history(self):
        """Updates response history from microcontroller"""
        while self.text_q.qsize() > 0:
            x = 0
            y = 0
            z = 0
            next_string = self.text_q.get(block=True, timeout=None)
            print(f"next_string: {next_string}, type(next_string): {type(next_string)}")
#            print (f'{next_string.find("git version ")}: {next_string.find("git version "), type(next_string.find("git version ")): {type(next_string.find("git version "))})}')
#            self.controller.frames["StageControlPage"].response_history.insert(tk.END, next_string)
#            self.controller.frames["StageControlPage"].response_history.yview(tk.END)
            self.response_history.insert(tk.END, next_string)
            self.response_history.yview(tk.END)
            next_string_list = next_string.split(" ")
            if len(next_string_list) == 3:
                new_x = int(next_string_list[0])
                new_y = int(next_string_list[1])
                new_z = int(next_string_list[2])
                if new_x != x or new_y != y or new_z != z:
                    self.controller_x_display.string_var.set((f'{new_x:,}'))
                    self.controller_y_display.string_var.set((f'{new_y:,}'))
                    self.controller_z_display.string_var.set((f'{new_z:,}'))
                    self.controller_x_display.after(1, self.send_to_port("W XYZ"))
                    self.stop_loop = True
                    x = int(next_string_list[0])
                    y = int(next_string_list[1])
                    z = int(next_string_list[2])
                else:
                    pass
            else:
                pass

            if next_string.find("git version ") > -1:
                self.git_version = next_string.partition("git version ")[2].split(' ')[0]
                if hshs_constants.DEBUG: print (f'ms2k git version: {self.git_version}')
            if next_string.find("over cutoff") > -1:
                gui.warning_message_box_with_focus_return("Error", next_string)
#                self.gui_obj.warning_message_box_with_focus_return("Error", "Servo out of range")
            if next_string.find("not in ready state") > -1:
                gui.warning_message_box_with_focus_return("Error", "Not in ready state")
        if self.controller.call_again:
            self.controller.after(hshs_constants.RESPONSE_HISTORY_INTERVAL, self.update_response_history)
        else:
            if hshs_constants.DEBUG: print("Ending update_response_history")

    def send_cmd_entry(self, cmd_entry):
        """Local method to send command entry to microcontroller from textbox on GUI."""
        self.send_to_port(cmd_entry)
        self.controller.focus()        #   Release key focus

    def move_rel(self, arg):
        print(arg)
        if len(arg) < 3:
            print(f"Invalid argument length: {len(arg)}")
            return
        else:
            pass
        if arg[2] == "+":
            sign = 1
        elif arg[2] == "-":
            sign = -1
        else:
            print(f"Invalid argument sign: {arg[2]}")
            return
        if arg[1] == "f":
            if arg[0] == "x" or arg[0] == "y":
                move_dist = (sign)*(self.f_xy_increment_val.value)
            elif arg[0] == "z":
                move_dist = (sign)*(self.f_z_increment_val.value)
            else:
                print(f"Invalid argument dimension: {arg[0]}")
        elif arg[1] == "c":
            if arg[0] == "x" or arg[0] == "y":
                move_dist = (sign)*(self.c_xy_increment_val.value)
            elif arg[0] == "z":
                move_dist = (sign)*(self.c_z_increment_val.value)
            else:
                print(f'Invalid argument dimension: {arg[0]}')
        else:
            print(f"Invalid argument fine/coarse: {arg[1]}")
            return
        move_dist_str = str(move_dist)
        if arg[0] == "x":
            out_string = "R X=" + move_dist_str + " Y Z"
            print (out_string)
            self.send_to_port(out_string)
        elif arg[0] == "y":
            out_string = "R Y=" + move_dist_str + " X Z"
            print (out_string)
            self.send_to_port(out_string)
        elif arg[0] == "z":
            out_string = "R Z=" + move_dist_str + " X Y" 
            print (out_string)
            self.send_to_port(out_string)
        else:
            print(f'Invalid argument dimension: {arg[0]}')
            

    def send_to_port(self, send_data):
        """Send commands to ms2k."""
        if not send_data:
            print ("Sent Nothing")
        elif not self.port_successfully_started:
            print("In send_to_port")
            self.start_port()
        else:
            print(f"Sending command to port: {send_data}")
            try:    #   Closing thread; shut serial port
                self.serial_port.write(send_data.encode()+b"\r")    #   Write string as collection of bytes, e.g., self.serial_port.write(b'reset_clock\n')

#                if self.very_verbose:
#                    print("%s: sending cmd = "%self.name, send_data)
#                assert type(send_data) is str, 'command should be a string'
#                send_data = bytes(send_data, encoding='ascii')
#                self.serial_port.write(send_data + b'\r')


            except serial.SerialException as error_message:
                print(f'Error writing command: {send_data}; {error_message}')

    def read_from_port(self):
        """Read data from serial port using thread and write to appropriate files when desired."""
        hshs_data_count = 0
        self.run_thread = True
        read_count = 0
#        hshs_response_count = 0
        started_shutdown = False
        while self.run_thread:
            try:
                bytes_to_read = self.serial_port.in_waiting
                if bytes_to_read > 0:

                    response = self.serial_port.readline().decode('ascii').strip(':A \r\n')
                    print (f"response: {response}")
                    if response.find("Version") > -1:
                        if hshs_constants.DEBUG: print ("Communication with ms2k initiated")
                        self.port_initiate = False
                        self.port_successfully_started = True

#                    data = self.serial_port.read(bytes_to_read) # switched from readline()
 #                   print(f"data: {data}")
                    print(f"response: {response}")
#                    for byte in data:
#                        (frame_data, frame_detected) = self.codec_obj.get_frame(byte)
#                        if frame_detected:
 #                           obj = self.codec_obj.decode_frame(frame_data)
  #                          code_flag = frame_data[0]

   #                         if code_flag == 0xFD:   #   Text response
    #                            self.hshs_response_count += 1
     #                           new_string = obj.text.value
#                                self.text_q.put(new_string)          #   Queue text for display
#                    self.text_q.put(data)          #   Queue text for display
                    self.text_q.put(response)          #   Queue text for display
#                                file_queue_element = FileQueueData("Responses", new_string, "")
 #                               self.file_obj.file_data_q.put(file_queue_element)
  #                              if self.port_initiate is True:
   #                                 if hshs_constants.DEBUG: print ("Initiating port", new_string)
    #                                if new_string.find("will process command: [version]") > -1:
     #                                   if hshs_constants.DEBUG: print ("Communication with ms2k initiated")
      #                                  self.port_initiate = False
       #                                 self.port_successfully_started = True
        #                        if self.thread_initiate_close.is_set():   #   Waiting for microcontroller shutdown
         #                           if new_string.find("MS2K shutdown complete")>-1:
          #                              self.run_thread = False         # End thread
           #                             file_queue_element = hshs_file_save.FileQueueAction(None, "CloseFileThread")
            #                            self.file_obj.file_data_q.put(file_queue_element)


                else:
                    time.sleep(0.25)
                        
            except serial.SerialException as error_message:
                self.run_thread = False
                print("Error reading from serial port: ", error_message)
            read_count +=1   #   End while self.run_thread
        try:    #   Closing thread; shut serial port
            if hshs_constants.DEBUG: print("Attempting to close serial port")
            self.serial_port.close()
        except serial.SerialException as error_message:
            print("Error closing thread: ", error_message)
        if hshs_constants.DEBUG: print ("Ending MS-2000 read_from_port thread")

    def start_port(self):
        """Connect serial port and start serial thread."""
        
        if not self.port_successfully_started:
            self.ms2k_port = self.port_menu_selected.get()
            if hshs_constants.DEBUG: print("port = ", self.ms2k_port)
            try:
                if hshs_constants.DEBUG: print("Opening port")
                self.serial_port = serial.Serial(self.ms2k_port, baudrate=115200, timeout=0.5)

#                self.send_to_port('V')
                self.serial_port.write("V".encode()+b"\r")
#                self.serial_port.write(b'V\r')      #   Check ms2k communication
#self.serial_port.write(send_data.encode()+b"\r") 

                self.port_initiate = True
            except serial.SerialException as error_message:
                print("Error opening serial port: ", error_message)
                self.gui_obj.warning_message_box_with_focus_return("Warning", "Serial port not found")
                return
            try:
                if hshs_constants.DEBUG: print("starting thread")
                self.port_thread = threading.Thread(target=self.read_from_port)
                self.port_thread.start()
            except BaseException as error_message:
                print("Error starting thread: ", error_message)
            tic = time.perf_counter()
#            self.port_successfully_started = True    #FIXME - set in read_from_port with response to query
#            while False:
            while not self.port_successfully_started:    #   Wait for confirmation of MS2K communications
                time.sleep(0.1)
                toc = time.perf_counter()
                if toc-tic > 2:
                    print("Timeout waiting for MS2K communication")
                    self.run_thread = False
                    if tk.messagebox.askretrycancel("Warning", "No communications with MS2K"):
                        self.controller.after(1, self.controller.focus_force)     #   Reset focus after messagebox
                        print("Retry")
                        return
                    else:
                        self.controller.after(1, self.controller.focus_force)     #   Reset focus after messagebox
                        print ("stop")
                        self.controller.quit()
                        return
#            self.send_to_port('V')
#            self.version = self._send_mod('V').strip(':A \r\n')
#            print(f"self.version: {self.version}")
#            self.version = self._send_mod('V').strip(':A \r\n')
#            print(f"self.version: {self.version}")
#            self.version = self._send_mod('V').strip(':A \r\n')
#            print(f"self.version: {self.version}")
#            self.version = self._send_mod('V').strip(':A \r\n')
#            print(f"self.version: {self.version}")

#            self.controller.show_frame("DataPage")

#            self.send_to_port('version')
#            self.send_to_port('reset_state')    #   Put ms2k in ready state with pwm and fan off
#            self.send_to_port('reset_clock')
#            self.send_to_port('sensor on')
#            self.send_to_port('lock off')
            
#            self.sensor_status = True
#            self.servo.value = 500  #   Reset servo to center
#            startup_objects = [self.servo, self.crisp_led_level, self.set_point, self.k_prop, self.hshs_diff_time_const, self.acquire_debug_count_interval_obj]
#            for obj in startup_objects:     #   Apply values loaded from config.json to ms2k
#                self.update_ms2k(obj)
#            if self.hshs_direction_obj.value:
#                self.send_to_port('set_pid_direction F')
#            else:
#                self.send_to_port('set_pid_direction R')
#            for obj in self.file_obj.file_objects:       #       Open files as appropriate based on json status
#                if obj.value:       #   In startup, opening files is initiated by value = True
#                    if hshs_constants.DEBUG: print("Queueing file status change", obj.name, obj.value)
#                    self.file_obj.queue_file_status_change(obj)
#            self.send_to_port('zero_cal')


    def _send(self, cmd, respond=True, parse_axes=False):
        if self.very_verbose:
            print("%s: sending cmd = "%self.name, cmd)
        assert type(cmd) is str, 'command should be a string'
        cmd = bytes(cmd, encoding='ascii')
        self.port.write(cmd + b'\r')
        response = self.port.readline().decode('ascii').strip(':A \r\n')
        if respond:
            assert response != '', '%s: No response'%self.name
            if parse_axes:
                axes, values = [], []
                for a in response.split():
                    axis, value = a.split('=')
                    axes.append(axis), values.append(float(value))
                assert tuple(axes) == self.axes
                response = tuple(values)
        else:
            response = None
        if self.very_verbose:
            print("%s: -> response = "%self.name, response)
        assert self.port.in_waiting == 0
        return response

    def _send_mod(self, cmd, respond=True, parse_axes=False):
        if self.very_verbose:
            print("%s: sending cmd = "%self.name, cmd)
        assert type(cmd) is str, 'command should be a string'
        cmd = bytes(cmd, encoding='ascii')
        self.serial_port.write(cmd + b'\r')
        response = self.serial_port.readline().decode('ascii').strip(':A \r\n')
#        if respond:
#            assert response != '', '%s: No response'%self.name
#            if parse_axes:
##                axes, values = [], []
 #               for a in response.split():
 #                   axis, value = a.split('=')
 #                   axes.append(axis), values.append(float(value))
 #               assert tuple(axes) == self.axes
 #               response = tuple(values)
 #       else:
 #           response = None
        if self.very_verbose:
            print("%s: -> response = "%self.name, response)
  #      assert self.port.in_waiting == 0
        return response

