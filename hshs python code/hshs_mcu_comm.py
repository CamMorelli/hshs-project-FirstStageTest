#   serial_port module
"""Module for MCU communication: open, close, read, and write, also queues for thread-safe communication"""

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
import hshs_file_save
import datetime

#   Dependencies on entry parameters, controller (start page, shutdown), file_save

class CRISPQueueUpdate:
    """Defines sample for pid data queue"""
    def __init__(self):
        self.entry_param = None
        self.new_value = 0

class FileQueueData:
    """For general queued elements for file writing"""
    def __init__(self, name, data, action):
        self.name = name
        self.data = data
        self.action = action    #   Use to open and close files from data queue

class SerialPort():
    """Class for serial port funtions"""

    def __init__(self, controller, gui_obj, file_obj, plot_obj, crisp_q, fits_q):

        self.controller = controller
        self.gui_obj = gui_obj
        self.file_obj = file_obj
        self.plot_obj = plot_obj
        self.crisp_q = crisp_q
        self.text_q = queue.Queue()
        self.fits_q = fits_q
        self.crisp_vals_q = queue.Queue()

        self.thread_initiate_close = threading.Event()
        self.port_initiate = True
        self.run_thread = True
        self.port_successfully_started = False
        self.port_menu_selected = None
        self.serial_port = None
        self.port_thread = None
        self.led_on = False
        self.mcu_port = None
        self.ports_list = None
        self.git_version = ""
        self.bidirectional_params = None
        self.num_bidirectional_settings = 3
        self.bidirectional_param_values = np.zeros(self.num_bidirectional_settings)
        self.num_crisp_settings = 4
        self.current_crisp_values = np.zeros(self.num_crisp_settings)
        self.port_option_menu = None
#        self.dither_control_list = None
        self.crisp_led_controls = None
        self.acquire_param_objects = None
        self.acquire_debug_count_interval_obj = None
#        self.prior_updating_entry_parameter_values = [-1, -1, -1]   # List of GUI entry parameter values that can be changes unilaterally by mcu
        self.prior_updating_entry_parameter_values = -1*np.ones(3*hshs_constants.NUM_CHANNELS+1)   # List of GUI entry parameter values that can be changes unilaterally by mcu
        self.set_point = None
        self.last_message_time = -1
        self.state_value = -1
        self.prior_state__value = -2
        self.state_description = ''
        self.initial_command_box_text = "Enter Command Here, press Enter"
        self.hshs_response_count = 0
        self.zero_cal_values = [0, 0]

        self.codec_obj = hshs_codec.Codec(hshs_constants.NUM_CHANNELS)    #   Creates objects and fills attributes

    def comm_control_widgets(self, port_frame):
        """Populate widgets in mcu communication control frame including serial port list"""
        ports_detail_list = [tuple(p) for p in list(serial.tools.list_ports.comports())]   #   Get list of serial ports
        self.ports_list = [p[0] for p in ports_detail_list]
        if hshs_constants.DEBUG: print(f'Found serial ports: {self.ports_list}')
        settings_heading = gui.SimpleButton(port_frame, self.controller, 0, 0, 'Comm\nControls', 'header font', '', '')
        start_button = gui.SimpleButton(port_frame, self.controller, 0, 1, 'Start\n[s]', 'basic font', self.start, '')
        stop_button = gui.SimpleButton(port_frame, self.controller, 0, 2, 'Quit\n[q]', 'basic font', self.controller.quit, '')
        tk.Label(port_frame, text = "Serial Port", font=self.controller.widget_basic_font).grid(row=0, column = 3)
        self.port_menu_selected = tk.StringVar(port_frame)      #   menu variable for port list
        self.port_option_menu = ttk.Combobox(port_frame, textvariable=self.port_menu_selected, postcommand=self.update_option_menu)  #   postcommand updates list before it is shown
        self.port_option_menu.grid(row=1, column = 3, sticky=tk.NSEW, padx=hshs_constants.ENTRY_PAD, pady=hshs_constants.ENTRY_PAD)
        self.port_option_menu.config(width=25)
        self.port_option_menu['values'] = [port for port in self.ports_list]
        self.port_option_menu['state'] = 'readonly'

        cmd_entry = gui.TextEntry(port_frame, self.controller, 0, 6, 35, self.initial_command_box_text, "MCU Command Entry", "mcu_command_entry", "MCU Command Entry")
        cmd_entry.bind('<FocusIn>',lambda x: self.clear_command_box(cmd_entry))
        cmd_entry.bind('<Return>',lambda x: self.send_cmd_entry(cmd_entry.get()))

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

    def select_mcu_port(self, port_name):
        """Select mcu port from list"""
        if port_name in self.ports_list:
            self.port_menu_selected.set(port_name)
        else:
            other_port = str(next((s for s in self.ports_list if "usbmodem" in s), self.ports_list[-1]))
                            # On mac, port will contain "usbmodem"; last item in list is a good default value
            self.port_menu_selected.set(str(other_port))

    def state_controls_widgets(self, state_controls_frame):
        """Populate widgets in state controls frame"""
        #       State controls frame:
        self.control_heading = gui.SimpleButton(state_controls_frame, self.controller, 0, 0, 'State\nControls', 'header font', '', '')
        self.ready_state_obj =  gui.StateButton(state_controls_frame, self.controller, 0, 1, False, False, "Ready\n[r]", "Ready\nIs Off [r]", "Ready Button", \
                                                "", False, self.handle_mcu_command, 'r')
#        self.lock_state_obj =  gui.StateButton(state_controls_frame, self.controller, 0, 2, False, False, "Locking\n[l]", "Lock\nIs Off [l]", "Lock Button", \
#                                                hshs_constants.LOCK_ON_OFF_STATUS, False, self.handle_mcu_command, 'l')
        self.servo_scan_state_obj = gui.StateButton(state_controls_frame, self.controller, 0, 3, False, False, 'Scanning\nServo [n]', 'Servo\nScan Off [n]', 'Servo Scan Button', '', False, self.handle_mcu_command, 'n')
#        self.led_scan_state_obj = gui.StateButton(state_controls_frame, self.controller, 0, 4, False, False, 'Scanning\nLED [c]', 'LED\nScan Off [c]', 'LED Scan Button', '', False, self.handle_mcu_command, 'c')
#        self.dither_state_obj = gui.StateButton(state_controls_frame, self.controller, 0, 5, False, False, 'Dither\nOn [i]', 'Dither\nis Off [i]', 'Dither Button', '', False, self.handle_mcu_command, 'i')
        self.sleep_state_obj =  gui.StateButton(state_controls_frame, self.controller, 0, 6, False, False, "Sleeping\n[d]", "Sleep\nIs Off [d]", "Sleep Button", \
                                                hshs_constants.SENSOR_ON_OFF_STATUS, False, self.handle_mcu_command, 'd')

        self.zero_state_obj =  gui.StateButton(state_controls_frame, self.controller, 0, 7, False, False, "Zeroing\n[d]", "Zero\nIs Off [d]", "Zero Button", \
                                                hshs_constants.SENSOR_ON_OFF_STATUS, False, self.handle_mcu_command, 'z')

#        self.state_obj_list = [self.ready_state_obj, self.lock_state_obj, self.servo_scan_state_obj, self.led_scan_state_obj, self.dither_state_obj, self.sleep_state_obj]
#        self.button_type = {'Ready' : self.ready_state_obj,'Reset' : self.ready_state_obj, 'Data Off' : self.sleep_state_obj, 'Lock Init' : self.lock_state_obj, \
#                       'Lock On' : self.lock_state_obj, 'Scan Init' : self.servo_scan_state_obj, 'Scan On' : self.servo_scan_state_obj, 'LED Scan Init' : self.led_scan_state_obj, \
#                        'LED Scan On' : self.led_scan_state_obj, 'Dither Init' : self.dither_state_obj, 'Dither On' : self.dither_state_obj, 'Shutdown' : self.sleep_state_obj, \
#                        'Zero_Init' : self.zero_state_obj, 'Zero_Cal' : self.zero_state_obj}
        self.button_type = {'Ready' : self.ready_state_obj,'Reset' : self.ready_state_obj, 'Data Off' : self.sleep_state_obj,\
                       'Scan Init' : self.servo_scan_state_obj, 'Scan On' : self.servo_scan_state_obj, \
                        'Shutdown_Init' : self.sleep_state_obj, 'Shutdown' : self.sleep_state_obj,  'Zero_Init' : self.zero_state_obj, 'Zero_Cal' : self.zero_state_obj }
        self.prior_state_obj = self.ready_state_obj

    def laser_controls_widgets(self, laser_settings_frame):
        """Populate widgets in pwm controls frame."""
        pwm_heading = gui.SimpleButton(laser_settings_frame, self.controller, 0, 0, 'Laser\nControls', 'header font', '', '')
        self.laser_pwm_list = [gui.NumberEntry(laser_settings_frame, self.controller, 0, 1+ii, 0., 0., 0., 100., "Laser 0"+str(ii), \
                                        hshs_constants.LASER_PWM_VALUES_CONTROL, self.update_mcu, 'self') for ii in range(hshs_constants.NUM_CHANNELS)]
#        self.tec_pwm_list = [gui.NumberEntry(laser_settings_frame, self.controller, 0, 1+hshs_constants.NUM_CHANNELS+ii, 0., 0., 0., 100., "TEC 0"+str(ii), \
#                                        hshs_constants.TEC_PWM_VALUES_CONTROL, self.update_mcu, 'self') for ii in range(hshs_constants.NUM_CHANNELS)]
#        self.fan=gui.NumberEntry(laser_settings_frame, self.controller, 0, 1+2*hshs_constants.NUM_CHANNELS, 0., 0., 0., 100., "Fan", hshs_constants.FAN_PWM_VALUE_CONTROL, self.update_mcu, 'self')
#        self.pwm_and_fan = self.laser_pwm_list + self.tec_pwm_list + [self.fan]

    def tec_controls_widgets(self, tec_settings_frame):
        pwm_heading = gui.SimpleButton(tec_settings_frame, self.controller, 0, 0, 'TEC\nControls', 'header font', '', '')
        self.tec_pwm_list = [gui.NumberEntry(tec_settings_frame, self.controller, 0, 1+ii, 0., 0., 0., 100., "TEC 0"+str(ii), \
                                        hshs_constants.TEC_PWM_VALUES_CONTROL, self.update_mcu, 'self') for ii in range(hshs_constants.NUM_CHANNELS)]

    def fan_controls_widgets(self, fan_settings_frame):
        pwm_heading = gui.SimpleButton(fan_settings_frame, self.controller, 0, 0, 'Fan\nControls', 'header font', '', '')
        self.fan=gui.NumberEntry(fan_settings_frame, self.controller, 0, 1, 0., 0., 0., 100., "Fan", hshs_constants.FAN_PWM_VALUE_CONTROL, self.update_mcu, 'self')
        self.pwm_and_fan = self.laser_pwm_list + self.tec_pwm_list + [self.fan]


    def temperature_setpoint_widgets(self, temperature_settings_frame):
        """Populate widgets in pwm controls frame."""
        pwm_heading = gui.SimpleButton(temperature_settings_frame, self.controller, 0, 0, 'Temperature\nSet Points', 'header font', '', '')
#        pwm_off_button = gui.SimpleButton(temperature_settings_frame, self.controller, 0, 1, 'PWM All\nOff', 'header font', self.send_to_port, 'pwm off')
#        self.global_pwm_obj = gui.NumberEntry(temperature_settings_frame, self.controller, 0, 2, 0., 0., 0., 100., "PWM ALL", \
#                                        hshs_constants.GLOBAL_PWM_VALUES_CONTROL, self.update_mcu, 'self')
#        self.global_pwm_obj.bind('<FocusOut>',lambda x: self.global_pwm_obj.update_param('0'))    #   Change binding only for global PWM

        self.temperature_setting_list = [gui.NumberEntry(temperature_settings_frame, self.controller, 0, 1+ii, 0., 0., 0., 100., "Temp 0"+str(ii), \
                                        hshs_constants.TEMPERATURE_SETTINGS_CONTROL, self.update_mcu, 'self') for ii in range(hshs_constants.NUM_CHANNELS)]
        self.bidirectional_param_list = self.laser_pwm_list + self.tec_pwm_list + [self.fan] + self.temperature_setting_list


    def temperature_lock_widgets(self, temperature_lock_frame):
        pwm_heading = gui.SimpleButton(temperature_lock_frame, self.controller, 0, 0, 'PID Lock\nControls', 'header font', '', '')
        self.temperature_lock_obj_list = [gui.StateButton(temperature_lock_frame, self.controller, 0, 1+ii, False, False, "Locking 0" + str(ii) + "\n[" + str(ii) + "]", "Lock 0" + str(ii) + "\nIs Off [" + str(ii) + "]", "Lock Button", \
                                                hshs_constants.TEMPERTURE_LOCK_ON_OFF_STATUS, False, self.handle_mcu_command, str(ii))  for ii in range(hshs_constants.NUM_CHANNELS)]

    def crisp_led_controls_widgets(self, crisp_settings_frame):
        """Populate widgets in piezo controls frame."""
#       Entry parameters for CRISP led control:
        crisp_led_heading = gui.SimpleButton(crisp_settings_frame, self.controller, 0, 0, 'CRISP LED\nControls', 'header font', '', '')
        self.crisp_led_level = gui.NumberEntry(crisp_settings_frame, self.controller, 0, 3, 100., 100., 0., 100., "LED Level", hshs_constants.CRISP_LED_VALUE_CONTROL, self.update_mcu, 'self')
        self.crisp_led_scan_time = gui.NumberEntry(crisp_settings_frame, self.controller, 0, 4, 10, 10, 0.1, 1000, 'Scan Time (s)', hshs_constants.CRISP_LED_CONTROLS, '', '')
        self.crisp_led_scan_low = gui.NumberEntry(crisp_settings_frame, self.controller, 0, 5, 0.0, 0.0, 0.00, 100., 'Scan Low', hshs_constants.CRISP_LED_CONTROLS, '', '')
        self.crisp_led_scan_high = gui.NumberEntry(crisp_settings_frame, self.controller, 0, 6, 100., 100., 0.001, 100., 'Scan High', hshs_constants.CRISP_LED_CONTROLS, '', '')


        self.crisp_led_scan_controls = [self.crisp_led_scan_time, self.crisp_led_scan_low, self.crisp_led_scan_high]
        self.crisp_led_controls = [self.crisp_led_level] + self.crisp_led_scan_controls
        crisp_led_scan_defaults_button = gui.SimpleButton(crisp_settings_frame, self.controller, 0, 2, 'Default\nValues', 'basic font', self.write_default_values, self.crisp_led_controls)

    def scan_controls_widgets(self, scan_controls_frame):
        """Populate widgets in scan parameters controls frame."""
#           Entry parameters for scan control:
        scan_controls_heading = gui.SimpleButton(scan_controls_frame, self.controller, 0, 0, 'Servo Scan\nControls', 'header font', '', '')

        self.scan_time = gui.NumberEntry(scan_controls_frame, self.controller, 0, 3, 10, 10, 0.1, 1000, 'Scan Time (s)', hshs_constants.SCAN_PARAMETERS_CONTROL, '', '')
        self.scan_range = gui.NumberEntry(scan_controls_frame, self.controller, 0, 4, 0.1, 0.1, 0.001, 1000, 'Scan Range', hshs_constants.SCAN_PARAMETERS_CONTROL, '', '')

        self.scan_controls = [self.scan_time, self.scan_range]
        scan_defaults_button = gui.SimpleButton(scan_controls_frame, self.controller, 0, 2, 'Default\nValues', 'basic font', self.write_default_values, self.scan_controls)
            
    def acquire_parameters_widgets(self, acquire_parameters_frame):
        """Populate widgets in acquire parameters frame."""
#           Entry parameters for acquire parameters:

        self.acquire_debug_count_interval_obj=gui.NumberEntry(acquire_parameters_frame, self.controller, 0, 3, 200, 200, 0, 2000., "Debug Count", hshs_constants.DEBUG_COUNT_VALUE, self.update_mcu, 'self')     #   Define pid control entry boxes

        reset_clock_button = gui.SimpleButton(acquire_parameters_frame, self.controller, 0, 4, 'Reset\nClock [k]', 'basic font', self.handle_mcu_command, 'k')

        self.acquire_param_objects = [self.acquire_debug_count_interval_obj]
#       Acquire parameters frame:
        acquire_heading = gui.SimpleButton(acquire_parameters_frame, self.controller, 0, 0, 'Acquire\nSettings', 'header font', '', '')

        acquire_default_params_button = gui.SimpleButton(acquire_parameters_frame, self.controller, 0, 2, 'Default\nValues', 'basic font', self.write_default_values, self.acquire_param_objects)

    def hshs_displays_widgets(self, value_displays_frame):
        """Populate crisp control widgets"""
        hshs_value_heading = gui.SimpleButton(value_displays_frame, self.controller, 0, 0, 'Laser\nParams', 'header font', '', '')

        self.summed_laser_current_obj=gui.EntryAsDisplay(value_displays_frame, self.controller, 2, 0, "Laser Current")     #   Define temperature display boxes
        self.summed_tec_current_obj=gui.EntryAsDisplay(value_displays_frame, self.controller, 4, 0, "TEC Current")     #   Define temperature display boxes
        self.temperatures_obj_list = [gui.EntryAsDisplay(value_displays_frame, self.controller, 2*ii+6, 0, "Temp 0"+str(ii)) for ii in range(hshs_constants.NUM_CHANNELS)]

    def feedback_controls_widgets(self, feedback_controls_frame):
        feedback_heading = gui.SimpleButton(feedback_controls_frame, self.controller, 0, 0, 'Feedback\nControls', 'header font', '', '')

        self.servo = gui.NumberEntry(feedback_controls_frame, self.controller, 0, 1, 0., 0., 0., 1000., "Servo", hshs_constants.SERVO_VALUE_CONTROL, self.update_mcu, 'self')
        center_servo_button = gui.SimpleButton(feedback_controls_frame, self.controller, 0, 2, 'Center\nServo [t]', 'basic font', self.handle_mcu_command, 't')
        update_set_point_button = gui.SimpleButton(feedback_controls_frame, self.controller, 0, 4, 'Update Set\nPoint [u]', 'basic font', self.handle_mcu_command, 'u')
        self.set_point = gui.NumberEntry(feedback_controls_frame, self.controller, 0, 5, 0., 0., -1000., 1000., "Set Point", hshs_constants.SET_POINT_CONTROL, self.update_mcu, 'self')
        self.feedback_control_list = [self.set_point]
        self.pd_diff_meas_obj_2=gui.EntryAsDisplay(feedback_controls_frame, self.controller, 0, 6, "PD Diff")     #   Define temperature display boxes
###        self.lock_status_text = tk.StringVar()
###        self.lock_status_text.set("Lock\nOff")
###        self.lock_status = tk.Entry(feedback_controls_frame, width=hshs_constants.ENTRY_WIDTH, textvariable=self.lock_status_text, font=self.controller.widget_header_font)
###        self.lock_status.grid(row = 0, column=7, rowspan = 2, sticky='nsew')

#    def dither_controls_widgets(self, dither_controls_frame):
#        dither_controls_heading = gui.SimpleButton(dither_controls_frame, self.controller, 0, 0, 'Dither\nControls', 'header font', '', '')
#        self.hop_value = gui.NumberEntry(dither_controls_frame, self.controller, 0, 3, 10., 10., .1, 500., "Dither Hop Value", hshs_constants.SET_POINT_CONTROL, '', '')
#        self.dither_control_list = [self.hop_value]
#        dither_defaults_button = gui.SimpleButton(dither_controls_frame, self.controller, 0, 2, 'Default\nValues', 'basic font', self.write_default_values, self.dither_control_list)

    def hshs_controls_widgets(self, hshs_controls_frame):
        """Populate widgets in pid controls frame."""
        hshs_heading = gui.SimpleButton(hshs_controls_frame, self.controller, 0, 0, 'PID\nParameters', 'header font', '', '')

        self.k_prop = gui.NumberEntry(hshs_controls_frame, self.controller, 0, 4, 1., 1., -100, 100., "K Prop", hshs_constants.HSHS_GAIN_CONTROLS, self.update_mcu, 'self')
        self.k_integ = gui.NumberEntry(hshs_controls_frame, self.controller, 0, 5, 4., 4., -100., 100., "K Integ", hshs_constants.HSHS_GAIN_CONTROLS, self.update_mcu, 'self')
        self.k_deriv = gui.NumberEntry(hshs_controls_frame, self.controller, 0, 6, 0., 0., -100., 100., "K Deriv", hshs_constants.HSHS_GAIN_CONTROLS, self.update_mcu, 'self')
        self.hshs_diff_time_const = gui.NumberEntry(hshs_controls_frame, self.controller, 0, 7, 0., 0., 0., 100., "Diff Time Const", hshs_constants.HSHS_DIFF_TIME_CONSTANT, self.update_mcu, 'self')

        self.hshs_gains_list = [self.k_prop, self.k_integ, self.k_deriv]
        self.hshs_control_list = [self.k_prop, self.k_integ, self.k_deriv, self.hshs_diff_time_const]

        self.hshs_direction_obj =  gui.ToggleButton(hshs_controls_frame, self.controller, 0, 9, True, True, "Forward [v]", "Reverse [v]", "PID Direction", \
                                                hshs_constants.HSHS_DIRECTION_CONTROL, True, self.handle_mcu_command, 'v')

        self.hshs_control_list_extended = self.hshs_control_list + [self.hshs_direction_obj]
        hshs_defaults_button = gui.SimpleButton(hshs_controls_frame, self.controller, 0, 2, 'Default\nValues', 'basic font', self.write_default_values, self.hshs_control_list_extended)

    def update_crisp_controls_gui(self):
        """Updates parameters with queued values coming as pid data from mcu"""
        while self.crisp_vals_q.qsize() > 0:
            next_update = self.crisp_vals_q.get(block=True, timeout=None)
            entry_param  = next_update.entry_param
            new_value = next_update.new_value
#            print("updating", entry_param, new_value)
            if not entry_param.focus_on:
                entry_param.update_param(new_value)
        if self.controller.call_again:
            self.controller.after(hshs_constants.RESPONSE_HISTORY_INTERVAL, self.update_crisp_controls_gui)
        else:
            print("Ending update_crisp_controls_gui")

    def update_response_history(self):
        """Updates response history from microcontroller"""
        while self.text_q.qsize() > 0:
            next_string = self.text_q.get(block=True, timeout=None)
            self.controller.frames["SettingsPage"].response_history.insert(tk.END, next_string)
            self.controller.frames["SettingsPage"].response_history.yview(tk.END)
            if next_string.find("git version ") > -1:
                self.git_version = next_string.partition("git version ")[2].split(' ')[0]
                if hshs_constants.DEBUG: print (f'mcu git version: {self.git_version}')
            if next_string.find("over cutoff") > -1:
                self.gui_obj.warning_message_box_with_focus_return("Error", next_string)
#                self.gui_obj.warning_message_box_with_focus_return("Error", "Servo out of range")
            if next_string.find("not in ready state") > -1:
                self.gui_obj.warning_message_box_with_focus_return("Error", "Not in ready state")

        if self.controller.call_again:
            self.controller.after(hshs_constants.RESPONSE_HISTORY_INTERVAL, self.update_response_history)
        else:
            if hshs_constants.DEBUG: print("Ending update_response_history")

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
                    data = self.serial_port.read(bytes_to_read) # switched from readline()
                    for byte in data:
                        (frame_data, frame_detected) = self.codec_obj.get_frame(byte)
                        if frame_detected:
                            obj = self.codec_obj.decode_frame(frame_data)
                            code_flag = frame_data[0]

                            if code_flag == 0xFC:   #   Fit data from Teensy
                                obj.type = obj.transition_type.value%100  #   Jump data has offset of 100 - not used at this time
                                if obj.transition_type.value < 100:
                                    self.fits_q.put(obj)
                                fit_param_file_list = [obj.transition_type.value, hshs_constants.STATE_LIST[obj.transition_type.value%100], \
                                    obj.first_point.value, obj.num_points.value, obj.slopes.value, obj.intercepts.value, \
                                    obj.debug_data.value, obj.offset.value]
                                fit_param_file_string = ",".join([str(item) for item in fit_param_file_list])
                                file_queue_element = FileQueueData("Fit Data", fit_param_file_string, "")    #   Queue works properly when FileQueueData class is instantiated here
                                self.file_obj.file_data_q.put(file_queue_element)
                                if obj.type == 13:
#                                    print(f'Zero_Cal values: {obj.slopes.value}, {obj.intercepts.value}')
                                    self.zero_cal_values = [obj.slopes.value, obj.intercepts.value]
                                    if hshs_constants.DEBUG: print(f'Zero_Cal values: {self.zero_cal_values}')

                            if code_flag == 0xFD:   #   Text response
                                self.hshs_response_count += 1
                                new_string = obj.text.value
                                self.text_q.put(new_string)          #   Queue text for display
                                file_queue_element = FileQueueData("Responses", new_string, "")
                                self.file_obj.file_data_q.put(file_queue_element)
                                if self.port_initiate is True:
                                    if hshs_constants.DEBUG: print ("Initiating port", new_string)
                                    if new_string.find("will process command: [version]") > -1:
                                        if hshs_constants.DEBUG: print ("Communication with MCU initiated")
                                        self.port_initiate = False
                                        self.port_successfully_started = True
                                if self.thread_initiate_close.is_set():   #   Waiting for microcontroller shutdown
                                    if new_string.find("MCU shutdown complete")>-1:
                                        self.run_thread = False         # End thread
                                        file_queue_element = hshs_file_save.FileQueueAction(None, "CloseFileThread")
                                        self.file_obj.file_data_q.put(file_queue_element)


                            elif code_flag == 0xFE:   #   PID data
                                hshs_data_count += 1
#                                print(obj.laser_pwm.value, type(obj.laser_pwm.value))
#                                print(obj.tec_pwm.value, type(obj.tec_pwm.value))
#                                print(obj.fan_pwm.value, type(obj.fan_pwm.value))

#                                combined_data = np.concatenate([obj.laser_pwm.value, obj.tec_pwm.value, np.array([obj.fan_pwm.value])])
                                combined_data = np.concatenate([obj.laser_pwm.value, obj.tec_pwm.value, np.array([obj.fan_pwm.value]), obj.temperature_set_points.value])
#                                print (combined_data, type(combined_data))
                                for i in range(len(combined_data)):   #   Check whether pwm values have changed; if so, queue updates
                                    if self.prior_updating_entry_parameter_values[i] != combined_data[i] :
                                        pwm_update=CRISPQueueUpdate()
#                                        pwm_update.entry_param = self.pwm_and_fan[i]
                                        pwm_update.entry_param = self.bidirectional_param_list[i]
                                        pwm_update.new_value = combined_data[i]
                                        self.crisp_vals_q.put(pwm_update)
                                        self.prior_updating_entry_parameter_values[i] = combined_data[i]

#                                for i, obj_attr, entry_param in zip(range(3), [obj.laser_pwm, obj.tec_pwm.tolist(), obj.fan_pwm], [self.laser_pwm_list, self.tec_pwm_list, self.fan ]):  # Matching list of codec attributes and display entry parameters
#                                    if self.prior_updating_entry_parameter_values[i] != obj_attr.value and self.controller.gui_obj.focus_in_entry_param != entry_param:
#                                        hshs_value_update=CRISPQueueUpdate()
#                                        hshs_value_update.entry_param = entry_param   #   List of entryparam GUI attributes
#                                        hshs_value_update.new_value = obj_attr.value
#                                        self.crisp_vals_q.put(hshs_value_update)
#                                        self.prior_updating_entry_parameter_values[i] = obj_attr.value
                                self.crisp_q.put(obj)        #   Next queue crisp data for saving to file:
                                hshs_param_file_list = [obj.time.value, obj.state_time.value, obj.count.value, obj.state_count.value] + \
                                    [obj.state.value%100, obj.state.value//100, hshs_constants.STATE_LIST[obj.state.value%100]] + \
                                        obj.laser_pwm.value.tolist() + obj.tec_pwm.value.tolist() + [obj.fan_pwm.value] + \
                                            obj.laser_temperatures.value.tolist() + obj.temperature_set_points.value.tolist() + \
                                                [obj.laser_current.value, obj.tec_current.value]    #   List of values to save to file
#                                        obj.pd_1_value.value, obj.pd_2_value.value, obj.pd_diff_meas.value, obj.pd_diff_set_point.value, \
#                                        + obj.error_value.value, obj.average_error.value, obj.servo_value.value, obj.crisp_led_value.value ]
                                hshs_param_file_string = ",".join([str(item) for item in hshs_param_file_list]) #   Now converted to comma separated values string
                                file_queue_element = FileQueueData("CRISP Data", hshs_param_file_string, "")
                                self.file_obj.file_data_q.put(file_queue_element)

                                if self.port_successfully_started:  #   Had problems with following unless port is started
                                    self.state_value = obj.state.value%100
                                    if self.state_value != self.prior_state__value:
                                        if hshs_constants.DEBUG: print(f'New state {self.state_value}, {hshs_constants.STATE_LIST[self.state_value]}')
                                        self.state_description = hshs_constants.STATE_LIST[obj.state.value%100]
                                        self.state_obj = self.button_type[self.state_description]
                                        self.state_obj.update_param(True)
                                        self.prior_state_obj.update_param(False)
                                        if hshs_constants.DEBUG: print(f'New state button: {self.state_obj.name}, type: {type(self.state_obj)}, prior state: {self.prior_state_obj.name}')
                                        self.prior_state__value = self.state_value
                                        self.prior_state_obj = self.state_obj

                                    if (hshs_data_count % 2) == 0:      #   Update current values to front panel, but not on every data point

                                        self.summed_laser_current_obj.string_var.set(obj.laser_current.value)
#                                        print(obj.laser_current.value)
                                        self.summed_tec_current_obj.string_var.set(obj.tec_current.value)
                                        for ii in range(hshs_constants.NUM_CHANNELS):
                                            self.temperatures_obj_list[ii].string_var.set(obj.laser_temperatures.value[ii])

#                                        self.temper_1_obj.string_var.set(obj.pd_2_value.value)
#                                        self.temper_2_obj.string_var.set(obj.)
#                                        self.temper_3_obj.string_var.set(obj.pd_diff_meas.value)
#                                        self.temper_4_obj.string_var.set(obj.pd_diff_set_point.value)
#                                        self.servo_value_obj.string_var.set(obj.servo_value.value)
#                                        self.pd_diff_meas_obj_2.string_var.set(obj.pd_diff_meas.value)
#                                        self.error_value_obj.string_var.set(obj.error_value.value)
#                                        self.average_error_obj.string_var.set(obj.average_error.value)

###
###                                        if (obj.servo_value.value < 0 or obj.servo_value.value > 990) and hshs_data_count > 5:   #   Ignore possible error from prior run
###                                            if time.time()-self.last_message_time > hshs_constants.SERVO_OVERRANGE_DELAY:
###                                                print (time.time(), time.time()-self.last_message_time)
###                                                self.last_message_time = time.time()
###                                                self.gui_obj.warning_message_box_with_focus_return("Error", "Servo out of range")
###                                                self.controller.after(1, self.controller.focus_force)     #   Reset focus after messagebox
###                                        if self.state_description == "Lock On":  #   Using button for lock_status did not allow changing background color on mac
###                                            absolute_error = abs(obj.average_error.value)
###                                            absolute_error = min(hshs_constants.LOCK_HIGH_REPORT_THRESHOLD, absolute_error)
###                                            absolute_error = max(hshs_constants.LOCK_LOW_REPORT_THRESHOLD, absolute_error)
###                                            red_level = int(255*(absolute_error - hshs_constants.LOCK_LOW_REPORT_THRESHOLD)/(hshs_constants.LOCK_HIGH_REPORT_THRESHOLD-hshs_constants.LOCK_LOW_REPORT_THRESHOLD))
###                                            green_level = int(255*(hshs_constants.LOCK_HIGH_REPORT_THRESHOLD - absolute_error)/(hshs_constants.LOCK_HIGH_REPORT_THRESHOLD-hshs_constants.LOCK_LOW_REPORT_THRESHOLD))
#                                            red_level = abs(obj.average_error.value)*hshs_constants.LOCK_HIGH_REPORT_THRESHOLD
#                                            red_level = min(red_level, 255)
#                                            red_level = int(max(red_level, 0))
#                                            color_string = f'#{red_level:02x}ff00'
###                                            color_string = f'#{red_level:02x}{green_level:02x}00'
###                                            if hshs_constants.DEBUG: print(f'absolute error: {absolute_error}, red level: {red_level}, green level: {green_level}, color string: {color_string}')
###                                            self.lock_state_obj.configure(bg = color_string)   #   Set color based on distance from set point
###                                            if (abs(obj.average_error.value)) < hshs_constants.LOCK_LOW_REPORT_THRESHOLD:     #   Show locking/lock status in GUI
                                                #                self.servo.entry_name.configure(state=tk.DISABLED)     #   This disables both entry and updates
###                                                self.lock_status_text.set("Lock\nOn")
###                                                self.lock_status.configure(bg = '#008B00')      # 008B00 is green4 of 
#                                                self.lock_state_obj.string_var.set("Locked [l]")   #   Set color based on distance from set point
#                                                self.lock_state_obj.string_var.set("Locked [l]")   #   Set color based on distance from set point
#                                                self.lock_state_obj.delete (1.0, tk.END)
#                                                self.lock_state_obj.insert(1.0, "Locked [l]")
###                                                self.lock_state_obj.add_text("Locked\n[l]")
                                                ###
###                                            else:
###                                                self.lock_status_text.set("Locking")
###                                                self.lock_status.configure(bg = '#8B0000')
#                                                self.lock_state_obj.string_var.set("Locking [l]")   #   Set color based on distance from set point
#                                                self.lock_state_obj.delete (1.0, tk.END)
#                                                self.lock_state_obj.insert(1.0, "Locking [l]")
###                                                self.lock_state_obj.add_text("Locking\n[l]")
###                                        else:
###                                            self.lock_status_text.set("Lock\nOff")
###                                            self.lock_status.configure(bg = '#0000FF')
###                                            self.lock_status.configure(fg = '#FFFFFF')
                                                
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
        if hshs_constants.DEBUG: print ("Ending read_from_port thread")

    def start(self):
        """Connect serial port and start serial thread."""
        
        if not self.port_successfully_started:
            self.mcu_port = self.port_menu_selected.get()
            if hshs_constants.DEBUG: print("port = ", self.mcu_port)
            try:
                if hshs_constants.DEBUG: print("Opening port")
                self.serial_port = serial.Serial(self.mcu_port, timeout=0.5)
                self.serial_port.write(b'version\n')      #   Check mcu communication
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
            while not self.port_successfully_started:    #   Wait for confirmation of MCU communications
                time.sleep(0.1)
                toc = time.perf_counter()
                if toc-tic > 2:
                    print("Timeout waiting for MCU communication")
                    self.run_thread = False
                    if tk.messagebox.askretrycancel("Warning", "No communications with MCU"):
                        self.controller.after(1, self.controller.focus_force)     #   Reset focus after messagebox
                        print("Retry")
                        return
                    else:
                        self.controller.after(1, self.controller.focus_force)     #   Reset focus after messagebox
                        print ("stop")
                        self.controller.quit()
                        return
            self.controller.show_frame("DataPage")

            self.send_to_port('version')
            self.send_to_port('reset_state')    #   Put mcu in ready state with pwm and fan off
            self.send_to_port('reset_clock')
            self.send_to_port('sensor on')
            self.send_to_port('lock off')
            
            self.sensor_status = True
            self.servo.value = 500  #   Reset servo to center
            startup_objects = [self.servo, self.crisp_led_level, self.set_point, self.k_prop, self.hshs_diff_time_const, self.acquire_debug_count_interval_obj]
            for obj in startup_objects:     #   Apply values loaded from config.json to mcu
                self.update_mcu(obj)
            if self.hshs_direction_obj.value:
                self.send_to_port('set_pid_direction F')
            else:
                self.send_to_port('set_pid_direction R')
            for obj in self.file_obj.file_objects:       #       Open files as appropriate based on json status
                if obj.value:       #   In startup, opening files is initiated by value = True
                    if hshs_constants.DEBUG: print("Queueing file status change", obj.name, obj.value)
                    self.file_obj.queue_file_status_change(obj)
            self.send_to_port('zero_cal')

    def send_to_port(self, send_data):
        """Send commands to mcu."""
        if not self.port_successfully_started: # haven't started up
            self.gui_obj.warning_message_box_with_focus_return("Warning", "Click start on settings page to enable this function")
        else:
            if not send_data:
                print ("Sent Nothing")
            else:
                try:    #   Closing thread; shut serial port
                    self.serial_port.write(send_data.encode()+b"\n")    #   Write string as collection of bytes, e.g., self.serial_port.write(b'reset_clock\n')
                except serial.SerialException as error_message:
                    print(f'Error writing command: {send_data}; {error_message}')

    def send_cmd_entry(self, cmd_entry):
        """Local method to send command entry to microcontroller from textbox on GUI."""
        self.send_to_port(cmd_entry)
        self.controller.focus()        #   Release key focus

    def write_default_values(self, objs):
        """Local method to write default parameter values and update selected objects on mcu.
         self.vis_roi_param_objects, self.roi_param_objects, self.cycle_param_objects are only written during command."""
        for obj in objs:
            obj.update_param(obj.default)
        if objs is self.acquire_param_objects:
            for obj in objs:
                self.update_mcu(obj)

    def update_mcu(self, _entry_param):
        """Update appropriate mcu value based on category of GUI changes. Generally this is reached when NumberEntry values are changed."""
        if hshs_constants.DEBUG: print(f'Handling update_mcu for {_entry_param.name} with category {_entry_param.category}')


        if _entry_param.category == hshs_constants.LASER_PWM_VALUES_CONTROL:  #   For crisp led, update values on mcu
            mcu_cmd = 'laser ' + _entry_param.label_name[-1] + ' ' + str(_entry_param.value)
#            print (_entry_param.label_name, _entry_param.label_name[-1])
        elif _entry_param.category == hshs_constants.TEC_PWM_VALUES_CONTROL:  #   For crisp led, update values on mcu
            mcu_cmd = 'tec ' + _entry_param.label_name[-1] + ' ' + str(_entry_param.value)
        elif _entry_param.category == hshs_constants.FAN_PWM_VALUE_CONTROL:  #   For crisp led, update values on mcu
            mcu_cmd = 'fan ' + str(_entry_param.value)
        elif _entry_param.category == hshs_constants.TEMPERATURE_SETTINGS_CONTROL:  #   For crisp led, update values on mcu
            mcu_cmd = 'temper ' + _entry_param.label_name[-1] + ' ' + str(_entry_param.value)

        elif _entry_param.category == hshs_constants.CRISP_LED_VALUE_CONTROL:  #   For crisp led, update values on mcu
            mcu_cmd = 'crisp_led ' + str(_entry_param.value)
        elif _entry_param.category == hshs_constants.SERVO_VALUE_CONTROL:  #   For servo, update values on mcu
            mcu_cmd = 'servo_level ' + str(_entry_param.value)
        elif _entry_param.category == hshs_constants.SET_POINT_CONTROL:  #   For set point, update values on mcu
            mcu_cmd = 'set_set_point '+ str(_entry_param.value)
        elif _entry_param.category == hshs_constants.HSHS_DIFF_TIME_CONSTANT:  #   For pid differential time constant, update values on mcu
            mcu_cmd = 'set_pid_diff_time_const ' + str(_entry_param.value)
        elif _entry_param.category == hshs_constants.HSHS_GAIN_CONTROLS:  #   For pid gain controls, update gain values on mcu
            mcu_cmd = 'set_pid_gain ' + " ".join([str(obj.value) for obj in self.hshs_gains_list])
        elif _entry_param.category == hshs_constants.DEBUG_COUNT_VALUE:  #   Update Debug Count on mcu
            mcu_cmd = 'debug_interval ' + str(self.acquire_debug_count_interval_obj.value)
        else:
            print(f'Unrecognized category: {_entry_param.category}')
            mcu_cmd = ''
        if mcu_cmd != '':
            if hshs_constants.DEBUG: print(mcu_cmd)
            self.send_to_port(mcu_cmd)

    def handle_mcu_command(self, shortcut):
        """Takes keyboard shortcuts and executes command to mcu. Generally, this is reached when buttons are clicked or keyboard shortcuts are used."""
        if hshs_constants.DEBUG: print(f'Received shortcut: {shortcut}')
        if not self.port_successfully_started: # haven't started up, show warning message
            self.gui_obj.warning_message_box_with_focus_return("Warning", "Click start on settings page to enable this function")
        else:
            if hshs_constants.DEBUG: print('Shortcut: ', shortcut)
#            if shortcut == 'c':                         #   Start LED power scan
#                mcu_cmd = 'led_scan ' + " ".join([str(obj.value) for obj in self.crisp_led_scan_controls]) if not self.led_scan_state_obj.value else 'reset_state'
            if shortcut == 'd':                       #   Toggle data/sensor on and off
                mcu_cmd = 'sensor off' if not self.sleep_state_obj.value else 'reset_state'
#            elif shortcut == 'i':                       #   Start servo dither action
#                mcu_cmd = "dither " + str(self.hop_value.value) if not self.dither_state_obj.value else 'reset_state'
            elif shortcut == 'k':                       #   Reset clock on MCU
                mcu_cmd = 'reset_clock'
#            elif shortcut == "l":                       #   Toggle lock state on and off
#                mcu_cmd = 'lock on' if not self.lock_state_obj.value else 'reset_state'


            elif shortcut == "0":                       #   Toggle lock state on and off
                mcu_cmd = 'lock 0 on' if not self.temperature_lock_obj_list[0].value else 'lock 0 off'
                self.temperature_lock_obj_list[0].toggle()
            elif shortcut == "1":                       #   Toggle lock state on and off
                mcu_cmd = 'lock 1 on' if not self.temperature_lock_obj_list[1].value else 'lock 1 off'
                self.temperature_lock_obj_list[1].toggle()
            elif shortcut == "2":                       #   Toggle lock state on and off
                mcu_cmd = 'lock 2 on' if not self.temperature_lock_obj_list[2].value else 'lock 2 off'
                self.temperature_lock_obj_list[2].toggle()
            elif shortcut == "3":                       #   Toggle lock state on and off
                mcu_cmd = 'lock 3 on' if not self.temperature_lock_obj_list[3].value else 'lock 3 off'
                self.temperature_lock_obj_list[3].toggle()

            elif shortcut == 'n':                       #   Start servo scan
                mcu_cmd = 'scan ' + " ".join([str(obj.value) for obj in self.scan_controls]) if not self.servo_scan_state_obj.value else 'reset_state'
            elif shortcut == 'r':                       #   Reset state on MCU
                mcu_cmd = 'reset_state' if not self.ready_state_obj.value else ''
#            elif shortcut == 't':                       #   Set servo to center position (500)
#                mcu_cmd = 'servo_level 500'
#            elif shortcut == 'u':                       #   Update set point
#                mcu_cmd = 'update_set_point'
            elif shortcut == 'v':                       #   Toggle PID feedback direction
                self.hshs_direction_obj.toggle()
                mcu_cmd = 'set_pid_direction F' if self.hshs_direction_obj.value else 'set_pid___direction R'
            elif shortcut == 'z':                       #   Zero photodiodes
                mcu_cmd = 'zero_cal'
            else:
                print('Unrecognized command')
                mcu_cmd = ''
            if mcu_cmd != '':
                if hshs_constants.DEBUG: print(mcu_cmd)
                self.send_to_port(mcu_cmd)
