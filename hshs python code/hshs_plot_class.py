#   plot module
"""Class for line plots and images"""

import tkinter as tk
import queue
#import math
import bisect
import numpy as np
from matplotlib import cm   #   Import colormap
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import hshs_constants
import hshs_gui_interface as gui

class PlotClass():
    """Class for data page"""

    def __init__(self, controller, display_state_frame, displays_frame, plot_controls_frame, fit_data_frame):
        self.controller = controller

        self.axis_limits_q = queue.Queue()  #   Queue to receive axis limits updates from entry parameter widgets
        self.num_plot_lines = 4*hshs_constants.NUM_CHANNELS + 1 + 2

        self.pd_1_data = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
        self.pd_2_data = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
        self.pd_diff_data = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
        self.set_point_data = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
        self.error_level_data = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
        self.average_error_data = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
        self.servo_data = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
#        self.time_array = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
#        self.counter_array = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
        self.crisp_led_array = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)

        self.laser_current_array = np.nan*np.ones((hshs_constants.NUM_CHANNELS, hshs_constants.PLOT_DATA_SIZE))
        self.tec_current_array = np.nan*np.ones((hshs_constants.NUM_CHANNELS, hshs_constants.PLOT_DATA_SIZE))
        self.temperatures_array = np.nan*np.ones((hshs_constants.NUM_CHANNELS*2, hshs_constants.PLOT_DATA_SIZE))

#        self.plot_data_array = np.nan*np.ones(hshs_constants.NUM_CHANNELS*4, hshs_constants.PLOT_DATA_SIZE)
        
        self.time_array = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
        self.y__array_new = np.nan*np.ones((self.num_plot_lines, hshs_constants.PLOT_DATA_SIZE))
        self.counter_array = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)

        self.time_array_short = np.array([0,0])
        self.zero_array_short = np.array([0,0])


        self.data_array_list = [self.pd_1_data, self.pd_2_data, self.pd_diff_data, self.set_point_data, self.error_level_data, self.average_error_data, self.servo_data, \
                                self.time_array, self.counter_array, self.crisp_led_array]
#        self.data_array_list_new = 
        

        self.attribute_list = ['pd_1_value', 'pd_2_value', 'pd_diff_meas', 'pd_diff_set_point', 'error_value', 'average_error', 'servo_value', 'time', 'count', 'crisp_led_value']
        self.axis_number_list = [0, 0, 1, 1, 2, 2, 3]
        self.line_color_list = ['r', 'g','r', 'g','r', 'g','r']

        self.array_index = 0
        self.laststate = -1      #   To track when state value has changed
        self.colors = ['r', 'g', 'y', 'm', 'k', 'c', 'tab:orange', 'tab:purple', 'tab:gray', \
            'tab:brown', 'tab:blue', 'tab:red', 'tab:green', 'tab:pink', 'tab:olive', 'tab:cyan']
        self.data_lines = [None] * hshs_constants.NUM_FIT_PLOTS  #    For fit plots - data
        self.fit_lines = [None] * hshs_constants.NUM_FIT_PLOTS #    For fit plots - fits

        self.x_range_obj=gui.NumberEntry(plot_controls_frame, self.controller, 2, 0, 5., 5., 2., 50000., "X Range", "Plot Parameters", self.update_plot_limits, 'self')     #   Define crisp control entry boxes
        self.y_top_low_obj=gui.NumberEntry(plot_controls_frame, self.controller, 4, 0, 20., 20., -10., 200., "Laser Low", "Plot Parameters", self.update_plot_limits, 'self')
        self.y_top_high_obj=gui.NumberEntry(plot_controls_frame, self.controller, 6, 0, 100., 100., -1., 200., "Laser High", "Plot Parameters", self.update_plot_limits, 'self')
        self.y_upper_middle_low_obj=gui.NumberEntry(plot_controls_frame, self.controller, 8, 0, -10., -10., -1000., 1000., "TEC Low", "Plot Parameters", self.update_plot_limits, 'self')
        self.y_upper_middle_high_obj=gui.NumberEntry(plot_controls_frame, self.controller, 10, 0, 110., 110., -1000., 1000., "TEC High", "Plot Parameters", self.update_plot_limits, 'self')
        self.y_lower_middle_low_obj=gui.NumberEntry(plot_controls_frame, self.controller, 12, 0, -10., -10., -1000., 1000., "Temper Low", "Plot Parameters", self.update_plot_limits, 'self')
        self.y_lower_middle_high_obj=gui.NumberEntry(plot_controls_frame, self.controller, 14, 0, 110., 110., -1000., 1000., "Temper High", "Plot Parameters", self.update_plot_limits, 'self')
        self.y_bottom_low_obj=gui.NumberEntry(plot_controls_frame, self.controller, 16, 0, -10., -10., -1000., 1000., "Current Low", "Plot Parameters", self.update_plot_limits, 'self')
        self.y_bottom_high_obj=gui.NumberEntry(plot_controls_frame, self.controller, 18, 0, 110., 110., -1000., 1000., "Current High", "Plot Parameters", self.update_plot_limits, 'self')

#        self.plot_param_objects = [self.x_range_obj, self.y_top_low_obj, self.y_top_high_obj, self.y_upper_middle_low_obj, self.y_upper_middle_high_obj, \
#                                   self.y_lower_middle_low_obj, self.y_lower_middle_high_obj, self.y_bottom_low_obj, self.y_bottom_high_obj]
        self.plot_param_objects = [self.x_range_obj, self.y_top_low_obj, self.y_top_high_obj, self.y_upper_middle_low_obj, self.y_upper_middle_high_obj, \
                                   self.y_lower_middle_low_obj, self.y_lower_middle_high_obj]
        self.crisp_ani = None

#       Local versions of plot variables for crisp plots
        self.x_range = self.x_range_obj.value
        self.x_max = -1     #   Vales of -1 will initiate setting new axis limits
        self.x_min = -1
        self.state_value = None

#       Set up display state frame:
        tk.Label(display_state_frame, text = "State", font=self.controller.widget_basic_font).grid(row=0, column = 0, columnspan=2)
        state_list_var = tk.StringVar(value=hshs_constants.STATE_LIST)
        self.state_listbox = tk.Listbox(display_state_frame, listvariable=state_list_var, width=15, height=1, justify='center') 
        self.state_listbox.grid(row=1, column=0, columnspan=2, sticky=tk.EW, padx=hshs_constants.ENTRY_PAD, pady=hshs_constants.ENTRY_PAD)
        self.state_listbox.see(0)
        tk.Label(display_state_frame, text = "Count", font=self.controller.widget_basic_font).grid(row=0, column = 3)
        self.state_count_text = tk.StringVar(display_state_frame, '0')
        state_info = tk.Label(display_state_frame, width = hshs_constants.ENTRY_WIDTH, textvariable=self.state_count_text)
        state_info.grid(row=1, column=3, padx=hshs_constants.ENTRY_PAD, pady=hshs_constants.ENTRY_PAD)

#       Set up plot controls frame
        plot_heading = gui.SimpleButton(plot_controls_frame, self.controller, 0, 0, 'Plot\nSettings', 'header font', '', '')

#       Set up crisp data plot
        font_size = 6
        plt.rc('font', size=font_size)  #   Reduce font size for plots
        self.crisp_fig, self.crisp_axes = plt.subplots(nrows=4, figsize = (3.5,2.7))
        plt.subplots_adjust(left=0.15, right=0.8, top=0.9, bottom=0.15)

        self.laser_current_lines = [self.add_plot_line(self.crisp_axes[0], self.colors[ii], "None", 0, 1, str(ii)) for ii in range(hshs_constants.NUM_CHANNELS)]
        self.tec_current_lines = [self.add_plot_line(self.crisp_axes[1], self.colors[ii], "None", 0, 1, str(ii)) for ii in range(hshs_constants.NUM_CHANNELS + 1)]
        self.temperature_lines = [self.add_plot_line(self.crisp_axes[2], self.colors[ii%4], "None", 0, 1, str(ii)) for ii in range(2*hshs_constants.NUM_CHANNELS)]
        self.current_lines = [self.add_plot_line(self.crisp_axes[3], self.colors[ii%4], "None", 0, 1, str(ii)) for ii in range(2)]
        self.all_lines_list = self.laser_current_lines + self.tec_current_lines + self.temperature_lines + self.current_lines
        for ii in range(hshs_constants.NUM_CHANNELS):
            self.temperature_lines[hshs_constants.NUM_CHANNELS + ii].set_dashes([2, 2])  #   Dashes for set points

#        self.pd_1_line = self.add_plot_line(self.crisp_axes[0], self.colors[0], "None", 0, 1, "PD 1  ")
#        self.pd_2_line = self.add_plot_line(self.crisp_axes[0], self.colors[1], "None", 0, 1, "PD 2  ")
#        self.pd_diff_line = self.add_plot_line(self.crisp_axes[1], self.colors[0], "None", 0, 1, "PD Diff")
#        self.set_point_line = self.add_plot_line(self.crisp_axes[1], self.colors[1], "None", 0, 1, "Set Pnt")
#        self.error_line = self.add_plot_line(self.crisp_axes[2], self.colors[0], "None", 0, 1, "Error")
#        self.zero_line = self.add_plot_line(self.crisp_axes[2], self.colors[1], "None", 0, 1, "Zero  ")
#        self.average_error_line = self.add_plot_line(self.crisp_axes[2], self.colors[2], "None", 0, 1, "Avg Err")
#        self.servo_line = self.add_plot_line(self.crisp_axes[3], self.colors[0], "None", 0, 1, "Servo ")
#        self.line_list = [self.pd_1_line, self.pd_2_line, self.pd_diff_line, self.set_point_line, self.error_line, self.average_error_line, self.servo_line]
#        self.line_list = [self.pd_1_line, self.pd_2_line, self.pd_diff_line, self.set_point_line, self.error_line, self.average_error_line]

#        self.set_point_line.set_dashes([2, 2])  #   Dashes for set point
#        self.zero_line.set_dashes([2, 2])  #   Dashes for zero line
#       Don't set axis limits now, they will be set in self.controller.prep_gui once values have been read from config.json
#        for ii in range(3):
#            self.crisp_axes[ii].legend(bbox_to_anchor = (1.3, 0.8))
        self.tec_current_lines[hshs_constants.NUM_CHANNELS].set_label("FanA")
        self.crisp_axes[1].legend(bbox_to_anchor = (1.3, 0.8))
        self.current_lines[0].set_label("Laser")
        self.current_lines[1].set_label("TEC")
        self.crisp_axes[3].legend(bbox_to_anchor = (1.3, 0.8))


        self.crisp_axes[0].set_ylabel('Laser\nPWMs')
        self.crisp_axes[1].set_ylabel('TEC\nPWMs')
        self.crisp_axes[2].set_ylabel('Tempera-\ntures')
        self.crisp_axes[3].set_ylabel('Total\nCurrents')
#        self.crisp_axes[3].set_ylabel('Servo')
#        self.crisp_axes[3].set_xlabel('Time (s)')
        self.crisp_axes[0].yaxis.set_label_coords(-0.1, 0.5)
        self.crisp_axes[1].yaxis.set_label_coords(-0.1, 0.5)
        self.crisp_axes[2].yaxis.set_label_coords(-0.1, 0.5)
        self.crisp_axes[3].yaxis.set_label_coords(-0.1, 0.5)

        self.crisp_canvas = FigureCanvasTkAgg(self.crisp_fig, master=displays_frame)
        self.crisp_canvas.get_tk_widget().grid(row=2, column=3, rowspan=2)


#       Set up fit plots
        """
        self.fit_fig, self.fit_axes = plt.subplots(nrows=1, ncols=3, figsize = (4,2))
        self.ax_list = self.fit_fig.axes
        if hshs_constants.DEBUG: print(self.ax_list, self.ax_list[0])
#        print(f'sizes: {len(self.data_lines)}, {len(self.ax_list)}')
        filled_markers = ('o', 'v', '^', '<', '>', '8', 's', 'p', '*', 'h', 'H', 'D', 'd', 'P', 'X')
        for i in range(hshs_constants.NUM_FIT_PLOTS):
            self.data_lines[i]=self.add_plot_line(self.ax_list[i], self.colors[0], ".", 2, 0.5, "data") # List of [plot_type]
            self.fit_lines[i]=self.add_plot_line(self.ax_list[i], self.colors[1], "None", 0, 1, "fit")
            self.fit_lines[i].set_dashes([2, 2])
        self.data_lines_b=self.add_plot_line(self.ax_list[2], self.colors[2], ".", 2, 0.5, "data") # List of [plot_type]     #   Zero_Cal plot has two lines
        self.fit_lines_b=self.add_plot_line(self.ax_list[2], self.colors[3], "None", 0, 1, "fit")
        self.fit_lines_b.set_dashes([2, 2])

        self.ax_list[0].set_ylabel('PD Difference')
        self.ax_list[1].set_ylabel('PD Difference')
        self.ax_list[0].set_xlabel('Servo')
        self.ax_list[1].set_xlabel('CRISP LED')
        self.ax_list[0].set_title("PD Diff Vs Servo", y = 0.95)
        self.ax_list[1].set_title("PD Diff Vs CRISP LED", y = 0.95)
        for i in range(hshs_constants.NUM_FIT_PLOTS):
            self.ax_list[i].tick_params(pad = 1)
        self.fit_canvas = FigureCanvasTkAgg(self.fit_fig, master=fit_data_frame)
        self.fit_canvas.get_tk_widget().grid(row=1, column=0, padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        """


    def start_crisp_animation(self, crisp_q, entry_obj):
        """Start crisp animation. This extra layer is required if calling module does not have matplotlib"""
        self.crisp_ani = animation.FuncAnimation(self.crisp_fig, self.crisp_graph_animate, fargs = (crisp_q, entry_obj), \
            interval=hshs_constants.PLOT_INTERVAL, cache_frame_data=False) # Must retain reference to animation (self.) or it will end



    def add_plot_line(self, axis, color_val, marker_val, marker_size, line_width, label_string):
        """Add line to plot defined by axis."""
        return axis.plot([], [], color=color_val, marker=marker_val, markersize=marker_size, lw=line_width, label=label_string)[0]

    def crisp_graph_animate(self, i, queue_name, entry_obj):     #   Does not work without argument i, though it is not used
        """Animation method receives crisp data and updates plots and status values."""
        new_data = queue_name.qsize() > 0
        new_axes = self.axis_limits_q.qsize() > 0

        if new_data:  #   Only update if there is new data
            while queue_name.qsize() > 0:      #   Update data from queue and axes from controls
                _sample = queue_name.get(block=True, timeout=None)
                if _sample.time.value < self.time_array[self.array_index-1]:     #   Remove old points from plot if time jumps backwards
                    if hshs_constants.DEBUG: print (f'Reseting  time axis, new time.value: {_sample.time.value} < previous data point: {self.time_array[self.array_index-1]}')
                    self.array_index = 0     #   Restart filling data if time jumps backwards
#                    for next_array in self.data_array_list: #   Set all old data to nan
#                        next_array = [np.nan]*len(next_array)
                    self.time_array = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
                    self.y__array_new = np.nan*np.ones((self.num_plot_lines, hshs_constants.PLOT_DATA_SIZE))
                    self.counter_array = np.nan*np.ones(hshs_constants.PLOT_DATA_SIZE)
                if self.array_index == hshs_constants.PLOT_DATA_SIZE-1:     #   Shift array and start filling from midpoint
                    shift_count = hshs_constants.PLOT_DATA_SIZE//2
#                    for next_array in self.data_array_list:
#                        shift_data(next_array, shift_count)
#                    self.array_index -= shift_count
#                    print('Shifting array, ', self.array_index, shift_count)

                    print('Shifting array, ', self.array_index, shift_count, self.time_array[0], self.time_array[1], self.time_array[self.array_index-1], self.time_array[-1])
                    shift_data(self.time_array, shift_count)
                    shift_data(self.counter_array, shift_count)
                    shift_2d_data(self.y__array_new, shift_count)
                    self.array_index -= shift_count
                    print('Shifting array, ', self.array_index, shift_count, self.time_array[0], self.time_array[1], self.time_array[self.array_index-1], self.time_array[-1])

#                for array, attribute in zip(self.data_array_list, self.attribute_list):     #   Insert new points from latest queue sample
#                    array[self.array_index] = getattr(_sample,attribute).value

                self.time_array[self.array_index] = _sample.time.value
#                self.y__array_new[:,self.array_index] = _sample.data.value[:]

                self.y__array_new[:,self.array_index] = np.concatenate([_sample.laser_pwm.value, _sample.tec_pwm.value, np.array([_sample.fan_pwm.value]), \
                                                            _sample.laser_temperatures.value, _sample.temperature_set_points.value,
                                                            np.array([_sample.laser_current.value]), np.array([_sample.tec_current.value]) ])
#                                                            _sample.laser_current.value, _sample.tec_current.value])

                self.counter_array[self.array_index]  = _sample.count.value


                self.array_index += 1

                if self.laststate != _sample.state.value:     #   Update state and count when state changes
                    self.state_value = _sample.state.value%100
                    if hshs_constants.DEBUG: print("STATE_VALUE = ", self.state_value, type(self.state_value), self.controller.plot_obj.state_value)
                    count_value = _sample.state.value//100

                    self.state_listbox.see(self.state_value)
                    self.state_count_text.set(count_value)
                    self.laststate = _sample.state.value

            if (self.x_min > self.time_array[0]):
                i_start = bisect.bisect_right(self.time_array, self.x_min, 0, self.array_index-1) # Find index where all self.time_array[istart:] > self.x_min, limit search to avoid bisecting into nan range
            else:
                i_start = 0
            i_step = (self.array_index-i_start)//500 + 1       #   Step over points to keep number of plotted points below 500

#            for line, array in zip(self.line_list, self.data_array_list):
#                line.set_data(self.time_array[i_start:self.array_index-1:i_step], array[i_start:self.array_index-1:i_step])
#            self.time_array_short = np.array([self.time_array[i_start], self.time_array[self.array_index-1]])
#            self.zero_line.set_data(self.time_array_short, self.zero_array_short)

            if self.controller.frame_list[self.controller.current_frame_index] == "DataPage":   # Only update graphs when page is at top
                for j, line in enumerate(self.all_lines_list, start=0):
                    line.set_data(self.time_array[i_start::i_step], self.y__array_new[j,i_start::i_step])
            

        if new_axes:
            while self.axis_limits_q.qsize() > 0:      #   Update data from queue and axes from controls
                entry_object = self.axis_limits_q.get(block=True, timeout=None)
                if hshs_constants.DEBUG: print('New value for:', entry_object.label_name)
                if entry_object.label_name == "X Range":
                    self.x_range = entry_object.value
                if entry_object.label_name == "Laser Low" or entry_object.label_name == "Laser High":
                    self.crisp_axes[0].set_ylim(self.y_top_low_obj.value,self.y_top_high_obj.value)
                if entry_object.label_name == "TEC Low" or entry_object.label_name == "TEC High":
                    self.crisp_axes[1].set_ylim(self.y_upper_middle_low_obj.value,self.y_upper_middle_high_obj.value)
                if entry_object.label_name == "Temper Low" or entry_object.label_name == "Temper High":
                    self.crisp_axes[2].set_ylim(self.y_lower_middle_low_obj.value,self.y_lower_middle_high_obj.value)
                if entry_object.label_name == "Current Low" or entry_object.label_name == "Current High":
                    self.crisp_axes[3].set_ylim(self.y_bottom_low_obj.value,self.y_bottom_high_obj.value)

        if new_data or new_axes:
            new_x_max = max(self.x_range, self.time_array[self.array_index-1] + 1) #   The + 1 keeps gap between the data and x_max
            new_x_min = new_x_max - self.x_range
            if abs(new_x_max - self.x_max) > 0.1 or abs(new_x_min - self.x_min) > 0.1:     #   Update axis frequency significantly affects display time
                self.x_max = new_x_max
                self.x_min = new_x_min
                for axis in self.crisp_axes:
                    axis.set_xlim(self.x_min, self.x_max)
            return self.crisp_axes[0],self.crisp_axes[1],self.crisp_axes[2]
#            return self.crisp_axes[0],self.crisp_axes[1],self.crisp_axes[2],self.crisp_axes[3]

        if not self.controller.call_again:
            print("Ending crisp_graph_animate")
            self.crisp_ani.event_source.stop()

    def fit_graph_update(self, fits_q):        #   Display fits to transition (rise/fall) data
        """Update fit plot from queue fits_q"""
        #   N.B. first_point and num_points are for count values, but not every count value is in the sample sent to the cpu
        if fits_q.qsize() > 0:      #   Check for items in queue
            end=fits_q.queue[0].first_point.value+fits_q.queue[0].num_points.value    #   Find last point in fit without removing element from queue
            if self.counter_array[self.array_index-1] > end:    #   May need to wait for all data to arrive
                _sample = fits_q.get(block=True, timeout=None)     #  Get sample from queue
                if hshs_constants.DEBUG: print('_sample.type = ', _sample.type, hshs_constants.STATE_LIST[_sample.type])
                if hshs_constants.DEBUG: print ('_sample.slopes.value, _sample.intercepts.value:', _sample.slopes.value, _sample.intercepts.value)
                plot_type = hshs_constants.PLOT_TYPE[hshs_constants.STATE_LIST[_sample.type]]

                ax_set = self.ax_list[plot_type]
                if hshs_constants.DEBUG: print ('ax_set = ', ax_set)
                start = _sample.first_point.value
                stop = _sample.first_point.value + _sample.num_points.value - 1

                i_start = bisect.bisect_right(self.counter_array, start, 0, self.array_index-1) # Find index where all self.counter_array[istart:] > start, limit search to avoid bisecting into nan range
                i_stop = bisect.bisect_left(self.counter_array, stop, 0, self.array_index-1) # Find index where all self.counter_array[istart:] < stop, limit search to avoid bisecting into nan range
                if hshs_constants.DEBUG: print ('all plot_type = ', plot_type)
                if plot_type == 0:
                    self.data_lines[0].set_data(self.servo_data[i_start:i_stop], self.pd_diff_data[i_start:i_stop])
                    y_fit_data = np.ones(i_stop-i_start)
                    for ii in range(i_start,i_stop):
                        y_fit_data[ii-i_start] = _sample.slopes.value*self.servo_data[ii]+_sample.intercepts.value
                    self.fit_lines[0].set_data(self.servo_data[i_start:i_stop], y_fit_data)
                elif plot_type == 1:
                    if hshs_constants.DEBUG: print("in plot_type 1 case")
                    self.data_lines[1].set_data(self.crisp_led_array[i_start:i_stop], self.pd_diff_data[i_start:i_stop])
                    y_fit_data = np.ones(i_stop-i_start)
                    for ii in range(i_start,i_stop):
                        y_fit_data[ii-i_start] = _sample.slopes.value*self.crisp_led_array[ii]+_sample.intercepts.value
                    self.fit_lines[1].set_data(self.crisp_led_array[i_start:i_stop], y_fit_data)
                    if hshs_constants.DEBUG: print(self.crisp_led_array[i_start:i_stop], self.pd_diff_data[i_start:i_stop], y_fit_data)
                elif plot_type == 2:
                    self.data_lines[2].set_data(self.counter_array[i_start:i_stop], self.pd_1_data[i_start:i_stop])
                    self.data_lines_b.set_data(self.counter_array[i_start:i_stop], self.pd_2_data[i_start:i_stop])
                else:
                    if hshs_constants.DEBUG: print ('plot_type = ', plot_type)
#                    print(_sample.slopes.value, _sample.intercepts.value)
#                    print(y_fit_data)
                ax_set.relim()      #   Autoscale axes
                ax_set.autoscale_view(True,True,True)
                self.fit_canvas.draw()  #   Update plot
        if self.controller.call_again:
            self.controller.after(100, self.fit_graph_update, fits_q)    #   Queue update_image for next iteration
        else:
            print("Ending fit_graph_update")

    def update_plot_limits(self, _entry_param):
#        print('Update plot limits here')
        self.axis_limits_q.put(_entry_param)
        if hshs_constants.DEBUG: print('updating:', _entry_param.label_name)
        
    
    def set_all_axes(self):
#        for plot_object in [self.x_range_obj, self.y_top_low_obj,  self.y_upper_middle_low_obj, self.y_lower_middle_low_obj, self.y_bottom_low_obj]:
        for plot_object in [self.x_range_obj, self.y_top_low_obj,  self.y_upper_middle_low_obj, self.y_lower_middle_low_obj]:
            self.update_plot_limits(plot_object)        

def shift_data(in_array, shift_count):  #   Note: this changes input array, there is no return necessary
    array_len = len(in_array)
    print(shift_count,array_len)
    if (shift_count < 1):
        pass
    elif (shift_count >= array_len):
        in_array[:] = [np.nan]*array_len
    else:
        remainder_len = array_len-shift_count
        in_array[0:remainder_len] = in_array[shift_count:shift_count+remainder_len]
        in_array[remainder_len:array_len] = [np.nan]*(shift_count)
