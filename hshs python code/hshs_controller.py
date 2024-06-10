"""GUI for pid code."""
import tkinter as tk
from tkinter import font as tkfont
import queue
import time
import threading
import hshs_constants        #       Local python modules
import hshs_gui_interface as gui
import hshs_file_save
import hshs_mcu_comm
import hshs_plot_class
import hshs_ms2k_interface
import sys

#   TODO: test machine/human, what should be human versus both?
#   TODO: Clean up C++: fits/other calcss to new module?s
#   TODO: Display slopes, etc.?
#   TODO: Next steps: for up/down, update display, trigger entry update; for Return/KP_Enter/left mouse click, create focusout, validate/trigger entry update
#   TODO: switch to stringvar for entries?
#   TODO: On focus out: check for value change, if so, update parameter and update mcu
#   TODO: Use last entry name for widget update; handle from focus sout; split focus out and mcu update funtions from update_entry_widget
#   TODO: Note: do appendpoints, then switchstate/calcfits; already including last fit point
#   TODO: Clean up, check extra print statements
#   TODO: Is there extra update in focusout?

class RootApp(tk.Tk):
    """Main tkinter class; For reference, see: https://stackoverflow.com/questions/7546050/switch-between-two-frames-in-tkinter/7557028#7557028"""
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        default_font = tkfont.nametofont("TkDefaultFont")
        default_font.configure(family=hshs_constants.DEFAULT_FONT_FAMILY , size=hshs_constants.DEFAULT_FONT_SIZE )
        self.option_add("*Font", default_font)      #   Change all fonts to default font
        if hshs_constants.DEBUG: print(f'Default font settings: {default_font.actual()}')
        crisp_q = queue.Queue()
        fits_q = queue.Queue()
        hist_q = queue.LifoQueue()
        chan_0_q = queue.Queue()
        self.title_font = tkfont.Font(family=hshs_constants.DEFAULT_FONT_FAMILY, size=hshs_constants.TITLE_FONT_SIZE, weight="bold", slant="italic")
        self.title_font_behind = tkfont.Font(family=hshs_constants.DEFAULT_FONT_FAMILY, size=hshs_constants.TITLE_FONT_SIZE)
        self.title_font_front = tkfont.Font(family=hshs_constants.DEFAULT_FONT_FAMILY, size=hshs_constants.TITLE_FONT_SIZE, weight="bold", slant="italic")
        self.widget_basic_font = tkfont.Font(family=hshs_constants.DEFAULT_FONT_FAMILY, size=hshs_constants.DEFAULT_FONT_SIZE)
        self.widget_header_font = tkfont.Font(family=hshs_constants.DEFAULT_FONT_FAMILY, size=hshs_constants.DEFAULT_FONT_SIZE, weight="bold")
        container = tk.Frame(self)  #   This container is a frame holding a stack of frames/pages
        container.pack(side="top", fill="both", expand=True)        #   Expand container to fill window
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)
        self.frames = {}    #   Create dictionary of frame references; parent is main frame (container), conqtroller is RootApp
        self.frames["SettingsPage"] = SettingsPage(parent=container, controller=self)
        self.frames["DataPage"] = DataPage(parent=container, controller=self)
        self.frames["StageControlPage"] = StageControlPage(parent=container, controller=self)
        self.frames["SettingsPage"].grid(row=0, column=0, sticky="nsew")   # Stack frames (pages) on top of each other
        self.frames["DataPage"].grid(row=0, column=0, sticky="nsew")
        self.frames["StageControlPage"].grid(row=0, column=0, sticky="nsew")
        self.frame_list = list(self.frames)
        self.current_frame_index=0
        self.led_on = False
        self.key_focus_index = 0
        self.last_widget_focus = None
        self.call_again = True

#       Create commands and populate frames with widgets
        self.gui_obj = gui.GUIInterface(self)
        self.gui_obj.read_json_file()      #       Read configuration from json file
        self.file_obj = hshs_file_save.FileSave(self, self.frames["SettingsPage"].settings_parameters_frame, self.gui_obj)     # file saving members and methods
        self.plot_obj = hshs_plot_class.PlotClass(self, self.frames["DataPage"].display_state_frame, self.frames["DataPage"].displays_frame, \
            self.frames["DataPage"].plot_controls_frame, self.frames["StageControlPage"])
        self.mcu_obj = hshs_mcu_comm.SerialPort(self, self.gui_obj, self.file_obj, self.plot_obj, crisp_q, fits_q)
        self.mcu_obj.comm_control_widgets(self.frames["SettingsPage"].port_frame)
        self.mcu_obj.state_controls_widgets(self.frames["DataPage"].state_controls_frame)
        self.mcu_obj.crisp_led_controls_widgets(self.frames["DataPage"].crisp_settings_frame)
        self.mcu_obj.laser_controls_widgets(self.frames["DataPage"].laser_settings_frame)
        self.mcu_obj.tec_controls_widgets(self.frames["DataPage"].tec_settings_frame)
        self.mcu_obj.fan_controls_widgets(self.frames["DataPage"].fan_settings_frame)
        self.mcu_obj.temperature_setpoint_widgets(self.frames["DataPage"].temperature_settings_frame)
        self.mcu_obj.temperature_lock_widgets(self.frames["DataPage"].temperature_lock_frame)

        self.mcu_obj.scan_controls_widgets(self.frames["DataPage"].scan_controls_frame)
        self.mcu_obj.hshs_controls_widgets(self.frames["DataPage"].hshs_controls_frame)
        self.mcu_obj.feedback_controls_widgets(self.frames["DataPage"].feedback_controls_frame)
#        self.mcu_obj.dither_controls_widgets(self.frames["DataPage"].dither_controls_frame)

        self.mcu_obj.acquire_parameters_widgets(self.frames["SettingsPage"].acquire_parameters_frame)
        self.mcu_obj.hshs_displays_widgets(self.frames["DataPage"].value_displays_frame)
        self.mcu_obj.update_response_history()
        self.mcu_obj.update_crisp_controls_gui()

        self.ms2k_obj = hshs_ms2k_interface.MS2KClass(self, self.gui_obj, self.frames["StageControlPage"])
        self.ms2k_obj.comm_control_widgets(self.frames["StageControlPage"].ms2k_frame)
        self.ms2k_obj.xy_stage_control_widgets(self.frames["StageControlPage"].xy_stage_control_frame)
        self.ms2k_obj.z_stage_control_widgets(self.frames["StageControlPage"].z_stage_control_frame)
        self.ms2k_obj.update_response_history()


#        self.plot_obj.fit_graph_update(fits_q)     #   Start callback loop
        #TODO: the below line causes an error: "Fatal Python error: PyEval_RestoreThread: the function must be called with the GIL held..."
        #      In pcr_combined_code, resolved by adding the after call but doesn't fix here.
        # self.after(50000, self.plot_obj.start_crisp_animation(crisp_q, self.gui_obj))   #   Start PID plot animation
        self.json_obj_list =  self.mcu_obj.scan_controls \
            + self.plot_obj.plot_param_objects + self.mcu_obj.acquire_param_objects \
            + self.file_obj.file_objects + [self.file_obj.folder_name_obj] + self.mcu_obj.crisp_led_controls \
            + [self.mcu_obj.servo] + self.mcu_obj.feedback_control_list + self.mcu_obj.hshs_control_list_extended

        self.show_frame("SettingsPage")

        key_list=["c", "C", "d", "D", "i", "I", "k", "K", "l", "L", "m", "M", "n", "N", "q", "Q", "r", "R", "s", "S", "t", "T", "u", "U", "v", "V", "Z", "z", "[", "]", "0", "1", "2", "3"]
        for key in key_list:
            self.bind(key, lambda event: self.gui_obj.handle_global_key_press(event, self.mcu_obj, self.ms2k_obj))

        self.bind("<Button-1>", self.click_event)       #   Handle mouse left click

    def show_frame(self, page_name):
        """This method raises selected frame (page) to top of stack based on page name"""
        frame = self.frames[page_name]
        frame.tkraise()
        self.current_frame_index = self.frame_list.index(page_name)

    def click_event(self, event):       #   This is primarily used to create focus out events when clicking outside entry box
        """Left mouse click generates focus out when not on entry"""
        x_pos, y_pos = self.winfo_pointerxy()                   # get the mouse position on screen
        new_widget_focus = self.winfo_containing(x_pos, y_pos)            # identify the widget at this location
        if hshs_constants.DEBUG: print (f'Mouse click into: {new_widget_focus}, with event.keysym: {event.keysym}, is entry box: {str(new_widget_focus).find("entry") > -1}')
        if str(new_widget_focus).find("entry") == -1 :    # Generate FocusOut if don't click on entry box
            self.focus()     #   Generate FocusOut event
            self.last_widget_focus = new_widget_focus   #   TODO: Is this used?

    def advance_frame(self, direction):
        """This method advances sframe (page, in insertion order) to top of stack based on page name"""
        self.current_frame_index = (self.current_frame_index + int(direction/abs(direction)))%len(self.frame_list)
        frame = self.frames[self.frame_list[self.current_frame_index]]
        frame.tkraise()

    def prep_gui(self):
        """Initialize GUI parameter objects"""
        if hshs_constants.DEBUG: print("In prep_gui")
        self.gui_obj.update_from_json_config(self.json_obj_list, self.mcu_obj, self.ms2k_obj)      #       Read configuration from json file
        self.plot_obj.set_all_axes()

    def quit(self):
        """Shut down: Stop mcu, close data files, store configuation to json file, stop GUI"""
        try:
            if hshs_constants.DEBUG: print("starting quit thread")
            self.thread_initiate_close = threading.Event()  ###  FIXME: This seems irrelevant
            self.quit_thread = threading.Thread(target=self.quit_thread)
            self.quit_thread.start()
        except BaseException as error_message:
            print("Error starting quit thread: ", error_message)

    def quit_thread(self):
        if hshs_constants.DEBUG: print("Stopping, Current active threads:", [thread.name for thread in threading.enumerate()])
        self.call_again = False     #   Stop plot, response history, and update_pwm_and_fan_gui loops
        if self.mcu_obj.port_successfully_started:       #   Stop mcu
            self.mcu_obj.thread_initiate_close.set()        #   Initiate microcontroller close
#            print("Shutting mcu function")
#            self.mcu_obj.serial_port.write(b'shutdown\n')      #   Shutdown mcu, this queues element to stop file queue after receiving last response (shutdown)
            self.mcu_obj.send_to_port('shutdown')      #   Send shutdown to mcu, mcu response halts read_from_port and file threads
        else:
            file_queue_element = hshs_file_save.FileQueueAction(None, "CloseFileThread")
#            file_queue_element = hshs_mcu_comm.FileQueueData("", "", "CloseFileThread") #   Queue element to stop file queue
            self.file_obj.file_data_q.put(file_queue_element)
        if self.ms2k_obj.port_successfully_started:       #   Stop ms2k
#            self.ms2k_obj.thread_initiate_close.set()        #  FIXME: restore coordinated shutdown with MS2K
            self.ms2k_obj.run_thread = False        #   Initiate microcontroller close
#            self.mcu_obj.send_to_port('shutdown')      #   Send shutdown to mcu, mcu response halts read_from_port and file threads
        self.gui_obj.dump_json_config(self.json_obj_list, self.mcu_obj, self.ms2k_obj)      #   Save configuration data in json file
        thread_list = threading.enumerate()
        thread_test_count = 0
        while len(thread_list) > 2 and thread_test_count < 40:     #   Need all threads to close except MainThread
            time.sleep(0.1)
            if thread_test_count % 10 == 9:  print("Waiting for threads to close", [thread.name for thread in thread_list])
            thread_list = threading.enumerate()
            thread_test_count += 1
        if hshs_constants.DEBUG: print(f'Waited {thread_test_count*0.1:.1f} seconds for threads to close')
        if thread_test_count >= 40: print(f'Timeout waiting for threads {thread_list[1].name} and {thread_list[2].name} to close')
        if hshs_constants.DEBUG: print("Before destroy")
        self.after(100, self.destroy_gui)   #   Program hangs without this

    def destroy_gui(self):
        """Destroy GUI if it doesn't close properly"""
        if hshs_constants.DEBUG: print("start destroy")
        self.destroy()
        print("GUI closed")
        sys.exit(0)

class SettingsPage(tk.Frame):
    """Class for settings page; includes serial functions."""
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
#   Set up frames
        self.page_buttons_frame = tk.Frame(self, borderwidth=1, relief="solid")
        self.page_buttons_frame.grid(row=0, column = 0, columnspan = 3, sticky='w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
#        self.page_buttons_frame.configure(bg='red')
        self.port_frame = tk.Frame(self, borderwidth=1, relief='solid')
        self.port_frame.grid(row=1, column = 0, columnspan = 3, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.settings_parameters_frame = tk.Frame(self, borderwidth=1, relief='solid')
        self.settings_parameters_frame.grid(row=2, column = 0, columnspan = 3, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.response_history = tk.Listbox(self, height=18, width=80)
        self.response_history.grid(row=3, column=0, columnspan = 3, sticky=tk.EW, padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.acquire_parameters_frame = tk.Frame(self, borderwidth=1, relief='solid')
        self.acquire_parameters_frame.grid(row=4, column = 0, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)

#   Set up page buttons frame:
        settings_button = gui.SimpleButton(self.page_buttons_frame, self.controller, 0, 0, 'Settings Page', 'title font front', '', '')
        data_page_button = gui.SimpleButton(self.page_buttons_frame, self.controller, 0, 1, 'Go to Data Page', 'title font behind', controller.show_frame, 'DataPage')
        fits_page_button = gui.SimpleButton(self.page_buttons_frame, self.controller, 0, 2, 'Go to Stage Control Page', 'title font behind', controller.show_frame, 'StageControlPage')

class DataPage(tk.Frame):
    """Class for data page"""
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
#   Set up main frames
        self.page_buttons_frame = tk.Frame(self, borderwidth=1, relief="solid")      #   Define frames within main DataPage frame
        self.page_buttons_frame.grid(row=0, column = 0, columnspan = 2, sticky='w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)

        self.data_page_row_1 = tk.Frame(self)      #   Define frames within main DataPage frame
        self.data_page_row_1.grid(row=1, column = 0, columnspan = 2, sticky='w')
        self.state_controls_frame = tk.Frame(self.data_page_row_1, borderwidth=1, relief='solid')
        self.state_controls_frame.grid(row=0, column = 0, columnspan = 2, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.display_state_frame = tk.Frame(self.data_page_row_1, borderwidth=1, relief='solid')
        self.display_state_frame.grid(row=0, column = 3, columnspan = 2, sticky= 'e', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)

        self.data_page_row_2 = tk.Frame(self)
        self.data_page_row_2.grid(row=2, column = 0, columnspan = 2, sticky= 'w')
        self.feedback_controls_frame = tk.Frame(self.data_page_row_2, borderwidth=1, relief='solid')
        self.feedback_controls_frame.grid(row=0, column = 0, columnspan = 2, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)

        self.displays_frame = tk.Frame(self)
        self.displays_frame.grid(row=3, column = 0, columnspan = 2, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.plot_controls_frame = tk.Frame(self.displays_frame, borderwidth=1, relief='solid')
        self.plot_controls_frame.grid(row=2, column = 4, rowspan=2, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.value_displays_frame = tk.Frame(self.displays_frame, borderwidth=1, relief='solid')
        self.value_displays_frame.grid(row=2, column = 5, rowspan=2, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)

        self.data_page_row_4 = tk.Frame(self)
        self.data_page_row_4.grid(row=4, column = 0, columnspan = 2, sticky= 'w')
        self.scan_controls_frame = tk.Frame(self.data_page_row_4, borderwidth=1, relief='solid')
        self.scan_controls_frame.grid(row=0, column = 1, columnspan = 1, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.crisp_settings_frame = tk.Frame(self.data_page_row_4, borderwidth=1, relief='solid')
        self.crisp_settings_frame.grid(row=0, column = 2, columnspan = 1, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)

        self.data_page_row_5 = tk.Frame(self)
        self.data_page_row_5.grid(row=5, column = 0, columnspan = 2, sticky= 'w')
        self.hshs_controls_frame = tk.Frame(self.data_page_row_5, borderwidth=1, relief='solid')
        self.hshs_controls_frame.grid(row=0, column = 0, columnspan = 2, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
#        self.dither_controls_frame = tk.Frame(self.data_page_row_5, borderwidth=1, relief='solid')
#        self.dither_controls_frame.grid(row=0, column = 2, columnspan = 2, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)

        self.data_page_row_6 = tk.Frame(self)
        self.data_page_row_6.grid(row=6, column = 0, columnspan = 2, sticky= 'w')
        self.laser_settings_frame = tk.Frame(self.data_page_row_6, borderwidth=1, relief='solid')
        self.laser_settings_frame.grid(row=0, column = 1, columnspan = 1, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.tec_settings_frame = tk.Frame(self.data_page_row_6, borderwidth=1, relief='solid')
        self.tec_settings_frame.grid(row=0, column = 2, columnspan = 1, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.temperature_settings_frame = tk.Frame(self.data_page_row_6, borderwidth=1, relief='solid')
        self.temperature_settings_frame.grid(row=0, column = 3, columnspan = 1, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.fan_settings_frame = tk.Frame(self.data_page_row_6, borderwidth=1, relief='solid')
        self.fan_settings_frame.grid(row=0, column = 4, columnspan = 1, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)

        self.data_page_row_7 = tk.Frame(self)
        self.data_page_row_7.grid(row=7, column = 0, columnspan = 2, sticky= 'w')
        self.temperature_lock_frame = tk.Frame(self.data_page_row_7, borderwidth=1, relief='solid')
        self.temperature_lock_frame.grid(row=0, column = 1, columnspan = 1, sticky= 'w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)


#   Page buttons frame:
        settings_button = gui.SimpleButton(self.page_buttons_frame, self.controller, 0, 0, 'Go to Settings Page', 'title font behind', controller.show_frame, 'SettingsPage')
        data_page_button = gui.SimpleButton(self.page_buttons_frame, self.controller, 0, 1, 'Data Page', 'title font front', '', '')
        fits_page_button = gui.SimpleButton(self.page_buttons_frame, self.controller, 0, 2, 'Go to Stage Control Page', 'title font behind', controller.show_frame, 'StageControlPage')

class StageControlPage(tk.Frame):
    """Class for data page"""
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller

#       Set up main frames
        self.page_buttons_frame = tk.Frame(self, borderwidth=1, relief="solid")
        self.page_buttons_frame.grid(row=0, column = 0, sticky='w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.ms2k_frame = tk.Frame(self, borderwidth=1, relief="solid")
        self.ms2k_frame.grid(row=1, column = 0, sticky='w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.xy_stage_control_frame = tk.Frame(self, borderwidth=1, relief="solid")
        self.xy_stage_control_frame.grid(row=3, column = 3, columnspan= 2 , sticky='w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)
        self.z_stage_control_frame = tk.Frame(self, borderwidth=1, relief="solid")
        self.z_stage_control_frame.grid(row=3, column = 2, sticky='w', padx=hshs_constants.FRAME_PAD, pady=hshs_constants.FRAME_PAD)


#       Page buttons frame:
        settings_button = gui.SimpleButton(self.page_buttons_frame, self.controller, 0, 0, 'Go to Settings Page', 'title font behind', controller.show_frame, 'SettingsPage')
        data_page_button = gui.SimpleButton(self.page_buttons_frame, self.controller, 0, 1, 'Go to Data Page', 'title font behind', controller.show_frame, 'DataPage')
        fits_page_button = gui.SimpleButton(self.page_buttons_frame, self.controller, 0, 2, 'Stage Control Page', 'title font front', '', '')


def main():
    """Main program."""
    root = RootApp()
    root.geometry('1220x800')
    root.title('PID Controller')
    root.after(1, root.focus_force)   #   This is needed for Windows only to get focus on main GUI window
    root.after(10, root.prep_gui)    #   This needs to occur after mainloop creates GUI to avoid zsh: illegal hardware
                                        # instruction error when messagebox is created when polling for directory
    root.mainloop()
    print("The End")

if __name__ == "__main__":
    main()
