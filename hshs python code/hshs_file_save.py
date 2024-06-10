#   File_Save module
"""Class File_Save for buttons, members, and methods to save data, images, and parameters to files"""

import tkinter as tk

import queue
import threading        #   Write to files in a different thread

import time
from tkinter import filedialog
import datetime
import os
import numpy as np
import hshs_constants
import hshs_gui_interface as gui
import subprocess

#   Dependencies on entry parameters and serial port (pending; currently through controller)


class FileQueueAction():
    """Class with sole purpose of changing file status"""
    def __init__(self, obj, action):
        self.obj = obj
        self.action = action
class ButtonFileObject(gui.ToggleButton):
    """Child of ToggleButton that tracks file-specific attributes"""
    def __init__(self, parent, controller, row, column, on_text, off_text, label_name, command, filename_suffix, filename_extension):
        gui.ToggleButton.__init__(self, parent, controller, row, column, False, False, on_text, off_text, label_name, "File Save Parameters", False, command, 'self')
        self.filename_suffix = filename_suffix
        self.filename_extension = filename_extension
        self.file_path = None
        self.filename_string = ""
        self.file_header_string = ""
        self.file_is_open = False
        self.file_object = None
class FileSave():
    """Class for saving to file"""

    def __init__(self, controller, active_frame, entry_parameter_object):
        self.controller = controller
        self.entry_parameter_object = entry_parameter_object

        self.file_data_q = queue.Queue()

#       Instantiate and initialize file control:

#       ButtonFileObjects are associated with GUI buttons for opening/closing files and also store state of resultant files. Vis Cam button creates multiple files.
        file_settings_heading = gui.SimpleButton(active_frame, self.controller, 0, 0, 'File\nSettings', 'header font', '', '')

        self.all_file_obj=ButtonFileObject(active_frame, self.controller, 0, 1, "Save\n No Data", "Save\nAll Data", "All Data", \
                             self.handle_file_button_key_press, "", "")
        self.crisp_file_obj=ButtonFileObject(active_frame, self.controller, 0, 2, "Saving\n CRISP Data", "Save\nCRISP Data", "CRISP Data", \
                             self.handle_file_button_key_press, "_crisp_data", "csv")
        self.fit_file_obj=ButtonFileObject(active_frame, self.controller, 0, 3, "Saving\n Fit Data", "Save\nFit Data", "Fit Data", \
                             self.handle_file_button_key_press, "_fit_data", "csv")
        self.metadata_file_obj=ButtonFileObject(active_frame, self.controller, 0, 4, "Saving\n Metadata", "Save\nMetadata", "Metadata", \
                             self.handle_file_button_key_press, "_metadata", "csv")
        self.response_file_obj=ButtonFileObject(active_frame, self.controller, 0, 5, "Saving\n Responses Data", "Save\nResponses Data", "Responses", \
                             self.handle_file_button_key_press, "_responses", "txt")

        self.file_objects = [self.crisp_file_obj, self.fit_file_obj, self.metadata_file_obj, self.response_file_obj]

        self.folder_name_obj = gui.TextEntry(active_frame, self.controller, 0, 10, 65, "", "Data Folder", "folder_name", "Text Entry")
        self.folder_name_obj.bind('<FocusIn>', lambda event: self.browse_directory()) # Wrapping callback removes event from argument list

        self.crisp_file_obj.file_header_string = 'global_time_sec, state_time_sec, global_counter, state_counter, state_number, counter, state_text, ' + \
                                                    ','.join(['laser_pwm_'+str(i) for i in range(hshs_constants.NUM_CHANNELS)] + ['tec_pwm_'+str(i) for i in range(hshs_constants.NUM_CHANNELS)] \
                                                             + ['fan_pwm'] + ['laser_temperatures_'+str(i) for i in range(hshs_constants.NUM_CHANNELS)] + \
                                                                ['temper_set_points'+str(i) for i in range(hshs_constants.NUM_CHANNELS)]) + ', summed_laser_current, summed_tec_current'

        self.fit_file_obj.file_header_string = 'state_number, state_name, first_point, num_points, slope_data, intercept_data, debug_data, y_offset'
        self.metadata_file_obj.file_header_string = 'Parameter, Value, Type, State'

        tk.Label(active_frame, text = "File Descriptor", font=self.controller.widget_basic_font).grid(row=2, column = 0)
        self.file_descriptor_text = tk.StringVar(active_frame, "")
        self.file_descriptor = tk.Entry(active_frame, textvariable=self.file_descriptor_text)
        self.file_descriptor.grid(row=2, column=1, columnspan = 10, sticky=tk.EW, padx=hshs_constants.ENTRY_PAD, pady=hshs_constants.ENTRY_PAD)

        try:
            file_thread = threading.Thread(target=self.run_file_thread)
            file_thread.start()
        except BaseException as error_message:
            print("Error starting file write thread: ", error_message)

    def open_file(self, obj):
        """Function opens new file including datetime and adds parameters for writing"""
        if obj.file_is_open:
            print(f'Error: Received open requested for {obj.name} file, but file is already open')
            return
        if hshs_constants.DEBUG: print("Opening file for ", obj.name)
        if obj.value is True:

            folder_path = self.folder_name_obj.value
            if not os.path.isdir(folder_path):   #   Check for valid path
                print(f'Warning, {folder_path} is not a valid path, please select a directory')
                if self.browse_directory():     #   Not valid, ask for path
                    if hshs_constants.DEBUG: print ("Browse for directory")
                    folder_path = self.folder_name_obj.value
                    if hshs_constants.DEBUG: print("New folder_path", folder_path)
                else:     #   if not valid, ask for path
                    obj.update_param(False)
                    obj.file_is_open = False
                    print(f'Error no valid directory, turning off file save for {obj.name}')
                    return      #   If fail, return
            try:
                obj.filename_string=datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+"_"+self.file_descriptor_text.get()+obj.filename_suffix
                if obj.filename_extension == "csv":
                    obj.file_path = os.path.normpath(folder_path + "/"+ obj.filename_string+'.csv')
                    obj.file_object = open(obj.file_path, 'a', newline='', encoding='UTF-8')    #   Open file in append mode; need newline='' to avoid extra CR on Windows
                    obj.file_object.write(obj.file_header_string + "\n")
                elif obj.filename_extension == "txt":
                    obj.file_path = os.path.normpath(folder_path + "/"+ obj.filename_string+'.txt')
                    obj.file_object = open(obj.file_path, "a", encoding='ascii')     #   Open file with append mode
                else:
                    print("Invalid file type with extension", obj.filename_extension)
                if obj.name == "Metadata":
                    self.write_metadata_to_file(obj, "Opening File")
                obj.file_is_open = True
            except BaseException as error_message:
                print(f'Error opening file: {obj.name}, Error message: {error_message}')
                obj.file_is_open = False        
#            print(obj.file_object, obj.file_path)
        else:
            obj.file_is_open = False
            print(f'Error: Trying to open file for {obj.name}, but status is set to close')
            return

    def close_file(self, obj):
        """Close file and clean up"""
        if not obj.file_is_open:
            print(f'Error: Received close requested for {obj.name} file, but file is not open')
            return
        if hshs_constants.DEBUG: print("Closing file for ", obj.name, obj.file_path)
        if obj.name == "Metadata":
            self.write_metadata_to_file(obj, "Closing File")    #   Write post-run metadata to file before closing
        try:
#            if obj.filename_extension != "tif":   #   .csv and .txt files, these should be explicitly closed
            if obj.file_object:
                obj.file_object.close()
                obj.file_object = None
                obj.file_path = None
                obj.file_is_open = False    #   We are not changing button value. For entry from GUI, this has already been changed; for entry from shutdown, the value doesn't matter
            else:
                print("File object not found for: ", obj.name, obj.file_path)
        except BaseException as error_message:
            print(f'Error closing file: {obj.name}, Error message: {error_message}')
            obj.file_is_open = True       

    def write_metadata_to_file(self, obj, state):
        """Write metadata to file"""
        if obj.file_object is not None:
            for next_obj in self.controller.json_obj_list:
                obj.file_object.write(next_obj.label_name + "," + str(next_obj.value) + "," + str(next_obj.category) + "," + state + "\n")
#            print(f' zero value array len: {len(self.controller.mcu_obj.zero_cal_values)}')
#            obj.file_object.write("PD 1 Zero," + str(self.controller.mcu_obj.zero_cal_values[0]) + ", PD Zero Levels,"+ state + "\n")
#            obj.file_object.write("PD 2 Zero," + str(self.controller.mcu_obj.zero_cal_values[1]) + ", PD Zero Levels,"+ state + "\n")
            obj.file_object.write("MCU Git Version," + str(self.controller.mcu_obj.git_version) + ",System Parameters,"+ state + "\n")
            obj.file_object.write("Python Git Version," + get_git_revision_short_hash() + ",System Parameters,"+ state + "\n")

    def handle_file_button_key_press(self, obj):
        """Handle button that opens or closes a file and requests status change to be performed in file save thread, avoiding synchronization issues. """
        obj.toggle()
        if hshs_constants.DEBUG: print(f'Changing file status for {obj.name} to {obj.value}')
#        print(f'Changing file status for {obj.name} to {obj.value}')
        if obj.name == "All Data":  #   Updates for all files
            for file_obj in self.file_objects:
                if file_obj.value != obj.value:
                    self.handle_file_button_key_press(file_obj)
        else:
            self.queue_file_status_change(obj)  #   Single file change

    def queue_file_status_change(self, obj):
        """Open or close file by adding element to queue"""
        next_action = "Open" if obj.value else "Close"
        file_queue_element = FileQueueAction(obj,next_action)
        self.file_data_q.put(file_queue_element)
        if hshs_constants.DEBUG: print(f'Queueing file change for: {obj.name} with new open status: {obj.value}')

    def browse_directory(self):
        """Browse for folder to store data."""
        if hshs_constants.DEBUG: print("In browse for directory")
        self.controller.gui_obj.warning_message_box_with_focus_return("Warning, can't open directory", "Please select directory")
        directory = filedialog.askdirectory()
        if hshs_constants.DEBUG: print("directory = ", directory)
        if not directory:
            self.controller.focus()     #   Release focus after askdirectory
            return False
        else:
            self.folder_name_obj.update_param(directory)
            self.controller.focus()     #   Release focus after askdirectory
            return True

    def run_file_thread(self):
        """Handle file thread actions, including opening/closing files, writing data to files, and closing thread"""
        continue_thread = True
        while continue_thread:
            while self.file_data_q.qsize() > 0:      #   Process queue elements
                next_element = self.file_data_q.get(block=True, timeout=None)
                if next_element.action:     #   action attribute has flag to open or close file or to end file queue
                    if hshs_constants.DEBUG: print(f'Action in data queue: {next_element.action}')
                    if next_element.action == "Open":     #   Element has flag to open or close file
                        self.open_file(next_element.obj)
                        if hshs_constants.DEBUG: print(f'Open file: {next_element.obj.name}')
                    elif next_element.action == "Close":
                        self.close_file(next_element.obj)    #   Close individual open files
                        if hshs_constants.DEBUG: print(f'Queue close for file: {next_element.obj.name}')
                    elif next_element.action == "CloseFileThread":
                        if hshs_constants.DEBUG: print('Closing all data files and stopping file save queue')
                        for obj in self.file_objects:
                            if obj.file_is_open:
                                self.close_file(obj) 
                        if hshs_constants.DEBUG: print('Closing file queue')
                        continue_thread = False     #   Close file queue
                        all_files_closed = not any([obj.file_is_open for obj in self.file_objects])  #   Any returns True if any item in list is True, we should find all files are closed
                        if hshs_constants.DEBUG: print (f'All files closed status: {all_files_closed}')
                        if not all_files_closed:
                            print (f'Warning, all files closed status: {all_files_closed}')
                            print (f'Files still open: {[obj.name for obj in self.file_objects if obj.file_is_open]}')
                    else:
                        if hshs_constants.DEBUG: print("Unrecognized file action: ", next_element.action)
                else:   #   Case with no action is to save data. File object (next_file_object) is identified by attribute "name"
                    next_file_object = next((x for x in self.file_objects if x.name == next_element.name), None)
                    if next_file_object is not None: #skip over if got default None instead of object to process
                        if (next_file_object.name == "CRISP Data") or (next_file_object.name == "Fit Data") or (next_file_object.name == "Responses"):
                            if next_file_object.file_is_open:      #   value = True means file is open; write data
                                if not next_file_object.file_object is None:    #   For these data types, open created a file_object
                                    try:
                                        next_file_object.file_object.write(next_element.data + "\n")
                                    except BaseException as error_message:
                                        print(f'Error writing file: {error_message}, name: {next_file_object.name}, file_object: {next_file_object.file_object}')
                        else:
                            print(f'No valid type for file object: {next_file_object.name}, key: {next_file_object.name}')
                    else:
                        print(f'Error: No match found for file object with name: {next_element.name}')
            time.sleep(0.25)        # Sleep is OK here because it is in a separate thread???
        if hshs_constants.DEBUG: print("Ending file thread")

def get_git_revision_hash() -> str:     #   Retrieve git version for this python code; this will go in the metadata file
    return subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode('ascii').strip()

def get_git_revision_short_hash() -> str:
    return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('ascii').strip()

