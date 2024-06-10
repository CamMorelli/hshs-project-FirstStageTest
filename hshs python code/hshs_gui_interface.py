"""GUI interface pieces."""
import tkinter as tk
import re   #   Regular expression for checking valid data entry
import json
import hshs_constants        #       Local python constants
import hshs_ms2k_interface as ms2k


#   Note: Mac does not support button color changes: https://github.com/python/cpython/issues/88409

def good_regex(string):
    """Check whether string is int or float"""
    regex_any = '^-?[0-9]+\\.?[0-9]*$'   #   Regular expression for int or float: 0 or 1 minus signs; 1 or more digits; 0 or 1 decimal points; 0 or more digits
    return re.search(regex_any, string)

class NumberEntry(tk.Entry):
    """Extended tk.Entry class includes methods for numerical value entry"""
    def __init__(self, parent, controller, row, column, value, default, minval, maxval, label_name, category, command, argument, *args, **kwargs):
        tk.Entry.__init__(self, parent, *args, **kwargs)
        self.value = value
        self.default = default
        self.minval = minval
        self.maxval = maxval
        self.label_name = label_name
        self.name = label_name
        self.focus_on = False
        self.category = category
        self.row = row
        self.column = column
        self.command = command
        self.argument = argument
        self.controller = controller
        self.cursor_position = 1    #   Tracks starting cursor position for up/down arrow value changes
        self.prior_string_len = 0
        self.last_param_update = ''
        self['font'] = self.controller.widget_basic_font
        self['width'] = hshs_constants.ENTRY_WIDTH
        self['justify']='center'

        # Don't include label if given blank label name
        if (label_name == ""):
            self.grid(row=self.row, column=self.column, rowspan=1, sticky='nsew', padx = hshs_constants.ENTRY_PAD, pady = hshs_constants.ENTRY_PAD)
        else:
            tk.Label(parent, text = self.label_name, font=self.controller.widget_basic_font).grid(row=row, column = column)
            self.grid(row=self.row + 1, column=self.column, sticky='nsew', padx = hshs_constants.ENTRY_PAD, pady = hshs_constants.ENTRY_PAD)
        
        self.insert(0,self.value)
        self.bind('<KeyRelease>',lambda event: self.handle_entry_key_press(event, self))    # '<Tab>' is apparently not a '<Key>', it does invoke FocusOut, though
        self.bind('<FocusIn>',lambda event: self.handle_focus_in(event, self))      #   Handle key focus in for PWM/Fan/ROI updates
        self.bind('<FocusOut>',lambda event: self.handle_focus_out(event, self))      #   Handle key focus out for PWM/Fan/ROI updates

    def update_param(self, new_value):
        """Update both value and GUI display for NumberEntry. Currently new_value argument may be either a string or number (int or float)"""
        try:                #   Try catches invalid number entries
            new_value = max(float(new_value),float(self.minval))    #   Coerce number into range
            new_value = min(float(new_value),float(self.maxval))
            if str(self.default).find(".") == -1:  #   Coerce type based on presence of decimal point in _entry_param.default
                new_value=int(float(new_value))     #   Apparently, int cannot convert a float string
            else:
#                print(f'New value is float: {new_value}, {float(new_value)}')
                new_value=float(new_value)
            if abs(new_value) < 0.0001: new_value = 0   #   This prevents display in scientific notation on GUI, may want to revisit this later
            self.value = new_value
            self.delete (0, tk.END)
            self.insert(0, self.value)
#            temp_string = "{:f}".format(self.value)
#            print(f'temp_string: {temp_string}')
#            self.insert(0, f'{self.value}:.f')
#            self.insert(0, self.value)
        except ValueError:      #   Invalid string in GUI entry; update GUI string from prior (validated) value attribute instead of from new_value argument
            print(new_value, "is not valid expression for parameter", self.label_name)
            self.delete (0, tk.END)  #   Display previous value
            self.insert(0, self.value)
        if self.focus_on:   #   Restore cursor position when responding to up/down arrows
            self.icursor(self.cursor_position+len(str(self.value))-self.prior_string_len)   #    Move cursor to correct position after arrow
        new_string = self.get()
#        print(f'Created new value for {self.name}: {new_string !=  self.last_param_update}, Old string: { self.last_param_update}, New String {new_string}')
        if hshs_constants.DEBUG: print(f'Created new value for {self.name}: {new_string !=  self.last_param_update}, Old string: { self.last_param_update}, New String {new_string}')
        string_changed = new_string != self.last_param_update
        self.last_param_update = new_string
        return string_changed  #   Returns true when value has changed
    
    def handle_entry_key_press(self, event, _entry_param):
        """Process key entry for validity (Return) or validity and increment/decrement (Up/Down)"""
        in_string = _entry_param.get()        #   N.B. event.widget.get does not contain latest key press; however, this does not affect this method
        self.prior_string_len = len(in_string)
        if hshs_constants.DEBUG: print(f'Handling entry key press for key: {event.keysym} in entry: {_entry_param.label_name} with widget value: {event.widget.get()} entry value: {_entry_param.value} '
              f'and type:{type(_entry_param.value)}')
        if event.keysym == 'Return' or event.keysym == 'KP_Enter' : #   Tab event is handled by focusout? validity is checked in update_param method
            self.controller.focus()     #   Generate FocusOut event, which will update paramater value and mcu from there
#        elif event.keysym == '<Button-1>':
#            print('Mouse click, ', _entry_param.name)  #   Doesn't work
        elif (event.keysym == 'Up' or event.keysym == 'Down') and good_regex(in_string):
            self.cursor_position=event.widget.index(tk.INSERT)        #   Cursor position is used even for invalid entry (restore prior value and cursor position)
            decimal_position = in_string.find(".")
            sign = 1 if event.keysym=='Up' else -1  #   Determine sign for Up and Down movement
            if decimal_position == -1:             #   No decimal point -> increment/decrement for integer type
                out_value = int(int(in_string)+sign*10**(self.prior_string_len-self.cursor_position))
            else:                                   #   Decimal point -> float type, handle increment/decrement for left and right of decimal place differently
                if self.cursor_position<= decimal_position:
                    out_value = float(in_string)+sign*10**(decimal_position-self.cursor_position)
                else:
                    out_value = float(in_string)+sign*10**(decimal_position+1-self.cursor_position)
                out_value = round(out_value, self.prior_string_len-decimal_position-1)     #   Remove extra decimal places from calculation
            if _entry_param.update_param(out_value):            #   Update parameter (both GUI entry string and value attribute), but only update mcu if value has changed
                if self.command:
                    if not self.argument:
                        self.command()
                    elif self.argument == 'self':
                        self.command(self)
                    else:
                        self.command(self.argument)

    def handle_focus_in(self, event, _entry_param):
        """Establish new entry parameter for focus in"""
        _entry_param.focus_on = True
        if hshs_constants.DEBUG: print("Focus In: ", _entry_param.label_name, event.widget.get())
        self.focus_in_entry_param = _entry_param    #   Might use later

    def handle_focus_out(self, event, focus_out_obj):
        """Process entry parameter when focus is removed"""
        focus_out_obj.focus_on = False
        if hshs_constants.DEBUG: print("Focus Out: ", focus_out_obj.label_name, focus_out_obj.get(), event.widget.get())
        self.focus_in_entry_param = None
        if focus_out_obj.update_param(focus_out_obj.get()):    #   Although there is already a string in entry, this validates value, adds to value attribute, 
            if self.command:                                   #    and updates mcu is value has changed
                if not self.argument:
                    self.command()
                elif self.argument == 'self':
                    self.command(self)
                else:
                    self.command(self.argument)
class TextEntry(tk.Entry):
    """Extended tk.Entry class includes methods for text entry"""
    def __init__(self, parent, controller, row, column, width, value, label_name, entry_name, category, *args, **kwargs):
        tk.Entry.__init__(self, parent, *args, **kwargs)
        self.value = value
        self.controller = controller
        self.row = row
        self.column = column
        self.width = width
        self.default = ""
        self.string_var = tk.StringVar()
        self.string_var.set(value)
        self.label_name = label_name
        self.name = label_name
        self.entry_name = entry_name
        self.focus_on = False
        self.category = category
        self['font'] = self.controller.widget_basic_font
        self['width'] = self.width
        self['textvariable'] = self.string_var
        tk.Label(parent, text = self.label_name, font=self.controller.widget_basic_font).grid(row=row, column = column)
        self.grid(row=self.row + 1, column=self.column, sticky='nsew', padx = hshs_constants.ENTRY_PAD, pady = hshs_constants.ENTRY_PAD)

    def update_param(self, text):
        """Update values in TextEntry"""
        self.value = text
        self.string_var.set(text)
class ToggleButton(tk.Button):
    """Extended tk.Button class for toggle-type switch buttons"""
    def __init__(self, parent, controller, row, column, state, default, on_text, off_text, label_name, category, show_label, command, argument, *args, **kwargs):
        tk.Button.__init__(self, parent, *args, **kwargs)
        self.controller = controller
        self.value = state
        self.name = label_name
        self.default = default
        self.on_text = on_text
        self.off_text = off_text
        self.label_name=label_name
        self.category = category
        self.show_label = show_label
        self.row = row
        self.column = column
        self.command = command
        self.argument = argument
        if command:
            if not argument:
                self['command'] = self.command
            elif argument == 'self':
                self['command'] = lambda: self.command(self)
            else:
                self['command'] = lambda: self.command(self.argument)
        else:
            self['command'] = self.toggle
        self['font'] = self.controller.widget_basic_font
        self['width'] = hshs_constants.ENTRY_WIDTH
        self['padx'] = hshs_constants.ENTRY_PAD
        self['pady'] = hshs_constants.ENTRY_PAD
        if self.show_label:
            tk.Label(parent, text = self.label_name, font=self.controller.widget_basic_font).grid(row=row, column = column)
            self.grid(row=self.row + 1, column=self.column, sticky='nsew')
        else:
            self.grid(row=self.row, column=self.column, rowspan = 2, sticky='nsew')
        if self.value:
            self['text']=self.on_text
        else:
            self['text']=self.off_text

    def update_param(self, value):
        """Update both value and displayed GUI button text"""
        self.value=value
        self['text']=self.on_text if self.value else self.off_text

    def toggle(self):
        """Update following key press"""
        self.value = not self.value
        self.update_param(self.value)
        if hshs_constants.DEBUG: print (f'Toggling {self.name}, new value = {self.value}')

class StateButton(tk.Text):
    """Extended tk.Button class for toggle-type switch buttons"""
    #   Note: Mac does not support button color changes: https://github.com/python/cpython/issues/88409 This uses and Entry to act as a button
    #   Entry widgets do not support multiple lines of text. Hence using tk.Label here.
    #   Label widgets do not support key focus. 
    #   Text widgets do not support StringVar variables, Try text widgets?
    def __init__(self, parent, controller, row, column, state, default, on_text, off_text, label_name, category, show_label, command, argument, *args, **kwargs):
        tk.Text.__init__(self, parent, *args, **kwargs)
        self.controller = controller
        self.value = state
        self.name = label_name
        self.default = default
        self.on_text = on_text
        self.off_text = off_text
#        self.string_var = tk.StringVar()
        self.label_name=label_name
        self.category = category
        self.show_label = show_label
        self.row = row
        self.column = column
        self.command = command
        self.argument = argument
        self['font'] = self.controller.widget_basic_font
#        self['width'] = hshs_constants.ENTRY_WIDTH
        self['width'] = 12
        self['height'] = 2
        self['borderwidth'] = 2
        self['relief'] = 'raised'

        self.tag_configure("tag_name", justify='center')    #   This odd construction is how to center text in text widget
        self.insert(1.0, 'start text')
        self.tag_add("tag_name", "1.0", "end")
#            self.add_text(self.on_text)

#        self.tag_add("center", "1.0", "end")
#        self['textvariable'] = self.string_var
        self.grid(row=self.row, column=self.column, rowspan = 2, sticky='nsew', padx = hshs_constants.ENTRY_PAD, pady = hshs_constants.ENTRY_PAD)

        self.update_param(self.value)

        self.bind('<FocusOut>',lambda event: self.handle_focus_out(event, self))      #   Handle key focus out for PWM/Fan/ROI updates

    def add_text(self, new_text):
        self.delete ("1.0", "end")
        self.insert("1.0", new_text)
        self.tag_add("tag_name", "1.0", "end")

    def update_param(self, value):
        """Update both value and displayed GUI button text"""
        self.value=value
        if self.value:
#            self.string_var.set(self.on_text)
            self.add_text(self.on_text)
            self.configure(font=self.controller.widget_header_font)
            self.configure(background = '#00FF00')
        else:
            self.add_text(self.off_text)
#            self.string_var.set(self.off_text)
            self['font'] = self.controller.widget_basic_font
            self.configure(bg = '#f0f0f0')   #   Set to default background color

    def toggle(self):
        """Update following key press"""
        self.value = not self.value
        self.update_param(self.value)

    def handle_focus_out(self, event, focus_out_obj):
        """Process entry parameter when focus is removed"""
        if hshs_constants.DEBUG: print("Focus Out: ", focus_out_obj.label_name, focus_out_obj.get(1.0, tk.END), event.widget.get(1.0, tk.END))
        if self.command:                                   #    and updates mcu is value has changed
            if not self.argument:
                self.command()
            elif self.argument == 'self':
                self.command(self)
            else:
                self.command(self.argument)

class EntryAsDisplay(tk.Entry):
    """Extended tk.Entry class for toggle-type switch buttons"""
    def __init__(self, parent, controller, row, column, label_name, *args, **kwargs):
        tk.Entry.__init__(self, parent, *args, **kwargs)
        self.controller = controller
        self.name = label_name
        self.label_name = label_name
        self.string_var = tk.StringVar()
        self.row = row
        self.column = column
        self['font'] = self.controller.widget_basic_font
        self['width'] = hshs_constants.ENTRY_WIDTH
        self['justify']='center'
        self['textvariable'] = self.string_var
        tk.Label(parent, text = self.label_name, font=self.controller.widget_basic_font).grid(row=row, column = column)
        self.grid(row=self.row + 1, column=self.column, sticky='nsew', padx = hshs_constants.ENTRY_PAD, pady = hshs_constants.ENTRY_PAD)
        self.bind('<Return>', lambda event: self.controller.focus())      #     Allow return to create focus out; Wrapping callback removes event from argument list
        self.bind('<KP_Enter>', lambda event: self.controller.focus())      #     Allow return to create focus out; Wrapping callback removes event from argument list


class SimpleButton(tk.Button):
    """Extended version of tk.Button class"""
    def __init__(self, parent, controller, row, column, text, font, command, argument, *args, **kwargs):
        tk.Button.__init__(self, parent, *args, **kwargs)
        self.controller = controller
        self.row = row
        self.column = column
        self.command = command
        self.text = text
        self.font = font
        self.argument = argument
        self.font = self.controller.widget_basic_font
        if command:
            if not argument:
                self['command'] = self.command
            elif argument == 'self':
                self['command'] = lambda: self.command(self, self)
            else:
                self['command'] = lambda: self.command(self.argument)
        self['padx'] = hshs_constants.ENTRY_PAD
        self['pady'] = hshs_constants.ENTRY_PAD
        self['text'] = self.text
        if font == 'title font front':
            self['font'] = self.controller.title_font_front
            self['width'] = hshs_constants.TITLE_WIDTH
        elif font == 'title font behind':
            self['font'] = self.controller.title_font_behind
            self['width'] = hshs_constants.TITLE_WIDTH
        elif font == 'header font':
            self['font'] = self.controller.widget_header_font
            self['width'] = hshs_constants.ENTRY_WIDTH
        elif font == 'arrow':
            self['font'] = self.controller.title_font_behind
            self['width'] = hshs_constants.ARROW_WIDTH
        else:
            self['font'] = self.controller.widget_basic_font
            self['width'] = hshs_constants.ENTRY_WIDTH

        # have arrow buttons span single row to simplify grid in stage controls
        if font== 'arrow':
            self.grid(row=self.row, column=self.column, rowspan = 1, sticky='nsew')
        else:
            self.grid(row=self.row, column=self.column, rowspan = 2, sticky='nsew')

        # self.grid(row=self.row, column=self.column, rowspan = 2, sticky='nsew')
        
class GUIInterface():
    """Pieces for GUI interface"""
    def __init__(self, controller):

        self.controller = controller
        self.in_data = None
        self.focus_in_entry_param = None    #   This tracks entry parameter for FocusIn/FocusOut

    def read_json_file(self):
        """Read configuration from json file"""
        try:
            in_file = open('config.json', 'r', encoding='ascii')         #       Read configuration from json file
        except OSError as error:
            print(f'Opening json file failed with error: {error}')
            self.in_data = []
        else:
            with in_file:
                self.in_data = json.load(in_file)
#        print ("self.in_data type = ", type(self.in_data))
#        return self.in_data

    def update_from_json_config(self, obj_list, mcu_obj, ms2k_obj):
        """Update parameters from configuration in json file"""
        for obj in obj_list:
#            print(obj, obj.label_name, self.in_data[obj.label_name], obj.label_name)
#            print(obj.label_name, self.in_data[obj.label_name])
            if obj.label_name in self.in_data:
                obj.update_param(self.in_data[obj.label_name])  # Files are now opened after pressing start button in hshs_mcu_comm.py
            else:
                print ("Key: ", obj.label_name, "is missing, use default")
                obj.update_param(obj.default)
#        print([obj.name for obj in mcu_obj.led_list])
#            print(obj, obj.label_name, obj.value)
        if "MCU Port" in self.in_data:
            mcu_obj.select_mcu_port(self.in_data["MCU Port"])
        if "MS2K Port" in self.in_data:
            ms2k_obj.select_ms2k_port(self.in_data["MS2K Port"])

    def dump_json_config(self, obj_list, mcu_obj, ms2k_obj):
        """Write configuration to json file."""
#        print("out_data", obj_list)
#        print ([obj.name for obj in obj_list])
#        for obj in obj_list:
#            print (obj.name)
#            print (obj.label_name)
        out_data = dict([obj.label_name, obj.value] for obj in obj_list)     #   Save configuration data in json file
        if hshs_constants.DEBUG: print ("Closing with mcu_port = ", mcu_obj.mcu_port)
#        print(mcu_obj.mcu_port, mcu_obj.port_successfully_started)
#        out_data.update({"PD 1 Zero": mcu_obj.zero_cal_values[0]})
#        out_data.update({"PD 2 Zero": mcu_obj.zero_cal_values[1]})
#        print (out_data)

        if mcu_obj.mcu_port is not None and mcu_obj.port_successfully_started:   #   Only update MCU Port to current value if it has successfully started port
            out_data.update({"MCU Port": mcu_obj.mcu_port})
#            print(mcu_obj.mcu_port, mcu_obj.port_successfully_started)
        elif "MCU Port" in self.in_data:                             #   Get here if quit before starting port
#            print(self.in_data["MCU Port"])
            out_data.update({"MCU Port": self.in_data["MCU Port"]})

        print(f"ms2k_obj.ms2k_port, {ms2k_obj.ms2k_port}")
        print(f"ms2k_obj.port_successfully_started, {ms2k_obj.port_successfully_started}")
        if ms2k_obj.ms2k_port is not None and ms2k_obj.port_successfully_started:   #   Only update MCU Port to current value if it has successfully started port
            out_data.update({"MS2K Port": ms2k_obj.ms2k_port})
#            print(mcu_obj.mcu_port, mcu_obj.port_successfully_started)
        elif "MS2K Port" in self.in_data:                             #   Get here if quit before starting port
#            print(self.in_data["MCU Port"])
            out_data.update({"MS2K Port": self.in_data["MS2K Port"]})

        # Update visual camera display preferences. Initialized in constructor so never None

        with open('config.json', 'w', encoding='ascii') as out_file:
            json.dump(out_data, out_file, indent=4)

    def handle_global_key_press(self, event, mcu_obj, ms2k_obj):
        """Process global key press"""
        entry_box_focus = str(self.controller.focus_get()).find("entry")>-1
#        print('Focus:', self.controller.focus_get())
#        print(entry_box_focus, event.keysym)
        key_symbol_lower_case = event.keysym.lower()
#        print(key_symbol_lower_case)
        if hshs_constants.DEBUG: print(f'Global key press with focus: {self.controller.focus_get()} and key: {event.keysym} / {key_symbol_lower_case}; focus is entry_box focus: {entry_box_focus}')
        if not entry_box_focus: #   Don't execute if focus is in entry box
            if event.keysym == "bracketleft":
                if hshs_constants.DEBUG: print("Change frame to left")
                self.controller.advance_frame(-1)
            elif event.keysym == "bracketright":
                if hshs_constants.DEBUG: print("Change frame to right")
                self.controller.advance_frame(1)
            elif key_symbol_lower_case == "s":
                mcu_obj.start()
            elif key_symbol_lower_case == "q":
                self.controller.quit()
            elif key_symbol_lower_case == "m":
                ms2k_obj.start_port()
            else:
#            elif key_symbol_lower_case in ["c", "d", "k", "o", "p", "r", "t", "v", "z"]:
                mcu_obj.handle_mcu_command(key_symbol_lower_case)

    def warning_message_box_with_focus_return(self, title, message):
        """Displays messagebox and resets focus after"""
        tk.messagebox.showwarning(title, message)
#        print('\a')    #   In theory, this should make a beep ('bell'), but it does not
        self.controller.after(1, self.controller.focus_force)     #   Reset focus after messagebox
