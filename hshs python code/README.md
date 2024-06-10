# Teensy CRISP python

## Name
Teensy CRISP python

## Known Bugs

## Testing
Tested on Python 3.10

## Description
<!-- Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors. -->

Handling widget events:
The program responds to both shortcut keys and clicking widgets. The supporting code is primarily in hshs_gui_interface
Shortcut keys (in key_list) are bound to the function handle_global_key_press, which then calls functions by key
Clicking widgets results in function calls, set up during creation using "command = "
<KeyRelease> is bound to function handle_entry_key_press as follows:
For 'Up' and 'Down' arrows, current value (text) is tested for validity for arrow changes, then increamented/decremented based on cursor position
'Return' or 'KP_Enter' call focus to the main controller (focus out for the widget); tab also generates a focus out withut this step
<FocusOut> is bound to handle_focus_out, which also updates widgets and calls functions
Both 'Up' and 'Down' (handle_entry_key_press, but no FocusOut) and <FocusOut> update widgets through:
(1) update_param method for EntryParam Class: does validity check, then updates stored variable and widget display
(2) calls to update_plot_limits (axis values) or update_mcu (microcontroller values) to make changes 

To add to GUI:
    Add frame for GUI elements in pcr_controller_new_start in Class init for proper page (Settings, Data, or Fits)
    create method to populate widgets (e.g., roi_centers_controls_widgets, pwm_controls_widgets
    Call method from main program (pcr_controller_new_start)
    Define value, default value, max, min, etc for each widget
    Each element will reference the frame from first step
    Include list of widgets for (1) inclusion of current values in config.json file and (2) loop creation such as by gui_obj.create_widgets or gui_obj.create_buttons
    Add above list to json_obj_list

To add states:
    Edit state_control.hpp and hshs_constants.py to add new state to stateType and STATE_LIST and PLOT_TYPE, respectively
    Add state button in self.button_type in mcu_comm, and 
    Add state shortcut to key_list in hshs_controller and handle_mcu_command in mcu_comm
    Edit serial_comm.cpp to add relevant commands to help list and command call to processInputLine
    Edit process.hpp and process.cpp to add function to handle command
    Edit state_control.hpp to handle appropriate case(s) in updateState

## Files
hshs_controller_new-start.py: Starts up program, handling the creation and eventual destruction of Tkinter GUI
hshs_mcu_comm.py: Handles reading and writing from port (read from port thread) as well as interaction between user and GUI
hshs_plot_class.py: Handles details of displaying images and other data on GUI
hshs_gui_interface.py: Basic functionality of interacting with GUI like creating buttons and handling key presses
hshs_file_save.py: Handle file save thread, saving data to files
hshs_constants.py: Center for constants to be used in dimenions of GUI

## Visuals
<!-- Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method. -->

## Installation

You will need to install several modules (e.g. using pip) including: serial, pyserial

<!-- Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection. -->

## Usage
<!-- Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README. -->


## Support
<!-- Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc. -->


## Roadmap
<!-- Ideas for the future. -->

Update command list, implement/repurpose commands, implement/repurpose return values

States: Lock, ready (no servo), manual servo, scan, off (LED off)

Commands: Lock, Ready, Manual, scan (limits, time), servo level (value), LED level (value), servo off, gain values
Error - servo at limit
Limit servo rate of change
Add PID/Lock

Outputs to MCU: state changes, live servo voltage, LED level
Returns from MCU: two photodiode signals, error signal, servo voltage, state

Consider what warnings/confirmations you want to pop up when the user tries to do something that is unavailable or risky.

Add an option for pausing collection.

To get more control over the placement of elements on the Data Page, one might consider breaking up the display frame into more rows and columns and just increasing the span of those large elements that are currently there.

Some reorganization across classes and files could make the code more digestible. pcr_mcu_comm.py is especially long
## Contributing
<!-- State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser. -->

## Authors and acknowledgment
Gregory Faris


## License
<!-- For open source projects, say how it is licensed. -->

## Project status
<!-- If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers. -->
