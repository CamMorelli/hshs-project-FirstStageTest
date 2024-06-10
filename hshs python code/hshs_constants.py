#   Constants for pid controller
"""Module contains constants for pid code"""

#enum stateType {Standby = 0, Ready = 1, Reset = 2, Sensor_Off = 3, Scan_Init = 5, Scan_On = 6, Shutdown_Init = 7, Shutdown = 8};

#STATE_LIST = ('Standby', 'Ready', 'Reset', 'Data Off', 'Scan Init', 'Scan On', 'Shutdown_Init', 'Shutdown')
STATE_LIST = ('Standby', 'Ready', 'Reset', 'Data Off', 'Scan Init', 'Scan On', 'Shutdown_Init', 'Shutdown', 'Zero_Init', 'Zero_Cal')
#enum stateType {Standby = 0, Ready = 1, Reset = 2, Data_Off = 3, Scan_Init = 4, Scan_On = 5, Shutdown_Init = 6, Shutdown = 7, Zero_Init = 8, Zero_Cal = 9};

PLOT_TYPE = {'Standby': 0, 'Ready' : 0,'Reset' : 0, 'Data Off' : 0, 'Scan Init' : 0, \
             'Scan On' : 0, 'Shutdown_Init': 0, 'Shutdown' : 0, 'Zero_Init' : 0, 'Zero_Cal' : 0}
NUM_CHANNELS = 4
IMAGE_DISPLAY_INTERVAL = 200         #   Interval in ms for image display update
RESPONSE_HISTORY_INTERVAL = 200      #   Interval in ms for updating response history
PLOT_INTERVAL = 200                 #   Interval in ms for one plot update
LED_FLASH_INTERVAL = 1000        #   Interval in ms for LED flashing
SERVO_OVERRANGE_DELAY = 30    # Seconds between servo overrange messages
CONTROLLER_STATE = ["Program Started", "Serial Started", "Microcontroller Started", "Microcontroller Stopped", "Serial Stopped", "Program Stopped"]
FIT_PLOT_DICTIONARY = {3: 0, 4: 1, 5: 2, 7: 3}
NUM_FIT_PLOTS = 3       # Number of plots on fit page
FRAME_PAD = 5       #   Padding between GUI frames
ENTRY_WIDTH = 7     #   Width for GUI entry boxes
ENTRY_PAD = 1       #   Padding between GUI entries
ARROW_WIDTH = 1     #   Width for arrow buttons
DEFAULT_FONT_FAMILY = "Helvetica"
DEFAULT_FONT_SIZE = 9
TITLE_FONT_SIZE = 16
TITLE_WIDTH = 20
LOCK_LOW_REPORT_THRESHOLD = 0.5
LOCK_HIGH_REPORT_THRESHOLD = 5
PLOT_DATA_SIZE = 3000 # This is enought points for 10 minutes of data for REPORT_INTERVAL_SEC = 0.2 (set in C++)
#PLOT_DATA_SIZE = 300 # This is enought points for 10 minutes of data for REPORT_INTERVAL_SEC = 0.2 (set in C++)
CRISP_LED_VALUE_CONTROL= "CRISP LED VALUE CONTROL"
CRISP_LED_CONTROLS = "CRISP LED Controls"
SERVO_VALUE_CONTROL = "Servo Value Control"
SET_POINT_CONTROL = "Set Point Control"
HSHS_GAIN_CONTROLS = "PID Gain Controls"
HSHS_DIRECTION_CONTROL = "PID Direction Control"
HSHS_DIFF_TIME_CONSTANT = "PID Diff Time Constant"
LOCK_ON_OFF_STATUS = "Lock On Off Status"
SCAN_PARAMETERS_CONTROL = "Scan Parameters Control"
DEBUG_COUNT_VALUE = "Debug Count Value"
SENSOR_ON_OFF_STATUS = "Sensor On Off Status"
GLOBAL_PWM_VALUES_CONTROL = "Global PWM Values Control"
LASER_PWM_VALUES_CONTROL = "Laser PWM Values Control"
TEC_PWM_VALUES_CONTROL = "TEC PWM Values Control"
FAN_PWM_VALUE_CONTROL = "Fan PWM Value Control"
TEMPERATURE_SETTINGS_CONTROL = "Temperature Settings Control"
TEMPERTURE_LOCK_ON_OFF_STATUS = 'Temperature Lock On Off Status'
DEBUG = True