import numpy as np
import hshs_plot_class

#   Test hshs_plot_class.shift_data

array_len = 10
for shift in [-1, 0, 9, 10, 11]:
    in_array = np.asarray([ii*1.0 for ii in range(array_len)])     #   Function changes array; need to recreate it on every iteration
    hshs_plot_class.shift_data(in_array,shift)
    print ('shift = ', shift, 'array = ', in_array)