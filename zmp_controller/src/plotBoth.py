


# import bagpy
# from bagpy import bagreader
# import pandas as pd
# import seaborn as sea
# from cProfile import label
# from tkinter import Label
import matplotlib.pyplot as plt
import numpy as np

import csv

names = ['03', '08']
for name in names:
    n = 40
    time = []
    ref = []
    x_zmp = []
    with open('/home/elisabeth/repos/eurobench_tests/data/real_reemc/ankles_commands_tests/csv/zmp_with_simulation_params/test0_'+name+'.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print('Column names are ')
            else:
                time.append(float(row[0]))
                ref.append(float(row[10]))
                x_zmp.append(float(row[7]))
            line_count += 1

    str_label = "DLIPM 0."+name
    plt.plot(time, x_zmp, '--',label= str_label)

    time.clear()
    ref.clear()
    x_zmp.clear()
    with open('/home/elisabeth/repos/eurobench_tests/data/real_reemc/ankles_commands_tests/csv/zmp_refs_longer_time/test0_'+name+'.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print('Column names are}')
            else:
                time.append(float(row[0]))
                ref.append(float(row[10]))
                x_zmp.append(float(row[7]))
            line_count += 1

    str_label = "Original 0." +name
    plt.plot(time, x_zmp,'--', label= str_label)

    plt.plot(time,ref)
plt.legend()
plt.show()

# plt.plot(df_zmp_ref['Time'][0:700]-df_zmp_ref['Time'][0], df_zmp_ref['point.x'][0:700])
# plt.plot(df_zmp['Time'][0:700]-df_zmp['Time'][0], df_zmp['point.x'][0:700])
# plt.plot(df_zmp_controller['Time'][0:700]-df_zmp_controller['Time'][0], df_zmp_controller['point.x'][0:700])
# plt.ylim([-0.1, 0.1])
# plt.show()
# plt.savefig("mygraph.png")
# # quickly plot velocities
# b.plot_vel(save_fig=True)

# # you can animate a timeseries data
# bagpy.animate_timeseries(veldf['Time'], veldf['linear.x'], title='Velocity Timeseries Plot')