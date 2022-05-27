import csv

from os import listdir
from os.path import isfile, join

mypath = "/home/user/catkin_ws/src/eurobench_tests/data/zmp_ref_ankles"
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]

print(onlyfiles)


# create the csv writer
header = ['test_file', 'Zmp_reference_[m]', 'Starting_Row_Zmp_Reference', 'period_[ms]', 'Duration_Step_rows', 'Column_zmp_ref', 'Column_zmp_sensors']
data = [ ]

for i in range(1,9):
    startExperiment = 0
    with open(mypath+'/test'+str(i)+'.csv') as csv_file:
        print('test'+str(i)+'.csv')
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print(f'Column names are {", ".join(row)}')
            else:
                print(row)
                if (float(row[10]) != 0.0):
                    startExperiment = line_count
                    print("ref: ", row[10])
                    break
            line_count += 1
        print(f'Processed {line_count} lines.')
    data.append(['test'+str(i)+'.csv', i/100, startExperiment, 1/50.0*1000.0, 30, 'zmp_ref', 'Xzmp_ft'])


# open the file in the write mode
with open('/home/user/catkin_ws/src/eurobench_tests/data/zmp_ref_ankles/ref_tests.csv', 'w', encoding='UTF8', newline='') as file:
    writer = csv.writer(file)

    # write the header
    writer.writerow(header)

    # write multiple rows
    writer.writerows(data)

