import os
print()

import yarp
import time
import csv
import sys

height_FT_sensor = 0.03225
period = 0.002
z_com = 0.683
gravity = 9.80665

directory = os.path.dirname(os.path.realpath(__file__))+'/data'
print(directory)
if __name__ == "__main__":
    if len(sys.argv) ==2:        
        yarp.Network.init()

        if yarp.Network.checkNetwork() != True:
            print('[error] Please try running yarp server')
            quit()

        options = yarp.Property()
        options.put('device','analogsensorclient')
        options.put('remote','/jr3/ch0:o')
        options.put('local','/jr3/ch0:i')
        ddRight = yarp.PolyDriver(options)  # calls open -> connects

        if not ddRight.isValid():
            print('Cannot open the device!')
            quit()

        optionsLeft = yarp.Property()
        optionsLeft.put('device','analogsensorclient')
        optionsLeft.put('remote','/jr3/ch1:o')
        optionsLeft.put('local','/jr3/ch1:i')
        ddLeft = yarp.PolyDriver(optionsLeft)  # calls open -> connects

        if not ddLeft.isValid():
            print('Cannot open the device!')
            quit()

        iAnalogSensorChO_rightLeg = ddRight.viewIAnalogSensor()
        iAnalogSensorCh1_leftLeg = ddLeft.viewIAnalogSensor()

        optionsImu = yarp.Property()
        optionsImu.put('device','multipleanalogsensorsclient')
        optionsImu.put('remote','/teo/imu')
        optionsImu.put('local','/teo/imu:i')
        ddImu = yarp.PolyDriver(optionsImu)  # calls open -> connects

        if not ddImu.isValid():
            print('Cannot open the device!')
            quit()

        # iThreeAxisGyroscopes = dd.viewIThreeAxisGyroscopes()
        iThreeAxisLinearAccelerometers = ddImu.viewIThreeAxisLinearAccelerometers()

        # The following delay should avoid 0 channels and bad read
        print('delay(1)')
        time.sleep(2)

        channelsRightLeg = iAnalogSensorChO_rightLeg.getChannels()
        print('channels:', channelsRightLeg)
        channelsLeftLeg = iAnalogSensorCh1_leftLeg.getChannels()
        print(iThreeAxisLinearAccelerometers.getThreeAxisLinearAccelerometerStatus(0))
        acc = yarp.Vector(3) # m^2/s


        # Of course we dislike while(1)
        step = 0
        start = time.time()
        sum_x_zmp = 0
        offs_x_ft = 0
        x_com = 0
        zero_time = 0
        

        with open(directory+sys.argv[1], 'w') as f:
            writer = csv.writer(f)
            header = ['time', 'com_x', 'zmp_x']
            # write a row to the csv file
            writer.writerow(header)

            while 1:
                
                vectorRightFT = yarp.Vector(channelsRightLeg)
                vectorLeftFT = yarp.Vector(channelsLeftLeg)
                iAnalogSensorChO_rightLeg.read(vectorRightFT)
                iAnalogSensorCh1_leftLeg.read(vectorLeftFT)
                if iThreeAxisLinearAccelerometers.getThreeAxisLinearAccelerometerStatus(0):
                    iThreeAxisLinearAccelerometers.getThreeAxisLinearAccelerometerMeasure(0, acc)
                xzmp_ft0 = -(((vectorRightFT[4]/10) + height_FT_sensor*vectorRightFT[0])) / vectorRightFT[2] # xzmp0_ft in [m] right foot
                xzmp_ft1 = -(((vectorLeftFT[4]/10) + height_FT_sensor*vectorLeftFT[0])) / vectorLeftFT[2] # xzmp1_ft in [m] left foot

                

                xzmp_ft = (xzmp_ft0 * vectorRightFT[2] + xzmp_ft1 * vectorLeftFT[2]) /(vectorLeftFT[2]+vectorRightFT[2])
                ddx = acc[0]
                ddz = acc[2]
                
                step+=1
                if step<200:
                    sum_x_zmp+=xzmp_ft
                    offs_x_ft = sum_x_zmp/step
                else:
                    if step == 200:
                        zero_time = time.time()
                    xzmp_ft -= offs_x_ft 
                    if(ddz!=0):
                        x_com = xzmp_ft + z_com/ddz*ddx #compute com from zmp and linear acceleration
                    print("step: ", step, "xmp: ", xzmp_ft)
                    print("ddx: ", acc[0], " xcom: ", x_com)
                    row = [time.time()-zero_time, x_com, xzmp_ft]
                    writer.writerow(row)
            
                time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))
    else:
        print('python3 computeCOM.py csv_filename.csv')
    print('computeCOM.py bye!')
