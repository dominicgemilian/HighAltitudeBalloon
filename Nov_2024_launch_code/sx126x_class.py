import time 
import serial 
import azimuth_elevation_angle as azel
import RPi.GPIO as GPIO
import subprocess
import shlex
from datetime import datetime
import sys

## This class is designed to work with both lora_send and lora_receive components
## Any additional features that needs to be added to the lora_send or lora_receive
## Can be added as a method to this class

class sx126x:

    M0 = 22
    M1 = 27
    cfg_reg = [0xC2,0x00,0x09,0x00,0x00,0x00,0x62,0x00,0x17,0x00,0x00,0x00]
    get_reg = bytes(12)
    rssi = False
    addr = 100
    serial_n = ""
    send_to = 0
    addr_temp = 100
    freq = 868
    power = 22
    air_speed =2400
    HAB_lat = []
    HAB_lon = []
    HAB_time = []
    HAB_alt = []
    SX126X_UART_BAUDRATE_1200 = 0x00
    SX126X_UART_BAUDRATE_2400 = 0x20
    SX126X_UART_BAUDRATE_4800 = 0x40
    SX126X_UART_BAUDRATE_9600 = 0x60
    SX126X_UART_BAUDRATE_19200 = 0x80
    SX126X_UART_BAUDRATE_38400 = 0xA0
    SX126X_UART_BAUDRATE_57600 = 0xC0
    SX126X_UART_BAUDRATE_115200 = 0xE0

    SX126X_AIR_SPEED_300bps = 0x00
    SX126X_AIR_SPEED_1200bps = 0x01
    SX126X_AIR_SPEED_2400bps = 0x02
    SX126X_AIR_SPEED_4800bps = 0x03
    SX126X_AIR_SPEED_9600bps = 0x04
    SX126X_AIR_SPEED_19200bps = 0x05
    SX126X_AIR_SPEED_38400bps = 0x06
    SX126X_AIR_SPEED_62500bps = 0x07

    SX126X_PACKAGE_SIZE_240_BYTE = 0x00
    SX126X_PACKAGE_SIZE_128_BYTE = 0x40
    SX126X_PACKAGE_SIZE_64_BYTE = 0x80
    SX126X_PACKAGE_SIZE_32_BYTE = 0xC0

    SX126X_Power_22dBm = 0x00
    SX126X_Power_17dBm = 0x01
    SX126X_Power_13dBm = 0x02
    SX126X_Power_10dBm = 0x03


    #
    # start frequence of two lora module
    #
    # E22-400T22S           E22-900T22S
    # 410~493MHz      or    850~930MHz
    start_freq = 850

    #
    # offset between start and end frequence of two lora module
    #
    # E22-400T22S           E22-900T22S
    # 410~493MHz      or    850~930MHz
    offset_freq = 18

    # power = 22
    # air_speed =2400

    SX126X_UART_BAUDRATE_1200 = 0x00
    SX126X_UART_BAUDRATE_2400 = 0x20
    SX126X_UART_BAUDRATE_4800 = 0x40
    SX126X_UART_BAUDRATE_9600 = 0x60
    SX126X_UART_BAUDRATE_19200 = 0x80
    SX126X_UART_BAUDRATE_38400 = 0xA0
    SX126X_UART_BAUDRATE_57600 = 0xC0
    SX126X_UART_BAUDRATE_115200 = 0xE0

    SX126X_PACKAGE_SIZE_240_BYTE = 0x00
    SX126X_PACKAGE_SIZE_128_BYTE = 0x40
    SX126X_PACKAGE_SIZE_64_BYTE = 0x80
    SX126X_PACKAGE_SIZE_32_BYTE = 0xC0

    SX126X_Power_22dBm = 0x00
    SX126X_Power_17dBm = 0x01
    SX126X_Power_13dBm = 0x02
    SX126X_Power_10dBm = 0x03

    lora_air_speed_dic = {
        1200:0x01,
        2400:0x02,
        4800:0x03,
        9600:0x04,
        19200:0x05,
        38400:0x06,
        62500:0x07
    }

    lora_power_dic = {
        22:0x00,
        17:0x01,
        13:0x02,
        10:0x03
    }

    lora_buffer_size_dic = {
        240:SX126X_PACKAGE_SIZE_240_BYTE,
        128:SX126X_PACKAGE_SIZE_128_BYTE,
        64:SX126X_PACKAGE_SIZE_64_BYTE,
        32:SX126X_PACKAGE_SIZE_32_BYTE
    }



    def __init__(self,serial_num,freq,addr,power,rssi,air_speed=2400,\
                 net_id=0,buffer_size = 240,crypt=0,\
                 relay=False,lbt=False,wor=False):

        self.rssi = rssi
        self.addr = addr
        self.freq = freq
        self.serial_n = serial_num
        self.power = power
        self.rssi = rssi
        self.addr = addr
        self.freq = freq
        self.serial_n = serial_num
        self.power = power
        self.send_to = addr

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.M0,GPIO.OUT)
        GPIO.setup(self.M1,GPIO.OUT)
        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.HIGH)

        # The hardware UART of Pi3B+,Pi4B is /dev/ttyS0
        self.ser = serial.Serial(serial_num,9600)
        self.ser.flushInput()
        self.set(freq,addr,power,rssi,air_speed,net_id,buffer_size,crypt,relay,lbt,wor)

    def air_speed_cal(self,airSpeed):
        air_speed_c = {
            1200:self.SX126X_AIR_SPEED_1200bps,
            2400:self.SX126X_AIR_SPEED_2400bps,
            4800:self.SX126X_AIR_SPEED_4800bps,
            9600:self.SX126X_AIR_SPEED_9600bps,
            19200:self.SX126X_AIR_SPEED_19200bps,
            38400:self.SX126X_AIR_SPEED_38400bps,
            62500:self.SX126X_AIR_SPEED_62500bps
        }
        return air_speed_c.get(airSpeed,None)

    def power_cal(self,power):
        power_c = {
            22:self.SX126X_Power_22dBm,
            17:self.SX126X_Power_17dBm,
            13:self.SX126X_Power_13dBm,
            10:self.SX126X_Power_10dBm
        }
        return power_c.get(power,None)

    def buffer_size_cal(self,bufferSize):
        buffer_size_c = {
            240:self.SX126X_PACKAGE_SIZE_240_BYTE,
            128:self.SX126X_PACKAGE_SIZE_128_BYTE,
            64:self.SX126X_PACKAGE_SIZE_64_BYTE,
            32:self.SX126X_PACKAGE_SIZE_32_BYTE
        }
        return buffer_size_c.get(bufferSize,None)

    def receive(self):
        #if self.ser.inWaiting() > 0:
        r_buff = ""
        r_buff = self.ser.read(self.ser.inWaiting())
        self.ser.flushInput()
        msg = str(r_buff[2:])
        return msg
    
    def send(self, data):
        # add the node address ,and the node of address is 65535 can konw who send messages
        l_addr = self.addr_temp & 0xff
        h_addr = self.addr_temp >> 8 & 0xff
        self.ser.flushInput()
        self.ser.write(bytes([h_addr,l_addr])+data)
        time.sleep(10)
            
    def free_serial(self):
        self.ser.close()

    def set(self,freq,addr,power,rssi,air_speed=2400,\
            net_id=0,buffer_size = 240,crypt=0,\
            relay=False,lbt=False,wor=False):
        self.send_to = addr
        self.addr = addr
        # We should pull up the M1 pin when sets the module
        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.HIGH)
        time.sleep(0.1)

        low_addr = addr & 0xff
        high_addr = addr >> 8 & 0xff
        net_id_temp = net_id & 0xff
        if freq > 850:
            freq_temp = freq - 850
            self.start_freq = 850
            self.offset_freq = freq_temp
        elif freq >410:
            freq_temp = freq - 410
            self.start_freq  = 410
            self.offset_freq = freq_temp
        
        air_speed_temp = self.lora_air_speed_dic.get(air_speed,None)
        # if air_speed_temp != None
        
        buffer_size_temp = self.lora_buffer_size_dic.get(buffer_size,None)
        # if air_speed_temp != None:
        
        power_temp = self.lora_power_dic.get(power,None)
        #if power_temp != None:

        if rssi:
            # enable print rssi value 
            rssi_temp = 0x80
        else:
            # disable print rssi value
            rssi_temp = 0x00        

        # get crypt
        l_crypt = crypt & 0xff
        h_crypt = crypt >> 8 & 0xff
        
        if relay==False:
            self.cfg_reg[3] = high_addr
            self.cfg_reg[4] = low_addr
            self.cfg_reg[5] = net_id_temp
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            # 
            # it will enable to read noise rssi value when add 0x20 as follow
            # 
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            #
            # it will output a packet rssi value following received message
            # when enable eighth bit with 06H register(rssi_temp = 0x80)
            #
            self.cfg_reg[9] = 0x43 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        else:
            self.cfg_reg[3] = 0x01
            self.cfg_reg[4] = 0x02
            self.cfg_reg[5] = 0x03
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            # 
            # it will enable to read noise rssi value when add 0x20 as follow
            # 
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            #
            # it will output a packet rssi value following received message
            # when enable eighth bit with 06H register(rssi_temp = 0x80)
            #
            self.cfg_reg[9] = 0x03 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        self.ser.flushInput()

        for i in range(2):
            self.ser.write(bytes(self.cfg_reg))
            r_buff = 0
            time.sleep(0.2)
            if self.ser.inWaiting() > 0:
                time.sleep(0.1)
                r_buff = self.ser.read(self.ser.inWaiting())
                if r_buff[0] == 0xC1:
                    pass
                    # print("parameters setting is :",end='')
                    # for i in self.cfg_reg:
                        # print(hex(i),end=' ')
                        
                    # print('\r\n')
                    # print("parameters return is  :",end='')
                    # for i in r_buff:
                        # print(hex(i),end=' ')
                    # print('\r\n')
                else:
                    pass
                    #print("parameters setting fail :",r_buff)
                break
            else:
                print("setting fail,setting again")
                sys.stderr.write("setting fail,setting again\n")
                self.ser.flushInput()
                time.sleep(0.2)
                print('\x1b[1A',end='\r')
                if i == 1:
                    print("setting fail,Press Esc to Exit and run again")
                    sys.stderr.write("setting fail,Press Esc to Exit and run again\n")
                    # time.sleep(2)
                    # print('\x1b[1A',end='\r')

        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.LOW)
        time.sleep(0.1)

    def get_settings(self):
        # the pin M1 of lora HAT must be high when enter setting mode and get parameters
        GPIO.output(self.M1,GPIO.HIGH)
        time.sleep(0.1)
        
        # send command to get setting parameters
        self.ser.write(bytes([0xC1,0x00,0x09]))
        if self.ser.inWaiting() > 0:
            time.sleep(0.1)
            self.get_reg = self.ser.read(self.ser.inWaiting())
        
        # check the return characters from hat and print the setting parameters
        if self.get_reg[0] == 0xC1 and self.get_reg[2] == 0x09:
            fre_temp = self.get_reg[8]
            addr_temp = self.get_reg[3] + self.get_reg[4]
            air_speed_temp = self.get_reg[6] & 0x03
            power_temp = self.get_reg[7] & 0x03
            
            print("Frequence is {0}.125MHz.",fre_temp)
            print("Node address is {0}.",addr_temp)
            print("Air speed is {0} bps"+ lora_air_speed_dic.get(None,air_speed_temp))
            print("Power is {0} dBm" + lora_power_dic.get(None,power_temp))
            GPIO.output(M1,GPIO.LOW)

    # This method return relevant ground station data
    def getGroundData(self): 

        print("getGroundData")
        sys.stderr.write("getGroundData")
        bearing_flag = 0
        ground_flag = 0

        while bearing_flag == 0 or ground_flag == 0:   
            msg = str(self.ser.readline())

            if msg.find("bearing") == 2:

                bearing = float(msg[10:-5])
                time.sleep(1)
                bearing_flag = 1

            if msg.find("GPGGA") == 3:

                ground_lat = msg[19:29]
                ground_lon = msg[33:43]
                ground_alt = msg[51:55]
                ground_flag = 1


                if(msg[30] == "S"):

                    ground_lat = -1*float(ground_lat)
                    ground_lat = float(ground_lat)/100
                
                elif(msg[30] == "N"):
                    
                    ground_lat = 1*float(ground_lat)
                    ground_lat = float(ground_lat)/100
                
                if(msg[44] == "W"):

                    ground_lon = -1*float(ground_lon)
                    ground_lon = float(ground_lon)/100

                elif(msg[44] == "E"):

                    ground_lon = 1*float(ground_lon)
                    ground_lon = float(ground_lon)/100

        return [ground_lat, ground_lon, ground_alt, bearing]

    # This method returns relevant HAB data
    def getHABData(self):

        print("getHABData")
        sys.stderr.write("getHABData")
        time_flag = 0
        alt_flag = 0
        lat_flag = 0
        lon_flag = 0

        while time_flag == 0 or alt_flag == 0 or lat_flag == 0 or lon_flag == 0:   

            msg = str(self.ser.readline())
            datafile = open('output.txt', 'a')
            now = datetime.now()
            date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
            log_msg = date_time + "," + msg + "\n"
            datafile.write(log_msg)
            datafile.close()

            if msg.find("time") == 10:

                HAB_time = msg[16:22]
                time_flag = 1

            if msg.find("alt") == 10:

                HAB_alt = msg[15:19]
                alt_flag = 1
            
            if msg.find("lat") == 10:

                HAB_lat = msg[15:23]
                size = len(HAB_lat)

                if(HAB_lat[size-1] == "N"):
                
                    HAB_lat = HAB_lat[:size-1]
                    HAB_lat = float(HAB_lat)/100
                    lat_flag = 1
                    
                else:
                
                    HAB_lat = HAB_lat[:size-1]
                    HAB_lat = -1*float(HAB_lat)/100
                    lat_flag = 1
                    
            if msg.find("lon") == 10:

                HAB_lon = msg[15:24]
                size = len(HAB_lon)
            
                if(HAB_lon[size-1] == "E"):
                
                    HAB_lon = HAB_lon[:size-1]
                    HAB_lon = float(HAB_lon)/100
                    lon_flag = 1
                    
                else:
                    HAB_lon = HAB_lon[:size-1]
                    HAB_lon = -1*float(HAB_lon)/100
                    lon_flag = 1
        
        return [HAB_lat, HAB_lon, HAB_alt, HAB_time]

    # This method drives the ground station
    def moveGroundStation(self, ground_lat, ground_lon, ground_alt, HAB_lat, HAB_lon, HAB_alt, bearing):
    
      [az,el] = azel.azimuth_elevation_angle(ground_lat, ground_lon, ground_alt, HAB_lat, HAB_lon, HAB_alt, bearing)

      az = int(az)
      el = int(el)
      ser1 = serial.Serial("/dev/ttyUSB1")
      
      # method that will convert variable into a 3 digit string
      if (az < 0):
       az_string = f'{az:04d}'
      else:
       az_string = f'{az:03d}'

      if (el < 0):
       el_string = f'{el:04d}'
      else:
       el_string = f'{el:03d}'

      ser1.write(bytearray(f"W{az_string} {el_string}\r", 'ascii'))
      #ser1.write(bytearray(f"W090 090\r", 'ascii'))
