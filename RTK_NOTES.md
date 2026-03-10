Different versions of LC29H

When ordering the module you need to be careful as there are 3 different versions doing completely different things. There is no way to programmatically change the version of the module and all of them are named LC29H:

LC29H(AA) – Supports positioning augmentation system and EASY technology. This one should give you about 1m of accuracy It DOESN’T WORK WITH RTK.
LC29H(DA) – Supports RTK Rover to realize high-precision centimeter-level positioning. This is the module you can install on your rover, robot, car whatever you want to localize.
LC29H(BS) – Supports RTK Base, and can establish a base station to transmit correction data. It is a stationary module that gives you correction data that can be sent to the rover (DA) module.

I didn’t use the AA version, so I can’t say much about it. Remember that you need some kind of base station that can output RTK correction data for the DA to work. Either you have to set up your own base station with a BS module or use services like RTK2GO or an open national network if you happen to have one.

How to use LC29H(DA)

This part of the tutorial is quite well described and provides enough information on how to run the example Python code. I will try to briefly explain it to you here as well.

Start by enabling serial, I2C, and SPI on your Raspberry Pi, it’s easy to do if you are using a desktop interface:

I have been using my Raspberry Pi in headless mode over SSH so to do the same thing I used the command:

sudo raspi-config

After running the command such a screen will appear, navigate (using arrow keys) to interface options and enable serial, I2C, and SPI. It’s straightforward for I2C and SPI but for serial, you need to enable only the serial port hardware we don’t need the login shell to be accessible over serial.

After that, you will need to reboot the Pi.

Run the following commands to update the software and install the necessary libraries:

sudo apt-get update
sudo apt-get install gpsd gpsd-clients 
sudo pip3 install gps3

Modify the gpsd file with nano:

#Open gpsd text
sudo nano /etc/default/gpsd
#Modify the following parameters of the document and save to exit
USBAUTO="false"
DEVICES="/dev/ttyS0"
GPSD_OPTIONS="/dev/ttyUSB0"

If you want you can use the example code provided by WaveShare but I will be using my own, slightly customized version of the code. It prints all the information to the terminal together with the status (GPS, GPS fix, RTK float, RTK fix).

#!/usr/bin/env -S python3 -u
"""
This is heavily based on the NtripPerlClient program written by BKG.
Then heavily based on a unavco original.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

"""

import socket
import sys
import datetime
import base64
import time
#import ssl
from optparse import OptionParser

import serial
from pynmeagps import NMEAReader
import requests

version=0.2
useragent="NTRIP JCMBsoftPythonClient/%.1f" % version

# reconnect parameter (fixed values):
factor=3 # How much the sleep time increases with each failed attempt
maxReconnect=1
maxReconnectTime=1200
sleepTime=3 # So the first one is 1 second

class NtripClient(object):
    def __init__(self,
                 buffer=50,
                 user="",
                 out=sys.stdout,
                 port=2101,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=50,
                 lon=19,
                 height=300,
                 ssl=False,
                 verbose=False,
                 UDP_Port=None,
                 V2=False,
                 headerFile=sys.stderr,
                 headerOutput=False,
                 maxConnectTime=0,
                 gps_file_path = None
                 ):
        self.buffer=buffer
        self.user=base64.b64encode(bytes(user,'utf-8')).decode("utf-8")
#        print(self.user)
        self.out=out
        self.port=port
        self.caster=caster
        self.mountpoint=mountpoint
        self.setPosition(lat, lon)
        self.height=height
        self.verbose=verbose
        self.ssl=ssl
        self.host=host
        self.UDP_Port=UDP_Port
        self.V2=V2
        self.headerFile=headerFile
        self.headerOutput=headerOutput
        self.maxConnectTime=maxConnectTime
        self.socket=None
        self.stream = serial.Serial('/dev/ttyS0', 115200, timeout=3)
        self.nmr = NMEAReader(self.stream)

        self.gps_file_path = gps_file_path
        if self.gps_file_path:
            self.gps_file = open(self.gps_file_path, 'ab')
        else:
            self.gps_file = None
        

        if UDP_Port:
            self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_socket.bind(('', 0))
            self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        else:
            self.UDP_socket=None


    def setPosition(self, lat, lon):
        self.flagN="N"
        self.flagE="E"
        if lon>180:
            lon=(lon-360)*-1
            self.flagE="W"
        elif (lon<0 and lon>= -180):
            lon=lon*-1
            self.flagE="W"
        elif lon<-180:
            lon=lon+360
            self.flagE="E"
        else:
            self.lon=lon
        if lat<0:
            lat=lat*-1
            self.flagN="S"
        self.lonDeg=int(lon)
        self.latDeg=int(lat)
        self.lonMin=(lon-self.lonDeg)*60
        self.latMin=(lat-self.latDeg)*60

    def getMountPointBytes(self):
        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (self.mountpoint, useragent, self.user)
#        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\n" % (self.mountpoint, useragent)
        if self.host or self.V2:
           hostString = "Host: %s:%i\r\n" % (self.caster,self.port)
           mountPointString+=hostString
        if self.V2:
           mountPointString+="Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString+="\r\n"
        if self.verbose:
           print (mountPointString)
        return bytes(mountPointString,'ascii')

    def getGGABytes(self):
        while True:
            (raw_data, parsed_data) = self.nmr.read()
            if bytes("GNGGA",'ascii') in raw_data :
                # print(parsed_data)
                return raw_data
            
    def __del__(self):
        if self.gps_file:
            self.gps_file.close()

    def parse_gngga_sentence(self, nmea_sentence):
        parts = nmea_sentence.split(',')
        if len(parts) < 10:
            return None  # Invalid sentence

        # Latitude
        try:
            lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
            if parts[3] == 'S':
                lat = -lat
            # Longitude
            lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
            if parts[5] == 'W':
                lon = -lon
            # Altitude
            alt = float(parts[9]) if parts[9] else 0.0
            # fix quality
            fix_quality = int(parts[6]) if parts[6].isdigit() else 0
        except ValueError:
            return None  # Handle parsing errors
        return lat, lon, alt, fix_quality


    def readData(self):
        reconnectTry=1
        sleepTime=1
        reconnectTime=0
        if self.maxConnectTime > 0 :
            EndConnect=datetime.timedelta(seconds=maxConnectTime)
        try:
            while reconnectTry<=maxReconnect:
                time.sleep(1)
                found_header=False
                if self.verbose:
                    sys.stderr.write('Connection {0} of {1}\n'.format(reconnectTry,maxReconnect))

                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if self.ssl:
                    self.socket=ssl.wrap_socket(self.socket)

                error_indicator = self.socket.connect_ex((self.caster, self.port))
                if error_indicator==0:
                    sleepTime = 1
                    connectTime=datetime.datetime.now()

                    self.socket.settimeout(10)
                    self.socket.sendall(self.getMountPointBytes())
                    while not found_header:
                        casterResponse=self.socket.recv(4096) #All the data
                        # print(casterResponse)
                        header_lines = casterResponse.decode('utf-8').split("\r\n")
                        
# header_lines empty, request fail,exit while loop
                        for line in header_lines:
                            if line=="":
                                if not found_header:
                                    found_header=True
                                    if self.verbose:
                                        sys.stderr.write("End Of Header"+"\n")
                            else:
                                if self.verbose:
                                    sys.stderr.write("Header: " + line+"\n")
                            if self.headerOutput:
                                self.headerFile.write(line+"\n")



#header_lines has content
                        for line in header_lines:
                            if line.find("SOURCETABLE")>=0:
                                sys.stderr.write("Mount point does not exist")
                                sys.exit(1)
                            elif line.find("401 Unauthorized")>=0:
                                sys.stderr.write("Unauthorized request\n")
                                sys.exit(1)
                            elif line.find("404 Not Found")>=0:
                                sys.stderr.write("Mount Point does not exist\n")
                                sys.exit(2)
                            elif line.find("ICY 200 OK")>=0:
                                #Request was valid
                                self.socket.sendall(self.getGGABytes())
                            elif line.find("HTTP/1.0 200 OK")>=0:
                                #Request was valid
                                self.socket.sendall(self.getGGABytes())
                            elif line.find("HTTP/1.1 200 OK")>=0:
                                #Request was valid
                                self.socket.sendall(self.getGGABytes())



                    data = "Initial data"
                    while data:
                        try:
                            data=self.socket.recv(self.buffer)
                            # self.out.buffer.write(data)
                            self.stream.write(data)
                            (raw_data, parsed_data) = self.nmr.read()
                            if bytes("GNGGA",'ascii') in raw_data :
                                print(raw_data)
                                location = self.parse_gngga_sentence(raw_data.decode('ascii'))
                                if location:
                                    lat, lon, _, fix_quality = location
                                    fix_status = {
                                            0: "Invalid",
                                            1: "GPS fix",
                                            2: "DGPS fix",
                                            3: "PPS fix",
                                            4: "RTK fixed",
                                            5: "RTK float",
                                            6: "Estimated",
                                            7: "Manual input",
                                            8: "Simulation"
                                        }.get(fix_quality, "Unknown")
                                    print(f"Parsed Location: Latitude={lat}, Longitude={lon}, Fix Status={fix_status}")
                                if self.gps_file:
                                    self.gps_file.write(raw_data)
                                    self.gps_file.flush()

                            if self.UDP_socket:
                                self.UDP_socket.sendto(data, ('<broadcast>', self.UDP_Port))
#                            print datetime.datetime.now()-connectTime
                            if maxConnectTime :
                                if datetime.datetime.now() > connectTime+EndConnect:
                                    if self.verbose:
                                        sys.stderr.write("Connection Timed exceeded\n")
                                    sys.exit(0)
#                            self.socket.sendall(self.getGGAString())



                        except socket.timeout:
                            if self.verbose:
                                sys.stderr.write('Connection TimedOut\n')
                            data=False
                        except socket.error:
                            if self.verbose:
                                sys.stderr.write('Connection Error\n')
                            data=False

                    if self.verbose:
                        sys.stderr.write('Closing Connection\n')
                    self.socket.close()
                    self.socket=None

                    if reconnectTry < maxReconnect :
                        sys.stderr.write( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime = factor

                        if sleepTime>maxReconnectTime:
                            sleepTime=maxReconnectTime
                    else:
                        sys.exit(1)


                    reconnectTry += 1
                else:
                    self.socket=None
                    if self.verbose:
                        print ("Error indicator: ", error_indicator)

                    if reconnectTry < maxReconnect :
                        sys.stderr.write( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime = factor
                        if sleepTime>maxReconnectTime:
                            sleepTime=maxReconnectTime
                    reconnectTry += 1

        except KeyboardInterrupt:
            if self.socket:
                self.socket.close()
            self.stream.close()
            sys.exit()

    
        

if __name__ == '__main__':
    usage="NtripClient.py [options] [caster] [port] mountpoint"
    parser=OptionParser(version=version, usage=usage)
    parser.add_option("-u", "--user", type="string", dest="user", default="IBS", help="The Ntripcaster username.  Default: %default")
    parser.add_option("-p", "--password", type="string", dest="password", default="IBS", help="The Ntripcaster password. Default: %default")
    parser.add_option("-o", "--org", type="string", dest="org", help="Use IBSS and the provided organization for the user. Caster and Port are not needed in this case Default: %default")
    parser.add_option("-b", "--baseorg", type="string", dest="baseorg", help="The org that the base is in. IBSS Only, assumed to be the user org")
    parser.add_option("-t", "--latitude", type="float", dest="lat", default=50.09, help="Your latitude.  Default: %default")
    parser.add_option("-g", "--longitude", type="float", dest="lon", default=8.66, help="Your longitude.  Default: %default")
    parser.add_option("-e", "--height", type="float", dest="height", default=1200, help="Your ellipsoid height.  Default: %default")
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose", default=True, help="Verbose")
    parser.add_option("-s", "--ssl", action="store_true", dest="ssl", default=False, help="Use SSL for the connection")
    parser.add_option("-H", "--host", action="store_true", dest="host", default=False, help="Include host header, should be on for IBSS")
    parser.add_option("-r", "--Reconnect", type="int", dest="maxReconnect", default=1, help="Number of reconnections")
    parser.add_option("-D", "--UDP", type="int", dest="UDP", default=None, help="Broadcast recieved data on the provided port")
    parser.add_option("-2", "--V2", action="store_true", dest="V2", default=False, help="Make a NTRIP V2 Connection")
    parser.add_option("-f", "--outputFile", type="string", dest="outputFile", default=None, help="Write to this file, instead of stdout")
    parser.add_option("-m", "--maxtime", type="int", dest="maxConnectTime", default=None, help="Maximum length of the connection, in seconds")

    parser.add_option("--Header", action="store_true", dest="headerOutput", default=False, help="Write headers to stderr")
    parser.add_option("--HeaderFile", type="string", dest="headerFile", default=None, help="Write headers to this file, instead of stderr.")
    (options, args) = parser.parse_args()
    # print(args)
    # print(len(args))
    ntripArgs = {}
    ntripArgs['lat']=options.lat
    ntripArgs['lon']=options.lon
    ntripArgs['height']=options.height
    ntripArgs['host']=options.host
    if options.outputFile:
        ntripArgs['gps_file_path'] = options.outputFile

    if options.ssl:
        import ssl
        ntripArgs['ssl']=True
    else:
        ntripArgs['ssl']=False


#ignore
    if options.org:
        if len(args) != 1 :
            print ("Incorrect number of arguments for IBSS. You do not need to provide the server and port\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user']=options.user+"."+options.org + ":" + options.password
        if options.baseorg:
            ntripArgs['caster']=options.baseorg + ".ibss.trimbleos.com"
        else:
            ntripArgs['caster']=options.org + ".ibss.trimbleos.com"
        if options.ssl :
            ntripArgs['port']=52101
        else :
            ntripArgs['port']=2101
        ntripArgs['mountpoint']=args[0]

    else:
        if len(args) != 3 :
            print ("Incorrect number of arguments for NTRIP\n")
            parser.print_help()
            sys.exit(1)
# execuet
        ntripArgs['user']=options.user+":"+options.password
        ntripArgs['caster']=args[0]
        ntripArgs['port']=int(args[1])
        ntripArgs['mountpoint']=args[2]

    if ntripArgs['mountpoint'][0:1] !="/":
        ntripArgs['mountpoint'] = "/"+ntripArgs['mountpoint']

    ntripArgs['V2']=options.V2

    ntripArgs['verbose']=options.verbose
    ntripArgs['headerOutput']=options.headerOutput

    if options.UDP:
         ntripArgs['UDP_Port']=int(options.UDP)

    maxReconnect=options.maxReconnect
    maxConnectTime=options.maxConnectTime

    if options.verbose:
        print ("Server: " + ntripArgs['caster'])
        print ("Port: " + str(ntripArgs['port']))
        print ("User: " + ntripArgs['user'])
        print ("mountpoint: " +ntripArgs['mountpoint'])
        print ("Reconnects: " + str(maxReconnect))
        print ("Max Connect Time: " + str (maxConnectTime))
        if ntripArgs['V2']:
            print ("NTRIP: V2")
        else:
            print ("NTRIP: V1")
        if ntripArgs["ssl"]:
            print ("SSL Connection")
        else:
            print ("Uncrypted Connection")
        print ("")



    fileOutput=False

    if options.outputFile:
        f = open(options.outputFile, 'wb')
        ntripArgs['out']=f
        fileOutput=True

    if options.headerFile:
        h = open(options.headerFile, 'w')
        ntripArgs['headerFile']=h
        ntripArgs['headerOutput']=True

    n = NtripClient(**ntripArgs)
    try:
        n.readData()
    finally:
        if fileOutput:
            f.close()
        if options.headerFile:
            h.close()

If you need more explanation about the code or want to modify something ask ChatGPT or any other LLM.

To run the script and connect to the NTRIP caster you need to add parameters when running the script. In my case to connect to ASG-EUPOS I run:

python3 main.py -u USERNAME -p PASSWORD -f gpsdata.txt system.asgeupos.pl 8086 RTK4G_MULTI_RTCM32

You should replace the username and password with your credentials. If you are using a different service for NTRIP correction data check their documentation and choose the proper address port and format. The gpsdata.txt file is used to save the data so that you can see it later on a computer.

How to set up the base station with LC29H(BS) module

I don’t think that such free NTRIp correction data services are popular around the world. In some cases, such services might not be available or might require you to pay a monthly fee. In such cases, it might be a good idea to setup the reference base station on your own. Since the price of the BS module is the same as the DA one it’s not going to be expensive. If you grab a used Pi 3 B+ you should be able to build the base station under $100. Setting it up is a bit more work than with the rover module.

So for the basestation to work you will most likely need to put it outside. For that, you will need some kind of waterproof case that will house the Raspberry Pi power adapter, Raspberry Pi, and the GPS module. The antenna and power cable need to go out of the case. You can also use PoE (Power over Ethernet) to power the base station while having it connected to the internet via ethernet (I decided to go with WIFI). At the top of the case, I attached a 3D-printed antenna holder using hot glue.

According to the tutorial from WaveShare you need to use QGNSS software to configure the module and do the survey-in so that it learns the position of the antenna. Since I do not have a Windows-based laptop it was quite hard for me to use. Also despite looking complicated, the software doesn’t help at all with configuring the module and you need to send the commands manually. In the end, I created a simple Python script that connects through serial and let me send the commands and see the responses from the module. It automatically calculates the checksum for you.

import serial
import threading

def calculate_checksum(data: str) -> str:
    """Calculate XOR checksum for given data (excluding $ and *)."""
    checksum = 0
    for char in data:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def read_from_gps(ser):
    """Reads data from the GPS module and prints messages that start with $."""
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$'):
                print(f"Received: {line}")
        except Exception as e:
            print(f"Error reading from GPS: {e}")

def main():
    baud_rate = 115200
    
    try:
        ser = serial.Serial("/dev/ttyS0", baud_rate, timeout=1)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return
    
    threading.Thread(target=read_from_gps, args=(ser,), daemon=True).start()
    
    print("Type GPS commands to send. Press Ctrl+C to exit.")
    try:
        while True:
            user_input = input("Send: ").strip()
            if user_input:
                if not user_input.startswith('$'):
                    print("Error: Command must start with '$'")
                    continue
                
                command = user_input[1:]  # Remove leading $
                checksum = calculate_checksum(command)
                full_command = f"${command}*{checksum}\r\n"
                
                ser.write(full_command.encode('utf-8'))
                print(f"Sent: {full_command.strip()}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

Here is what commands to send step by step:

Reset Module:
Send $PQTMRESTOREPAR to restore default parameters.

Start Position Training:
Send $PQTMCFGSVIN,W,1,3600,15,0,0,0 to begin a 1-hour training session with specific settings. In the documentation, the unit of field that’s here 3600 is specified as seconds. It’s not true, that’s a number of samples that meet the accuracy limit. If your accuracy limit is set too low it might take longer than the set time or even never complete if your GPS signal is weak. In the command above accuracy limit is set to 15m.

Retrieve Result:
Send $PQTMCFGSVIN,R*26 to request the outcome. The expected response is $PQTMCFGSVIN,OK,1,3600,1,X,Y,Z
X, Y, Z values are useful in the next step as well as later when setting up the RTKBase software so save them.

Save Adjusted Result:
Send $PQTMCFGSVIN,W,2,0,0,X,Y,Z to update the configuration with new X, Y, Z values obtained with a previous command.

Commit Changes:
Send $PQTMSAVEPAR to save all modifications. Unplug and plug back in the module.

Now we need to install software that will run as an NTRIP server and will send the RTK correction data to the module on the rover. For that,t I am using RTKBase:

https://github.com/Stefal/rtkbase

To install it on your Raspberry Pi just follow these commands:

git clone https://github.com/Stefal/rtkbase.git
cd rtkbase/tools
sudo ./install.sh --all release

Once you have it installed you should be able to open the RTKBase software by typing the IP address of your Raspberry Pi in the browser. The default password to rtkbase is admin. The configuration of the main service for this module looks as follows. For base coordinates you need to convert from ECEF to LLA coordinates, you can do that with QGNSS (screen below) or with some online tools. A small tip here, the altitude needs to have at least two decimal places otherwise you won’t be able to save the changes. Once your configuration is ready click on to turn on the main service, it should turn green.

 

After that, we can configure the NTRIP caster as follows. Choose whatever you want for the username, password, and mount point. These are the credentials you will use to connect from the DA module to the base station as shown previously.

If everything works you should see the signals from the satellites on the status tab (the first one). To connect to my own base station I am running the Python script like this:

python3 main.py -u niko -p PASSWORD -f gpsdata.txt rtkbase.local 2101 rtkbase

I hope this will help you with setting up your own base station and using this inexpensive GPS RTK system in your projects. If you enjoy consider subscribing to my YouTube channel and taking a look at my online store.

Happy making!