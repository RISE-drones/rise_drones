
import socket
import sys
import datetime
import base64
import time
import os
#import ssl
from optparse import OptionParser

#--------------------------------------------------------------------#

__author__ = 'Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>'
__version__ = '0.0.1'
__status__ = 'development'

#--------------------------------------------------------------------#
"""
This is heavily based on the NtripClient.py from https://github.com/jcmb/NTRIP,
which is distributed under the GNU General Public License
"""


VERSION=0.1
USERAGENT="NTRIP Client RISE v./%.1f" % VERSION

# reconnect parameter (fixed values):
SLEEP_FACTOR=2 # How much the sleep time increases with each failed attempt
class NtripClient(object):
  def __init__(self,
                buffer=50,
                user="",
                password="",
                out=sys.stdout,
                port=2101,
                caster="",
                mountpoint="",
                description="",
                lat=46,
                lon=122,
                height=1212,
                use_ssl=False,
                verbose=False,
                udp_ip=None,
                udp_port=None,
                version_2=False,
                header_file=sys.stderr,
                header_output=False,
                max_reconnect=1,
                max_connect_time=0
                ):
    self.buffer=buffer
    auth_string = user + ":" + password
    self.user=base64.b64encode(bytes(auth_string,'utf-8')).decode("utf-8")
    self.out=out
    self.port=port
    self.caster=caster
    self.mountpoint=mountpoint
    self.setPosition(lat, lon)
    self.height=height
    self.verbose=verbose
    self.use_ssl=use_ssl
    self.udp_ip=udp_ip
    self.udp_port=udp_port
    self.version_2=version_2
    self.header_file=header_file
    self.header_output=header_output
    self.max_reconnect=max_reconnect
    self.max_connect_time=max_connect_time
    self.description=description

    self.socket=None

    if udp_port and udp_ip:
      self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.UDP_socket.connect((udp_ip, udp_port))
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
    mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (self.mountpoint, USERAGENT, self.user)
#        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\n" % (self.mountpoint, USERAGENT)
    if self.version_2:
      mountPointString+="Ntrip-Version: Ntrip/2.0\r\n"
    mountPointString+="\r\n"
    if self.verbose:
      print (mountPointString)
    return bytes(mountPointString,'ascii')

  def getGGABytes(self):
    now = datetime.datetime.utcnow()
    ggaString= "GPGGA,%02d%02d%04.2f,%02d%011.8f,%1s,%03d%011.8f,%1s,1,05,0.19,+00400,M,%5.3f,M,," % \
      (now.hour,now.minute,now.second,self.latDeg,self.latMin,self.flagN,self.lonDeg,self.lonMin,self.flagE,self.height)
    checksum = self.calcultateCheckSum(ggaString)
    if self.verbose:
      print  ("$%s*%s\r\n" % (ggaString, checksum))
      print  (self.description)
    return bytes("$%s*%s\r\n" % (ggaString, checksum),'ascii')

  def calcultateCheckSum(self, stringToCheck):
    xsum_calc = 0
    for char in stringToCheck:
      xsum_calc = xsum_calc ^ ord(char)
    return "%02X" % xsum_calc

  def readData(self):
    reconnectTry=1
    sleepTime=1
    if self.max_connect_time > 0 :
      end_connect=datetime.timedelta(seconds=self.max_connect_time)
    try:
      while reconnectTry<=self.max_reconnect:
        found_header=False
        if self.verbose:
          sys.stderr.write('Connection {0} of {1}\n'.format(reconnectTry,self.max_reconnect))

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.use_ssl:
          print("Support for SSL not implemented yet")
#         self.socket=ssl.wrap_socket(self.socket)

        error_indicator = self.socket.connect_ex((self.caster, self.port))
        if error_indicator==0:
          sleepTime = 1
          connect_time=datetime.datetime.now()

          self.socket.settimeout(10)
          self.socket.sendall(self.getMountPointBytes())
          while not found_header:
            casterResponse=self.socket.recv(4096) #All the data
#                        print(casterResponse)
            header_lines = casterResponse.decode('utf-8').split("\r\n")

            for line in header_lines:
              if line=="":
                if not found_header:
                  found_header=True
                  if self.verbose:
                    sys.stderr.write("End Of Header"+"\n")
              else:
                if self.verbose:
                  sys.stderr.write("Header: " + line+"\n")
              if self.header_output:
                self.header_file.write(line+"\n")

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
#AG                            self.out.write(data)
#                            self.out.buffer.write(data)
              if self.UDP_socket:
                self.UDP_socket.send(data)
#                            print (datetime.datetime.now()-connect_time)
#                            print(self.max_connect_time)
              if self.max_connect_time :
                if datetime.datetime.now() > connect_time+end_connect:
                  if self.verbose:
                    sys.stderr.write("Connection Time exceeded\n")
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

          if reconnectTry < self.max_reconnect :
            sys.stderr.write( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
            time.sleep(sleepTime)
            sleepTime *= SLEEP_FACTOR

            if sleepTime>self.max_connect_time:
              sys.stderr.write('Maximum number of reconnects..\n')
              sleepTime=self.max_connect_time
          else:
            sys.exit(1)


          reconnectTry += 1
        else:
          self.socket=None
          if self.verbose:
            print ("Error indicator: ", error_indicator)

          if reconnectTry < self.max_reconnect :
            sys.stderr.write( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
            time.sleep(sleepTime)
            sleepTime *= SLEEP_FACTOR
            if sleepTime>self.max_connect_time:
              sleepTime=self.max_connect_time
          reconnectTry += 1

    except KeyboardInterrupt:
      if self.socket:
        self.socket.close()
      sys.exit()

if __name__ == '__main__':
  usage="NtripClient.py [options] caster port mountpoint"
  parser=OptionParser(version=str(VERSION), usage=usage)
  parser.add_option("-u", "--user", type=str, dest="user", default="user", help="The Ntrip caster username.")
  parser.add_option("-p", "--password", type=str, dest="password", default="pwd", help="The Ntrip caster password.")
  parser.add_option("-s", "--server", type=str, dest="server", default="server", help="The server to connect to")
  parser.add_option("-o", "--port", type=int, dest="port", default=2101, help="The ntrip server port")
  parser.add_option("-m", "--mountpoint", type=str, dest="mountpoint", default="mountpoint", help="The mountpoint to connect to")
  parser.add_option("-d", "--description", type=str, dest="description", default="Add mountpoint description", help="Describes the selected mountpoint")
  parser.add_option("-t", "--latitude", type=float, dest="lat", default=58.40062, help="Your latitude.")
  parser.add_option("-g", "--longitude", type=float, dest="lon", default=15.57558, help="Your longitude.")
  parser.add_option("-e", "--height", type=float, dest="height", default=40, help="Your ellipsoid height.")
  parser.add_option("-v", "--verbose", action="store_true", dest="verbose", default=False, help="Verbose")
  parser.add_option("-r", "--Reconnect", type=int, dest="max_reconnect", default=1, help="Number of reconnections")
  parser.add_option("-I", "--udp_ip", type=str, dest="udp_ip", default="127.0.0.1", help="Broadcast received data on the provided IP")
  parser.add_option("-P", "--udp_port", type=int, dest="udp_port", default=13320, help="Broadcast received data on the provided port")
  parser.add_option("-2", "--version_2", action="store_true", dest="version_2", default=True, help="Make a NTRIP V2 Connection")
  parser.add_option("-f", "--outputFile", type=str, dest="outputFile", default=None, help="Write to this file, instead of stdout")
  parser.add_option("-x", "--maxtime", type=int, dest="max_connect_time", default=0, help="Maximum length of the connection, in seconds. 0 = inf connection")
  parser.add_option("--Header", action="store_true", dest="header_output", default=False, help="Write headers to stderr")
  parser.add_option("--headerfile", type=str, dest="header_file", default=None, help="Write headers to this file, instead of stderr.")
  (options, args) = parser.parse_args()
  ntripArgs = {}
  ntripArgs['lat']=options.lat
  ntripArgs['lon']=options.lon
  ntripArgs['height']=options.height
  ntripArgs['caster'] = options.server
  ntripArgs['port'] = options.port
  ntripArgs['user'] = options.user
  ntripArgs['password'] =options.password
  ntripArgs['mountpoint'] = options.mountpoint
  ntripArgs['description'] = options.description
  ntripArgs['version_2']=options.version_2

  ntripArgs['verbose']=options.verbose
  ntripArgs['header_output']=options.header_output
  ntripArgs['max_reconnect'] = options.max_reconnect
  ntripArgs['max_connect_time']=options.max_connect_time

  if ntripArgs['mountpoint'][0:1] !="/":
    ntripArgs['mountpoint'] = "/"+ntripArgs['mountpoint']

  if options.udp_port and options.udp_ip:
    ntripArgs['udp_port']=int(options.udp_port)
    ntripArgs['udp_ip'] = options.udp_ip




  if options.verbose:
    print ("Server: " + ntripArgs['caster'])
    print ("Port: " + str(ntripArgs['port']))
    print ("User: " + options.user)
    print ("mountpoint: " +ntripArgs['mountpoint'])
    print ("Reconnects: " + str(ntripArgs['max_reconnect']))
    print ("Max Connect Time: " + str (ntripArgs['max_connect_time']))
    if ntripArgs['version_2']:
      print ("NTRIP: V2")
    else:
      print ("NTRIP: V1")
    if 'udp_port' in ntripArgs and 'udp_ip' in ntripArgs:
      print("Broadcast UDP data on: ", ntripArgs['udp_ip'], ":", ntripArgs['udp_port'])
    print ("")



  fileOutput=False

  if options.outputFile:
    f = open(options.outputFile, 'wb')
    ntripArgs['out']=f
    fileOutput=True
  else:
    stdout= os.fdopen(sys.stdout.fileno(), "wb", closefd=False,buffering=0)
    ntripArgs['out']=stdout

  if options.header_file:
    h = open(options.header_file, 'w')
    ntripArgs['header_file']=h
    ntripArgs['header_output']=True

  n = NtripClient(**ntripArgs)
  try:
    n.readData()
  finally:
    if fileOutput:
      f.close()
    if options.header_file:
      h.close()
