#!/usr/bin/env python

# jamorham

# quick and dirty python script to receive incoming data from the Parakeet
# and emulate some of the function of 'dexterity'

# This allows you to use the Parakeet like a 'wifi wixel' in the xDrip android application

# intended for raspberry pi or other linux box

# uses webpy module - apt-get install python-webpy or pip install webpy or from webpy.org

import web
import json

import time
import subprocess
#import xml.etree.ElementTree as ET
import threading

import os
import signal
import logging
import datetime as dt
import json
import socket
import sys
from thread import *


HOST = ''   # Symbolic name meaning all available interfaces
PORT = 50018 # Arbitrary non-privileged port for json requests
GSMPORT = 10080 # Incoming cgi request port from Parakeet

if (os.getuid() == 0):
	print "Dropping root"
	os.setgid(1000)
	os.setuid(1000)
	print "Dropped to user: ",os.getuid()
	if (os.getuid() == 0):
		print "Cannot drop root - exit!"
		sys.exit()


web.config.debug = False


urls = ('/request.cgi', 'entries',)

# Output Template
mydata = { "TransmitterId":"0","_id":1,"CaptureDateTime":0,"RelativeTime":0,"ReceivedSignalStrength":0,
		  "RawValue":0,"TransmissionId":0,"BatteryLife":0,"UploadAttempts":0,"Uploaded":0,
		  "UploaderBatteryLife":0,"FilteredValue":0 }


# Handle requests from android app asking for json data
def clientthread(conn):
 

	while True:
		data = conn.recv(1024)
		reply = ''
		if not data:
			break
		decoded = json.loads(data)
		print json.dumps(decoded,sort_keys=True,indent=4)
		print "Records wanted: ",decoded['numberOfRecords']

		mydata['RelativeTime'] = str((int(time.time()) * 1000) - int(mydata['CaptureDateTime']))

		if (mydata['RawValue'] != 0):
				reply = reply + json.dumps(mydata) + "\n\n"
		else:
				print "we do not have any data to send yet"

		print reply
		conn.sendall(reply)

	conn.close()

# thread end
class MyApplication(web.application):
	def run(self, port=GSMPORT, *middleware):
		func = self.wsgifunc(*middleware)
		return web.httpserver.runsimple(func, ('0.0.0.0', port))

class entries:
	def GET(self):
		global mydata
		global logging
		print "get"
		# lv=raw lf=filtered ts=timesince in ms
		data = web.input(lv=0,lf=0,ts=0)
		print "LV=",data.lv
		print "LF=",data.lf
		print "TS=",data.ts

		if (data.lv != "") and (int(data.lv) > 0) and (int(data.lf) > 0) and (int(data.ts) > 0):
			reply = "!ACK"

			mydata['CaptureDateTime'] = str(int(time.time()) - (int(data.ts) / 1000)) + "000"
			mydata['RelativeTime'] = "0"
			mydata['RawValue'] = data.lv
			mydata['FilteredValue'] = data.lf

			logging.info("Received GSM update: Capture Time: {0} - Raw: {1} - Filtered: {2} - uplink latency: {3}".format(mydata['CaptureDateTime'],mydata['RawValue'],mydata['FilteredValue'],data.ts))


		else:
			reply = "ERR"
		print reply
		return reply


def simtime(record,datefmt=None):
		return strftime('X %m-%d %H:%M')


def myFormatTime(self, record, datefmt=None):
   
		ct = self.converter(record.created)
		if datefmt:
			s = time.strftime(datefmt, ct)
		else:
			t = time.strftime("%Y-%m-%d %H:%M:%S", ct)
			s = "%s,%03d" % (t, record.msecs)
		return str(int(ct)) + " " + s

class MyFormatter(logging.Formatter):
	converter = dt.datetime.fromtimestamp
	def formatTime(self, record, datefmt=None):
		ct = self.converter(record.created)
		if datefmt:
			s = ct.strftime(datefmt)
			s = str(int(record.created)) + " " + s
		else:
			t = ct.strftime("%Y-%m-%d %H:%M:%S")
			s = "%s" % t    
			s = str(int(record.created)) + " " + s
		return s

if __name__ == "__main__":

	logging.basicConfig(level=logging.DEBUG,
						format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
						datefmt='%m-%d %H:%M',
						filemode='w')

	fh = logging.FileHandler('/tmp/gsm-receiver.log')
	
	console = logging.StreamHandler()
	console.setLevel(logging.INFO)
	formatter = MyFormatter('%(asctime)s %(name)-12s: %(levelname)-8s %(message)s')
	fh.setFormatter(formatter)
	console.setFormatter(formatter)
	
	# add the handler to the root logger
	logging.getLogger().addHandler(console)
	logging.getLogger().addHandler(fh)



	try: 
		logging.info("GSM receiver Main start")

		app = MyApplication(urls,globals())
		apt = threading.Thread(target=app.run,args=())
		apt.start()


		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		print 'Socket created'

		# Bind socket to local host and port
		try:
				s.bind((HOST, PORT))
		except socket.error as msg:
				print 'Json PORT Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
				sys.exit()

		print 'Socket bind complete'

		# Start listening on socket
		s.listen(10)
		print 'Socket now listening'


	# socket thread

		while 1:
				# Wait to accept a connection - blocking call
				conn, addr = s.accept()
				print 'Connected with ' + addr[0] + ':' + str(addr[1])
				start_new_thread(clientthread ,(conn,))

		s.close()



	except KeyboardInterrupt:
		print "shutting down"
		os.kill(os.getpid(), signal.SIGKILL)
