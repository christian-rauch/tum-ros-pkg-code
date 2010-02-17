#! /usr/bin/python
import roslib; roslib.load_manifest('web_hri')

import SocketServer

from threading import Thread, Lock
import time, random, select
from ros_xmpp import XmppClient

from web_hri.srv import *

import rospy
from rospy.exceptions import ROSException
from rospy.client import *

from std_msgs.msg import String

stupidAnswers = ('I got it!', 'Oh, really?', 'That\'s interesting', 'Ok', 'Well, if you say so...', 'Alright', 'Are you sure? ...ok', 'Strange idea... but you\'re the boss', 'I guess so')
welcomeIncipits = ("Hello!", "I'm online now", "Servus", "Gruessgott Deutschland!", "Hi. Fancy a chat?", "Hello. Back online.", "Servus! The bavarian robot is up and fit!")
reqQueueLock = Lock()
reqQueue = [random.choice(welcomeIncipits)]

resQueueLock = Lock()
dontStealMyAnswerLock = Lock() #special lock to avoid that the ROS publisher steals answer from the ROS "ask" service
resQueue = []

class HRIChatHandler(SocketServer.BaseRequestHandler):

	def processWhatsNew(self):
		with reqQueueLock:
			try:
				return reqQueue.pop()
			except IndexError: #request queue empty
				return ""
		
	
	def processHumanInitiative(self):
			print "%s wrote:" % self.client_address[0]
			
			print self.data
			
			with resQueueLock:
				resQueue.append(self.data)
				
			return random.choice(stupidAnswers)
			
		
	def handle(self):
		# self.request is the TCP socket connected to the client
		try:
			[msg_length, self.data] = self.request.recv(1024).strip().split("\n", 1)
			
			if (self.data == "whatsnew"):
				res = self.processWhatsNew()
			else:
				res = self.processHumanInitiative()
			
			out = "len " + str(len(res)) + "\n" + res
			print "Sending new socket message to webbrowser: " + res
			self.request.send(out)
			
		except ValueError:
			pass





class XmppHandler(Thread):
	def __init__ (self, jidparams, tojid):
		Thread.__init__(self)
		self.client = XmppClient(jidparams,tojid, self.xmpp_msg_handler)
		self.remotejid = tojid
		self.online = True
		self.client.set_status('chat', "Ready to chat with an human!")
		

	def xmpp_msg_handler(self, con, event):
		type = event.getType()
		fromjid = event.getFrom().getStripped()
		if type in ['message', 'chat', None] and fromjid == self.remotejid:
			if event.getBody():
				if event.getBody().startswith("xmpp:"): #special commands
					cmd = event.getBody().replace("xmpp:","")
					if cmd.startswith("status:"):
						self.client.set_status(cmd.replace("status:",""))
					if cmd.startswith("help"):
						with reqQueueLock:
							reqQueue.append("Available commands:\n\t- xmpp:status: [status]")
				else:
					with resQueueLock:
						resQueue.append(encode8bits(event.getBody()))
					with reqQueueLock:
						reqQueue.append(random.choice(stupidAnswers))
		
	def stop(self):
		self.online = False

	def whatsNew(self):
		with reqQueueLock:
			try:
				return reqQueue.pop()
			except IndexError: #request queue empty
				return None
				
	def run(self):

		socketlist = {self.client.get_socket():'xmpp'}

		while self.online:
			(i , o, e) = select.select(socketlist.keys(),[],[],1) #useless... but it could be nice to use select if I apply it as well for ROS
			for each in i:
				if socketlist[each] == 'xmpp':
					self.client.process()
				else:
					raise Exception("Unknown socket type: %s" % repr(socketlist[each]))
			
			newMsg = self.whatsNew()
			if (newMsg):
				print "Sending new XMPP chat message:" + newMsg
				self.client.send_message(newMsg)
			
			time.sleep(0.1)

class ROSMonitor(Thread):
	def __init__ (self):
		Thread.__init__(self)
		rospy.init_node('web_hri')		
		self.rosAsk = rospy.Service("hri/ask_human", AskHuman, self.handleAskHuman)
		self.rosTell = rospy.Service("hri/tell_human", AskHuman, self.handleTellHuman)
		self.rosListen = rospy.Publisher("hri/listen_human", String)
		
	def run(self):
		while not rospy.is_shutdown():
			with dontStealMyAnswerLock:
				with resQueueLock:
					try:
						oldestInput = resQueue.pop()
						self.rosListen.publish(String(oldestInput))						
					except: #request queue empty
						pass
										
			time.sleep(0.2)
		
		
	def handleAskHuman(self, msg):
		with dontStealMyAnswerLock:
			with reqQueueLock:
				print str(len(reqQueue)) + " requests pending."
				print "Sending new question:" + msg.question
				reqQueue.append(msg.question)
				
			with resQueueLock:
				currentNbAnswer = len(resQueue)
		
			while (1):
				time.sleep(0.1)
				with resQueueLock:
						if (len(resQueue) > currentNbAnswer): #a new input from the human has been receive. Good!
							return AskHumanResponse(resQueue.pop())
						
	def handleTellHuman(self, msg):
		with reqQueueLock:
			print str(len(reqQueue)) + " requests pending."
			print "Sending new message:" + msg.question
			reqQueue.append(msg.question)
			
		return AskHumanResponse("ok")

def usage():
	return """Usage:
web_hri.py [jabber distant_jabber_account|socket hostname port]
	For example: web_hri.py jabber lenz@jabber.ccc.d
	or: web_hri.py socket localhost 9999"""


def encode8bits(o):
	if isinstance(o, list):
		return [encode8bits(o2) for o2 in o]
	return o.encode('ascii')
	
if __name__ == "__main__":

	type = ""
	
	if len(sys.argv) > 1:
		type = sys.argv[1]
		
		if type == "jabber":
			if len(sys.argv) >= 2:
				tojid=sys.argv[2]
			else:
				print "Missing distant Jabber account"
				sys.exit(1)
		elif type == "socket":
			if len(sys.argv) >= 3:
				HOST = sys.argv[2]
				PORT = int(sys.argv[3])
			else:
				print "Missing hostname and/or port"
				sys.exit(1)
		else:
			print "A mode (jabber or socket) was expected"
			sys.exit(1)
	else:
		print usage()
		sys.exit(1)
	
	rosMonitor = ROSMonitor()
	rosMonitor.start()
	
	
	## Code for the Web interface
	
	if type == "socket":
		# Create the server for the Web based interface
		server = SocketServer.TCPServer((HOST, PORT), HRIChatHandler)
		server.serve_forever()
		rospy.signal_shutdown()
	
	else:
		## Code for Jabber
		jidparams={"jid":"kimp@jabber.ccc.de", "password":"kimp"}
		xmpp = XmppHandler(jidparams, tojid)
		xmpp.start()
		rospy.spin()
		xmpp.stop()

