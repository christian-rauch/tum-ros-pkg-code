#!/usr/bin/python
# Based on examples from http://xmpppy.sourceforge.net/
import sys,os,xmpp,time,select

class XmppClient:

	def __init__(self,jidparams,remotejid, message_handler = None):
		self.password = jidparams['password']
		self.jid=xmpp.protocol.JID(jidparams['jid'])
		self.client=xmpp.Client(self.jid.getDomain(),debug=[])
		self.remotejid = remotejid
		
		if not message_handler:
			self.message_handler = self.default_message_handler
		else:
			self.message_handler = message_handler
		
		if not self.xmpp_connect():
			sys.stderr.write("Could not connect to server, or password mismatch!\n")
			sys.exit(1)

	def register_handlers(self):
		self.client.RegisterHandler('message',self.message_handler)

	def default_message_handler(self, con, event):
		type = event.getType()
		fromjid = event.getFrom().getStripped()
		if type in ['message', 'chat', None] and fromjid == self.remotejid:
			sys.stdout.write(event.getBody() + '\n')

	def send_message(self, message):
		m = xmpp.protocol.Message(to=self.remotejid,body=message,typ='chat')
		self.client.send(m)
	
	def set_status(self, s, msg = None):
		print "Setting status to '" + s + "'"
		pres = xmpp.protocol.Presence(priority=5, show=s,status=msg)
		self.client.send(pres)
	
	def xmpp_connect(self):
		con=self.client.connect()
		if not con:
			sys.stderr.write('could not connect!\n')
			return False
		sys.stderr.write('connected with %s\n'%con)
		auth=self.client.auth(self.jid.getNode(),self.password,resource=self.jid.getResource())
		if not auth:
			sys.stderr.write('could not authenticate!\n')
			return False
		sys.stderr.write('authenticated using %s\n'%auth)
		self.register_handlers()
		
		self.client.sendInitPresence()
		return con

	def process(self):
		self.client.Process(1)

	def get_socket(self):
		return self.client.Connection._sock


if __name__ == '__main__':

	if len(sys.argv) < 2:
		print "Syntax: xtalk JID"
		sys.exit(0)

	tojid=sys.argv[1]
	
	jidparams={"jid":"kimp@jabber.ccc.de", "password":"kimp"}

	bot=XmppClient(jidparams,tojid)

	socketlist = {bot.get_socket():'xmpp',sys.stdin:'stdio'}
	online = 1

	while online:
		(i , o, e) = select.select(socketlist.keys(),[],[],1)
		for each in i:
			if socketlist[each] == 'xmpp':
				bot.process()
			elif socketlist[each] == 'stdio':
				msg = sys.stdin.readline().rstrip('\r\n')
				bot.send_message(msg)
			else:
				raise Exception("Unknown socket type: %s" % repr(socketlist[each]))
