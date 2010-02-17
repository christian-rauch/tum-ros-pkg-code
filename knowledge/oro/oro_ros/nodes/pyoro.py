#!/usr/bin/env python
import socket, json

class OroServerError(Exception):
	def __init__(self, value):
		self.value = value
	def __str__(self):
		return repr(self.value)
	
class Oro(object):
	def __init__(self, host, port):
		#create an INET, STREAMing socket
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		#now connect to the oro server
		self.s.connect((host, port))
		self.server = self.s.makefile()
		
		#get the list of methods currenlty implemented by the server
		try:		
			res = self.call_server(["listSimpleMethods"])
			self.rpc_methods = [(t.split('(')[0], len(t.split(','))) for t in res]
		except OroServerError:
			print('Cannot initialize the oro connector! Smthg wrong with the server!')
			self.server.close()
			self.s.close()			
			exit()			   
		
		#add the the Oro class all the methods the server declares
		for m in self.rpc_methods:
			self.add_methods(m)

	def __del__(self):
		close()
	
	def call_server(self, req):
		for r in req:		
			self.server.write(r)
			self.server.write("\n")
			
		self.server.write("#end#\n")
		self.server.flush()
		
		status = self.server.readline().rstrip('\n')
		
		if status == 'ok':
			raw = self.server.readline().rstrip('\n')
			self.server.readline() #remove the trailing #end#
			if raw == '':
				return
			#print "Raw: " + raw
			res = encode8bits(json.loads(raw))
			#print "After JSON loading: " + str(res)
			return res
		else:
			msg = self.server.readline().rstrip('\n') + ": " + self.server.readline().rstrip('\n')
			self.server.readline() #remove the trailing #end#
			raise OroServerError(msg)
		
	def add_methods(self, m):
		def innermethod(*args):
			req = ["%s" % m[0]]
			for a in args:
				req.append(a)
			return self.call_server(req)
				
		innermethod.__doc__ = "This method is a proxy for the oro-server %s method." % m[0]
		innermethod.__name__ = m[0]
		setattr(self,innermethod.__name__,innermethod)
	
	def close(self):
		print('Closing the connection to ORO...')
		self.server.close()
		self.s.close()
		print('Done. Bye bye!')


def encode8bits(o):
	if isinstance(o, list):
		return [encode8bits(o2) for o2 in o]
	return o.encode('ascii')


if __name__ == '__main__':

	HOST = 'localhost'	# ORO-server host
	PORT = 6969		# ORO-server port

	try:
		oro = Oro(HOST, PORT)
		
		oro.add("[johnny rdf:type Human, johnny rdfs:label \"A que Johnny\"]")
		oro.add("[hrp2 rdf:type Robot]")
		print(oro.lookup("A que Johnny")[0])
		print(oro.getSimilarities("johnny", "hrp2"))
		print(oro.getDifferences("johnny", "hrp2"))
	except OroServerError as ose:
		print('Oups! An error occured!')
		print(ose)
	finally:
		oro.close()
