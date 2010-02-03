/*
 * Copyright (C) 2009 by Moritz Tenorth
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

import ros.Ros;
import ros.NodeHandle;
import ros.RosException;
import ros.ServiceClient;
import ros.pkg.vision_srvs.srv.srvjlo;
import ros.pkg.vision_srvs.srv.srvjlo.Response;
import ros.pkg.vision_srvs.srv.srvjlo.Request;
import ros.pkg.vision_msgs.msg.partial_lo;
import java.util.concurrent.ExecutionException;

public class KBClient {

	static boolean rosInitialized = false;

    public static void main(String [] args) throws InterruptedException, RosException, ExecutionException {

    }

    public static Ros initRosInstance() {
    	
    	Ros ros = Ros.getInstance();
    	
    	if(!rosInitialized) {
	 		ros.init("kb_jlo_client");
	 		rosInitialized=true;
    	}
    	
	 	return ros;	
    }

	
	public static partial_lo loQuery(String command, String objName, int objID, int refFrame) throws RosException {
		
		Ros ros = initRosInstance();
		NodeHandle n = ros.createNodeHandle();
	 	
	 	ServiceClient<Request, Response, srvjlo> client = n.serviceClient("/located_object", new ros.pkg.vision_srvs.srv.srvjlo());
	
	 	srvjlo srv = new srvjlo();
	 	Request rq = srv.createRequest();
	 	
	 	rq.command=command;
	 	
	 	if(command.equals("namequery")) {
	 		rq.query.name=objName;
	 	}
	 	else if(command.equals("framequery")) {
	 		rq.query.id=objID;
	 		
	 		if(refFrame>0)
	 			rq.query.parent_id=refFrame;
	 	}
	 	
	 	return client.call(rq).answer;
	}
	

	
    
}
