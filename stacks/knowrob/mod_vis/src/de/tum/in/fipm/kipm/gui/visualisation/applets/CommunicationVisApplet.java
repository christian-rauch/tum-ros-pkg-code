package de.tum.in.fipm.kipm.gui.visualisation.applets;

import java.util.ArrayList;
import processing.core.*;
import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceClient;
import ros.ServiceServer;
import ros.pkg.mod_vis.srv.CommVisSetLeftImg;
import ros.pkg.mod_vis.srv.CommVisSetRightImg;
import ros.pkg.mod_vis.srv.CommVisSetReqText;
import ros.pkg.mod_vis.srv.CommVisSetResText;
import de.tum.in.fipm.kipm.gui.visualisation.base.PrologVisualizationCanvas;

public class CommunicationVisApplet extends PApplet {

  private static final long serialVersionUID = -3913679727924919169L;
  private PrologVisualizationCanvas prologVisCanvas = null;

  ////////////////////////////////////////////////////////////////////////////////
  // DISPLAY PROPERTIES (ROTATION, ZOOM, ...)

  PFont verdana;
  PFont verdanaBold;
  PFont dejavu;

  PImage leftImg, rightImg;
  

  
  ////////////////////////////////////////////////////////////////////////////////
  // BUFFERS
  ArrayList<Integer> colors = new ArrayList<Integer>(12);	// the colors used in the vis. HumanTrajVis

  int transmitting = 0;
  int receiving    = 0;
  int FRAMES_TO_DISPLAY_MESSAGES = 50;
  
  int LINE_LEFT_START = 250;
  int LINE_RIGHT_END  = 550;
  int LINE_WIDTH=LINE_RIGHT_END-LINE_LEFT_START;
  
  int UPPER_LINE_YPOS = 280;
  int LOWER_LINE_YPOS = 330;

  int transmitPosition=LINE_LEFT_START;
  int receivePosition=LINE_RIGHT_END;
  
  SetLeftImgCallback leftImgCallback;
  SetRightImgCallback rightImgCallback;
  SetRequestTextCallback requestTextCallback;
  SetResponseTextCallback responseTextCallback;
  
  
  String request  = "rdf_has(cupboard0, rdf:type, knowrob:'Handle'), \n" +
				   "rdf_has(cupboard0,knowrob:widthOfObject,literal(type(_,W))), \n" + 
				   "rdf_has(cupboard0,knowrob:heightOfObject,literal(type(_,H))), \n" + 
				   "rdf_has(cupboard0,knowrob:depthOfObject,literal(type(_,D))), \n" + 
				   "rdf_triple(knowrob:orientation,cupboard0,Or), \n" +
				   "rdf_triple(knowrob:m03, Or, literal(type(_,POS_X))),\n" +  
				   "rdf_triple(knowrob:m13, Or, literal(type(_,POS_Y))),\n" + 
				   "rdf_triple(knowrob:m23, Or, literal(type(_,POS_Z)))";
  
  String response = "A = 'http://ias.cs.tum.edu/kb/knowrob.owl#FoodOrDrink' ;\n" +
			  		"A = 'http://ias.cs.tum.edu/kb/knowrob.owl#Drink' ;\n" +
			  		"A = 'http://ias.cs.tum.edu/kb/knowrob.owl#Coffee-Beverage' ;\n" +
			  		"A = 'http://ias.cs.tum.edu/kb/knowrob.owl#InfusionDrink' ;\n" +
			  		"A = 'http://ias.cs.tum.edu/kb/knowrob.owl#Tea-Beverage' ;\n" +
			  		"A = 'http://ias.cs.tum.edu/kb/knowrob.owl#Tea-Iced' ";

  
	static Ros ros;
	static NodeHandle n;
	

	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

    	ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
	    	ros.init("knowrob_comm_vis_applet");
		}
		n = ros.createNodeHandle();

	}
  

  /**
   * Applet setup routine
   */
  public void setup() {
 
	size(800, 600, P2D);
    
    dejavu = createFont("DejaVu Sans",13);
    textFont(dejavu);    
    hint(ENABLE_ACCURATE_TEXTURES);
    ellipseMode(RADIUS);
    frameRate(25);
    
    
    leftImg  = loadImage("images/rosie.png");
    rightImg = loadImage("images/roboearth.png");
    //rightImg = loadImage("/work/ros/tumros-internal/stacks/knowrob/mod_vis/images/pr2.jpg");
    
    // init ros environment
    leftImgCallback = new SetLeftImgCallback();
    rightImgCallback =  new SetRightImgCallback();
    requestTextCallback = new SetRequestTextCallback();
    responseTextCallback = new SetResponseTextCallback();
    
    Thread startRosServices = new Thread( new RosServiceThread() ); 
    startRosServices.start();
    
    
    draw();    
    if(prologVisCanvas != null) prologVisCanvas.validate();
    
  }


  /**
   * drawing function, is called at 25Hz
   */
  public void draw() {
    
	background(20, 20, 20);
	
    cursor(CROSS);

    textFont(dejavu);
	textMode(SCREEN);
	fill(0xFFFFFFFF);stroke(0xFFFFFFFF);
  
	// draw images at the ends of the communication
	synchronized(leftImg) {
	if(leftImg != null)
		image(leftImg,  ((int)(150-0.5*leftImg.width)), ((int)(300-0.5*leftImg.height)));
	}
	
	synchronized(rightImg) {
	if(rightImg != null)
		image(rightImg, ((int)(650-0.5*rightImg.width)), ((int)(300-0.5*rightImg.height)));
	}
	
	// draw dotted line
	for(int x=LINE_LEFT_START;x<LINE_RIGHT_END;x+=20) {
	    line(x, UPPER_LINE_YPOS, x+10, UPPER_LINE_YPOS);
	    line(x, LOWER_LINE_YPOS, x+10, LOWER_LINE_YPOS);
	}
	
	// display query text, draw moving circle
 	if(transmitting>0) {
 		synchronized(request) {
 			textAlign(LEFT, BOTTOM);
 			text(request,  250, 30,  300, 230);
 		}
		ellipse(transmitPosition, UPPER_LINE_YPOS, 3, 3);
		transmitPosition= LINE_LEFT_START + ((transmitPosition - LINE_LEFT_START + 10)%LINE_WIDTH);
		transmitting--;
 	}

	// display response text, draw moving circle
 	if(receiving>0) {
 		synchronized(response) {
 			textAlign(LEFT, TOP);
 			text(response, 250, 370, 300, 250);
 		}
 		ellipse(receivePosition,  LOWER_LINE_YPOS, 3, 3);
 		receivePosition = LINE_RIGHT_END  - ((LINE_RIGHT_END   - receivePosition + 10)%LINE_WIDTH);
 		receiving--;
 	}
  }

  /**
   * set the request text and activate transmission display
   * @param request the query text to be displayed
   */
  public void setRequest(String req) {
	  
	  synchronized(request) {
		  this.request = req;
	  }
	  this.setTransmitting(true);

	  draw(); 
  }

  
  /**
   * set the response text and activate reception display
   * @param response the response text to be displayed
   */
  public void setResponse(String response) {
	  synchronized(response) {
		  this.response = response;
	  }
	  this.setReceiving(true);

	  draw(); 
  }

  
  /**
   * set the image on the left hand side of the communication
   * @param filename
   */
  public void setLeftImage(String filename) {
	  synchronized(leftImg) {
		  this.leftImg  = loadImage("images/"+filename);
	  }
  }

  
  /**
   * set the image on the right hand side of the communication
   * @param filename
   */
  public void setRightImage(String filename) {
	  synchronized(rightImg) {
		  this.rightImg = loadImage("images/"+filename);
	  }
  }

  /**
   * activate transmission display for the next FRAMES_TO_DISPLAY_MESSAGES frames
   * @param on switch transmission display on or off
   */
  public void setTransmitting(boolean on) {
	  if(on)
		  this.transmitting += FRAMES_TO_DISPLAY_MESSAGES;
	  else
		  this.transmitting = 0;
  }
  
  /**
   * activate reception display for the next FRAMES_TO_DISPLAY_MESSAGES frames
   * @param on switch reception display on or off
   */
  public void setReceiving(boolean on) {
	  if(on)
		  this.receiving += FRAMES_TO_DISPLAY_MESSAGES;
	  else
		  this.receiving = 0;
  }

  /**
   * debug: simulate visualization events
   */
  public void keyPressed(){

	switch(keyCode) {
		case 82: // r
			setReceiving(true);
			break;
			
		case 84: // t
			setTransmitting(true);
			break;
	}
	
	delay(50);
  	redraw();
  } 
  
  /**
   * 
   * Simple thread that creates the ROS services which update the internal state variables
   * 
   * @author tenorth@cs.tum.edu
   *
   */
  public class RosServiceThread implements Runnable {
  	
  	@Override public void run() {
  		
  		try {

  			initRos();
  			
			n.advertiseService("comm_vis_set_left_img",  new CommVisSetLeftImg(),  new SetLeftImgCallback());
			n.advertiseService("comm_vis_set_right_img", new CommVisSetRightImg(), new SetRightImgCallback());
			n.advertiseService("comm_vis_set_req_text",  new CommVisSetReqText(),  new SetRequestTextCallback());
			n.advertiseService("comm_vis_set_res_text",  new CommVisSetResText(),  new SetResponseTextCallback());
			ros.spin();
	    		
  		} catch(RosException e) {
  			e.printStackTrace();
  		}
  	}
  }
  
  
	/**
	 * 
	 * The callback class for updating the left image in the visualization
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class SetLeftImgCallback implements ServiceServer.Callback<CommVisSetLeftImg.Request, CommVisSetLeftImg.Response> {
		
		@Override
		public CommVisSetLeftImg.Response call(CommVisSetLeftImg.Request req) {

			CommVisSetLeftImg.Response res = new CommVisSetLeftImg.Response();
			res.success = 0;

			if (req.filename != null && req.filename.length() > 0) {
				setLeftImage(req.filename);
			}

			return res;
		}
	}
	  
	/**
	 * 
	 * The callback class for updating the right image in the visualization
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class SetRightImgCallback implements ServiceServer.Callback<CommVisSetRightImg.Request, CommVisSetRightImg.Response> {
		
		@Override
		public CommVisSetRightImg.Response call(CommVisSetRightImg.Request req) {

			CommVisSetRightImg.Response res = new CommVisSetRightImg.Response();
			res.success = 0;

			if (req.filename != null && req.filename.length() > 0) {
				setRightImage(req.filename);
			}

			return res;
		}
	}
	  
	/**
	 * 
	 * The callback class for updating the request text in the visualization
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class SetRequestTextCallback implements ServiceServer.Callback<CommVisSetReqText.Request, CommVisSetReqText.Response> {

		@Override
		public CommVisSetReqText.Response call(CommVisSetReqText.Request req) {

			CommVisSetReqText.Response res = new CommVisSetReqText.Response();
			res.success = 0;

			if (req.request != null && req.request.length() > 0) {
				setRequest(req.request);
			}

			return res;
		}
	}	  
	
	/**
	 * 
	 * The callback class for updating the response text in the visualization
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class SetResponseTextCallback implements ServiceServer.Callback<CommVisSetResText.Request, CommVisSetResText.Response> {

		@Override
		public CommVisSetResText.Response call(CommVisSetResText.Request req) {

			CommVisSetResText.Response res = new CommVisSetResText.Response();
			res.success = 0;

			if (req.response != null && req.response.length() > 0) {
				setResponse(req.response);
			}

			return res;
		}
	}
	
  

	/**
	 * Helper functions for visualization: send visualization strings to an instance of the
	 * CommunicationVisApplet to display the ongoing communication.
	 * 
	 * @param req The  String to be displayed in the 'request' field of the visualization
	 * @param res The  String to be displayed in the 'response' field of the visualization
	 * @param leftImg  Image to be displayed on the left side of the visualization
	 * @param rightImg Image to be displayed on the right side of the visualization 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 */
	public static void visualizeCommunication(String req, String res, String leftImg, String rightImg) {
	
		try {

  			initRos();
  			
			if(leftImg!=null) {

				CommVisSetLeftImg.Request vis_req = new CommVisSetLeftImg.Request();
				vis_req.filename = leftImg;
				ServiceClient<CommVisSetLeftImg.Request, CommVisSetLeftImg.Response, CommVisSetLeftImg> cl = 
					n.serviceClient("/comm_vis_set_left_img", new CommVisSetLeftImg());
				cl.call(vis_req);
				cl.shutdown();
			}
			
			if(rightImg!=null) {

				CommVisSetRightImg.Request vis_req = new CommVisSetRightImg.Request();
				vis_req.filename = rightImg;
				ServiceClient<CommVisSetRightImg.Request, CommVisSetRightImg.Response, CommVisSetRightImg> cl = 
					n.serviceClient("/comm_vis_set_right_img", new CommVisSetRightImg());
				cl.call(vis_req);
				cl.shutdown();
			}
			
			if(req!=null) {

				CommVisSetReqText.Request vis_req = new CommVisSetReqText.Request();
				vis_req.request = req;
				ServiceClient<CommVisSetReqText.Request, CommVisSetReqText.Response, CommVisSetReqText> cl = 
					n.serviceClient("/comm_vis_set_req_text", new CommVisSetReqText());
				cl.call(vis_req);
				cl.shutdown();
			}
			
			if(res!=null) {

				CommVisSetResText.Request vis_req = new CommVisSetResText.Request();
				vis_req.response = res;
				ServiceClient<CommVisSetResText.Request, CommVisSetResText.Response, CommVisSetResText> cl = 
					n.serviceClient("/comm_vis_set_res_text", new CommVisSetResText());
				cl.call(vis_req);
				cl.shutdown();

			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	

	
  
  public void setPrologVisCanvas(PrologVisualizationCanvas c){
	  prologVisCanvas = c;
  }
  
  public static void main(String args[]) {
	  PApplet.main(new String[] { "de.tum.in.fipm.kipm.gui.visualisation.applets.CommunicationVisApplet" });
  }
}

