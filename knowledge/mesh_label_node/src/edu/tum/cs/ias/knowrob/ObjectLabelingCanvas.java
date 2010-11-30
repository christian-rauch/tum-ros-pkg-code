package edu.tum.cs.ias.knowrob;

import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import controlP5.ControlP5;
import processing.core.*;
import ros.*;
import ros.pkg.mesh_label_node.srv.LabelMesh;


public class ObjectLabelingCanvas extends PApplet implements MouseListener, MouseMotionListener {

	private static final long serialVersionUID = 4575739930038583994L;
	public enum Part {KITCHEN_VIS, ACTION_VIS, VIDEO, ALL}; 

	  
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
	
	// components
	protected MeshLabelApplet label_applet;
	protected MeshVisApplet mesh_applet;
	
	public ControlP5 controlP5;
	
	private String label="";
	private String label_set="";
	
	public ObjectLabelingCanvas() {
		
		this.init();


		label_applet = new MeshLabelApplet();
		label_applet.setSize(700, 80);
		label_applet.init();
		
		label_applet.setObjectLabelingCanvas(this);
		
		mesh_applet = new MeshVisApplet();
		mesh_applet.setSize(700, 600);
		mesh_applet.init();
		
		this.add(label_applet);
		this.add(mesh_applet);
	

		// start ROS environment
		new LabelMeshCallback();
	    
	    Thread startRosServices = new Thread( new RosServiceThread() ); 
	    startRosServices.start();
		
		
		this.setSize(700, 700);
		this.draw();
		this.setVisible(true);
		
	}
	
	
	public void draw() {
		background(20, 20, 20);
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
	  			
				n.advertiseService("/mesh_label_node",  new LabelMesh(),  new LabelMeshCallback());
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
		class LabelMeshCallback implements ServiceServer.Callback<LabelMesh.Request, LabelMesh.Response> {
			
			@Override
			public LabelMesh.Response call(LabelMesh.Request req) {

				LabelMesh.Response res = new LabelMesh.Response();
				res.label = "";

				if (req.mesh != null) {
					
					mesh_applet.readTriangleMesh(req.mesh);
					
					// wait for the GUI to set the label and notify me
					synchronized( label_set ) 
					{ 
					  try { 
					    label_set.wait(); 
					    res.label = label;
					    return res;
					    
					  } catch ( InterruptedException e ) { 
						  e.printStackTrace();
					  } 
					}
				}
				return res;
				
			}
		}
		
	// called from the GUI: label set, notify waiting ROS answer thread
	public void setLabel(String l) {
		
		synchronized(label_set) {
			label=l;
			label_set.notify();
		}
	}
		
		
	public static void main(String args[]) {
		PApplet.main(new String[] { "edu.tum.cs.ias.knowrob.ObjectLabelingCanvas" });
	}
}

