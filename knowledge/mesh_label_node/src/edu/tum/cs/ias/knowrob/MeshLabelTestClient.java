package edu.tum.cs.ias.knowrob;

import ros.*;
import ros.pkg.mesh_label_node.srv.LabelMesh;


public class MeshLabelTestClient {

		
	public static void main(String args[]) {
		  
		Ros ros = Ros.getInstance();
		if(!Ros.getInstance().isInitialized()) {
			ros.init("mesh_label_test_client");
		}
		NodeHandle n = ros.createNodeHandle();
		
		MeshVisApplet m = new MeshVisApplet();
		
		
		for(String file : new String[]{ "./vtk/cup_handle_side_from_cylinder.vtk",
										//"./vtk/cup_handle_side.vtk",
										//"./vtk/teapot_green_order_5_th_0004.vtk",
										//"./vtk/teapot.vtk",
										//"./vtk/cornflakes.vtk"
				}) {

			//m.readMeshFile();
			m.readMeshFile(file);

			//mesh_applet.readMeshData("./vtk/teapot_green_order_5_th_0004.vtk");
			
			LabelMesh.Request req = new LabelMesh.Request();
			req.mesh = m.getTriangleMesh();
			
			ServiceClient<LabelMesh.Request, LabelMesh.Response, LabelMesh> cl = 
								n.serviceClient("/mesh_label_node", new LabelMesh());
			
			try {
				LabelMesh.Response res = cl.call(req);
				System.out.println("Label: " +res.label);
				
			} catch (RosException e) {
				e.printStackTrace();
			}
			
		}
		

		
	}
}

