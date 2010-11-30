package edu.tum.cs.ias.knowrob;


import java.io.BufferedReader;
import java.io.IOException;
import java.util.ArrayList;
import processing.core.PApplet;

import ros.pkg.roslib.msg.Header;
import ros.pkg.triangle_mesh_msgs.msg.TriangleMesh;
import ros.pkg.triangle_mesh_msgs.msg.Triangle;
import ros.pkg.geometry_msgs.msg.Point32;


public class MeshVisApplet extends PApplet {

  static final long serialVersionUID=0;

  
  
  private ArrayList<float[]> meshpoints = new ArrayList<float[]>();
  private ArrayList<int[]>   meshtriangles = new ArrayList<int[]>();
  float minX=1e10f, minY=1e10f, minZ=1e10f;
  float maxX=-1e10f, maxY=-1e10f, maxZ=-1e10f;
    
  
  
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  // 
  // INITIALIZATION FUNCTIONS
  // 
  
  /**
   * initialization.
   * automatically called once when applet is started.
   */
  public void setup() {

	size(700, 600, P3D);
    lights();

    noLoop();
    hint(ENABLE_DEPTH_SORT);
    draw(); 
  }



  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  // 
  // DRAW FUNCTIONS
  // 
  
  
/**
 * draws everything
 */
  public void draw() {


	background(20, 20, 20);
    fill(220);
		
    stroke(150);
	strokeWeight(1.0f);

	
    pushMatrix();

		applyMatrix(	1,  0,  0, 0,
						0,  1,  0, 0,
						0,  0, -1, 0,
						0,  0,  0, 1);
		
		pushMatrix();
		  	rotateZ( -PI/2 );
			rotateY( PI/2 );
			
			// shift to the center of the window
			translate(0, 350,  -300.0f);
			
		    float SCALE = (300/ (maxZ-minZ));
		    translate(SCALE*(-maxX), SCALE*(-maxY+(maxY-minY)/2), SCALE*(-maxZ+(maxZ-minZ)/2));
		   
		    beginShape(TRIANGLES);
			      
			    for (int i = 0; i < meshtriangles.size(); i++) {
			    	
			      int p0 = meshtriangles.get(i)[0]; 
			      int p1 = meshtriangles.get(i)[1];
			      int p2 = meshtriangles.get(i)[2];
		      
		          vertex(SCALE*meshpoints.get(p0)[0], SCALE*meshpoints.get(p0)[1], SCALE*meshpoints.get(p0)[2]);
		          vertex(SCALE*meshpoints.get(p1)[0], SCALE*meshpoints.get(p1)[1], SCALE*meshpoints.get(p1)[2]);
		          vertex(SCALE*meshpoints.get(p2)[0], SCALE*meshpoints.get(p2)[1], SCALE*meshpoints.get(p2)[2]);
			        
			    }
		    endShape();
			
			
	      popMatrix(); 
	  popMatrix();

  	}

  
    
    
    // ////////////////////////////////////////////////////////////////////////////////////////////
    // ////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    // 
    // HELPER FUNCTIONS
    // 
    

    /**
     * reads the mesh data from a file.
     * @param file
     */
    public void readMeshFile(String file) {
  	    
    	this.clear();
    	
  	    BufferedReader reader = createReader(file);
  	    try{
  	      
  	      String line;
  	      boolean pointFlag=false, triangleFlag=false;
  	      
  	      // offset needed when reading multiple meshes to a single array
  	      int ptOffset = meshpoints.size();

  	      while(true) {
  	        
  	        line = reader.readLine();
  	        if(line==null){break;}
  	        
  	        
  	        // read point data
  	        if(pointFlag && (line.trim().matches("\\-?[\\d]*\\.?[\\d]*e?\\-?[\\d]* \\-?[\\d]*\\.?[\\d]*e?\\-?[\\d]* \\-?[\\d]*\\.?[\\d]*e?\\-?[\\d]*"))) { 
  	          String[] coords = line.split(" ");
  	          if(coords.length==3) {
  	            
  	        	float x = Float.valueOf(coords[0]);
  	        	float y = Float.valueOf(coords[1]);
  	        	float z = Float.valueOf(coords[2]);
  	        	  
  	            this.meshpoints.add(new float[] {x,y,z});
  	            
  	            if(x<minX) minX=x;	if(x>maxX) maxX=x;
  	            if(y<minY) minY=y;  if(y>maxY) maxY=y;
  	            if(z<minZ) minZ=z;  if(z>maxZ) maxZ=z;
  	            
  	          }
  	          continue;
  	        }
  	        

  	        // read triangle data
  	        if(triangleFlag && (line.trim().matches("3 [\\d]* [\\d]* [\\d]*"))) {
  	          String[] pts = line.split(" ");
  	          if(pts.length==4) {
  	            
  	            this.meshtriangles.add(new int[] {
  	                              Integer.valueOf(pts[1])+ptOffset, 
  	                              Integer.valueOf(pts[2])+ptOffset,
  	                              Integer.valueOf(pts[3])+ptOffset});
  	            
  	          }          
  	          continue;}
  	        
  	        
  	        if(line.matches("POINTS.*")) {
  	          pointFlag=true;
  	          triangleFlag=false;
  	           continue;
  	        }
  	        
  	        if(line.matches("POLYGONS.*")) {
  	          pointFlag=false;
  	          triangleFlag=true;
  	           continue;
  	        }
  	      }
  	      this.redraw();
  	      
  	    } catch (IOException e) {}
  	  }

    
    void clear() {
    	this.meshpoints.clear();
    	this.meshtriangles.clear();
    	this.redraw();
    	
    	this.minX=1e10f;
    	this.minY=1e10f;
    	this.minZ=1e10f;
    	    	
    	this.maxX=-1e10f;
    	this.maxY=-1e10f;
    	this.maxZ=-1e10f;

    }
    
    /**
     * reads the mesh data from a TriangleMesh message
     * @param mesh
     */
    public void readTriangleMesh(TriangleMesh mesh) {

    	// read point data
    	for(Point32 p : mesh.points) {
    	
            this.meshpoints.add(new float[] {p.x,p.y,p.z});
            
            if(p.x<minX) minX=p.x;	if(p.x>maxX) maxX=p.x;
            if(p.y<minY) minY=p.y;  if(p.y>maxY) maxY=p.y;
            if(p.z<minZ) minZ=p.z;  if(p.z>maxZ) maxZ=p.z;
		}

    	// read triangle data
    	for(Triangle t : mesh.triangles) {
          this.meshtriangles.add(new int[] {
		          Integer.valueOf(t.i), 
		          Integer.valueOf(t.j),
		          Integer.valueOf(t.k)});
    	}
    	this.redraw();
    }
    
    
    /**
     * creates a TriangleMesh message from the internally saved data
     */
    TriangleMesh getTriangleMesh() {
    	
    	TriangleMesh res = new TriangleMesh();
    	
    	// fill points
    	Point32[] points = new Point32[this.meshpoints.size()];
    	int i=0;
    	for(float[] p : this.meshpoints) {
    		points[i]     = new Point32();
    		points[i].x   = p[0];
    		points[i].y   = p[1];
    		points[i++].z = p[2];
    		
    	}
    	res.points = points;
    	
    	
    	// fill triangles
    	Triangle[] triangles = new Triangle[this.meshtriangles.size()];
    	i=0;
    	for(int[] t : this.meshtriangles) {
    		
    		triangles[i] = new Triangle();
    		triangles[i].i = t[0];
    		triangles[i].j = t[1];
    		triangles[i++].k = t[2];
    		
    	}
    	res.triangles = triangles;
    	
    	res.intensities = new float[1];
    	res.sending_node="mesh_label_test_client";
    	res.normals = new Point32[1];
    	res.normals[0] = new Point32();
    	res.normals[0].x=1f;res.normals[0].y=1f;res.normals[0].z=1f;
    	res.header = new Header();
    	res.header.frame_id="bla";
    	res.header.seq=1;
    	res.header.stamp = ros.communication.Time.now();

    	
    	return res;
    }
    
}


