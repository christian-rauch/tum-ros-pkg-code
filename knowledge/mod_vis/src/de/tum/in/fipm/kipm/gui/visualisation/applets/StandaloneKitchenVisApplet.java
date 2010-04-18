package de.tum.in.fipm.kipm.gui.visualisation.applets;


import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.BufferedReader;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;

import java.util.HashMap;
import java.util.Vector;

import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.Timer;

import processing.core.PFont;
import processing.core.PMatrix;

import de.tum.in.fipm.kipm.gui.visualisation.base.PrologVisualizationCanvas;
import de.tum.in.fipm.kipm.gui.visualisation.items.*;
import de.tum.in.fipm.kipm.util.datastructures.Point;

import edu.tum.cs.vis.AnimatedCanvas;
import edu.tum.cs.vis.Canvas;



public class StandaloneKitchenVisApplet extends AnimatedCanvas implements MouseListener, MouseMotionListener {

  static final long serialVersionUID=0;

  ////////////////////////////////////////////////////////////////////////////////
  // DISPLAY PROPERTIES (ROTATION, ZOOM, ...)
  private float leftMouseX=-1.0f, leftMouseY=-1.0f, rightMouseX=-1.0f, rightMouseY=-1.0f, centerMouseY=-1.0f;
  private float xRotDisplay=30.4f, yRotDisplay=-20.75f, xShiftDisplay=1.0f, yShiftDisplay=369f, zoomDisplay=1.0f;
  
  
  ////////////////////////////////////////////////////////////////////////////////
  // SETTINGS
  private String meshFile = "/home/tenorth/work/kipm/meshdata/unclassified300p.vtk";

  
  ////////////////////////////////////////////////////////////////////////////////
  // MISC
  private PFont verdana;
  private PFont verdanaBold;
  private PFont dejavu;
  private ArrayList<int[]> colors = new ArrayList<int[]>(12);
  private PrologVisualizationCanvas prologVisCanvas = null;
  public MenuHuman menu_human = new MenuHuman();
  
  
  ////////////////////////////////////////////////////////////////////////////////
  // COLOR STUFF / CLICK STUFF
  private int[] grayValues = new int[] {160,190,210,170,220,180,200,165,185,205,175,195,215};
  private static int grayLevelCounter = 0;
  private EmptyCanvas buffer;
  private JFrame bufferFrame;
  private PMatrix bufferMatrix;
  
  
  ////////////////////////////////////////////////////////////////////////////////
  // BUFFERS, DATA
  
  /**
   * - items are usually to be referenced by their prolog identifier.
   * - the skeleton is referenced by "skeleton".
   */
  private HashMap<String,ItemBase> animatedItemsRef = new HashMap<String, ItemBase>();
  private ArrayList<ItemBase> allItems = new ArrayList<ItemBase>();
  private ArrayList<float[]> meshpoints = new ArrayList<float[]>();
  private ArrayList<int[]>   meshtriangles = new ArrayList<int[]>();
  private float[] framesToTimes;
  private String currentlyActiveItem = null;

  
  ////////////////////////////////////////////////////////////////////////////////
  // INFO ABOUT CURRENT DISPLAY
  private double startTime = 0;
  private double endTime = 0;
  private int numberFrames = 1;
  private int currentFrame = 0;
  private String queryString = "";
  
  
  
  
  ////////////////////////////////////////////////////////////////////////////////
  // FLAGS
  private boolean drawMeshes = true;
  public boolean isInitialized = false;
  private boolean playingForward = true;
  
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
/*
	  if(isInitialized) return;
	  
	  buffer = new Canvas();
	  buffer.init();
	  buffer.setup();
	  buffer.size(700,600,P3D);
	  buffer.noLights();

*/
	
	
	size(700, 600, P3D);
    lights();
    
    
    buffer = new EmptyCanvas(700,600,P3D);
    
    verdana = createFont("Verdana", 11);
    verdanaBold = createFont("Verdana-Bold", 18);
    dejavu = createFont("DejaVu Sans",13);
    textFont(verdana);
    textFont(verdanaBold);
    textFont(dejavu);    

    ellipseMode(RADIUS);
    frameRate(20);
    
    setColors();
 
    noLoop();
    
    drawBackground();
    draw();
    isInitialized = true;
    if(prologVisCanvas != null) prologVisCanvas.validate();
    

    
    
  }




  /**
   * reads the mesh data from a file.
   * @param file
   */
  private void readMeshData(String file) {
	    
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
	        if(pointFlag && (line.matches("\\-?[\\d]*\\.?[\\d]*e?\\-?[\\d]* \\-?[\\d]*\\.?[\\d]*e?\\-?[\\d]* \\-?[\\d]*\\.?[\\d]*e?\\-?[\\d]*"))) { 
	          String[] coords = line.split(" ");
	          if(coords.length==3) {
	            
	            this.meshpoints.add(new float[] {
	                              Float.valueOf(coords[0]), 
	                              Float.valueOf(coords[1]),
	                              Float.valueOf(coords[2])});
	          }
	          continue;
	        }
	        

	        // read triangle data
	        if(triangleFlag && (line.matches("3 [\\d]* [\\d]* [\\d]*"))) {
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
	      
	    } catch (IOException e) {}
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
	  PMatrix save = getMatrix();
	  try{
	background(20, 20, 20);
    cursor(CROSS);
    
    pushMatrix();
	    lights();
		applyMatrix(	1,  0,  0, 0,
						0,  1,  0, 0,
						0,  0, -1, 0,
						0,  0,  0, 1);
	
		
	    scale(zoomDisplay);
		translate(xShiftDisplay, yShiftDisplay, 0.0f);
		
		
		pushMatrix();
		  	rotateZ( -PI/2 );
			rotateY( PI/2 );


			rotateY( radians( -yRotDisplay) );
			rotateZ( radians(xRotDisplay) );
	      
			strokeWeight(1.5f);
			stroke(255, 0, 0);      line(0, 0, 0,   500, 0, 0);
			stroke(0, 255, 0);      line(0, 0, 0,   0, 500, 0); 
			stroke(0, 0, 255);      line(0, 0, 0,   0, 0, 500);
			stroke(0, 0, 0);
	      
			// draw the meshes
			if(drawMeshes)
				drawMeshes();
	      
			// draw all Items
			for(int i=0;i<allItems.size();i++)
				allItems.get(i).draw(this, currentFrame);	


			
			
			drawTime();
	      popMatrix(); 
	  popMatrix();
	  
	  //this.menu_human.drawMenu();
	  }catch(ArrayIndexOutOfBoundsException e) {
		  System.out.println("Some Drawing error occured... ");
		  resetMatrix();
		  applyMatrix(save);
	  }
 }
  
  private String getItemAt(int x, int y) {
	  
	  String[] id = null;
	  int clicked = -1;
	  
	  try{
	  
	  	if(bufferMatrix==null)
			bufferMatrix = buffer.getMatrix();
			
	  	
	  	buffer.setSize(700, 600);
	  	
	  	buffer.resetMatrix();
	  	buffer.applyMatrix(bufferMatrix);
	  	
	  	
	    buffer.noLights();
	  	buffer.pushMatrix();	    
		    buffer.background(60, 60, 60);
	
		    buffer.applyMatrix(	1,  0,  0, 0,
								0,  1,  0, 0,
								0,  0, -1, 0,
								0,  0,  0, 1);
		    buffer.scale(zoomDisplay);
		    buffer.translate(xShiftDisplay, yShiftDisplay, 0.0f);
		    buffer.ellipseMode(RADIUS);
		    buffer.rotateZ( -PI/2 );
		    buffer.rotateY( PI/2 );
	
		    buffer.rotateY( radians( -yRotDisplay) );
		    buffer.rotateZ( radians(xRotDisplay) );
		  
//			stroke(255, 0, 0);      line(0, 0, 0,   500, 0, 0);
//			stroke(0, 255, 0);      line(0, 0, 0,   0, 500, 0); 
//			stroke(0, 0, 255);      line(0, 0, 0,   0, 0, 500);
//			stroke(0, 0, 0);
	      
		
			// draw all Items
			id = new String[allItems.size()];
			for(int i=0;i<allItems.size();i++) {
				ItemBase itm = allItems.get(i);
				itm.colorOverride = 0xff000000 | ( 1*i + 1);
				allItems.get(i).draw(buffer, currentFrame);
				itm.colorOverride = 0;
				id[i] = itm.name;
			}
	
			// get color
			clicked = ((buffer.get(x,y)-1) & 0x00ffffff)/1;

			buffer.freeze();
			buffer.redraw();
		
			
		buffer.popMatrix();
		
	  }catch(Exception e){}

		if(clicked >= 0 && clicked<id.length)
			return id[clicked];
		else
			return null;
  }
  

  
  /**
   * draws the meshes. if enabled, called by draw().
   */
  private void drawMeshes() {

	  pushMatrix();
	    scale(100);
	    strokeWeight(0.2f);
	    
	    //if(printMode==false)
	    stroke(90, 90, 90);
	    //else 
	    //	stroke(180, 180, 180);
	    
	    noFill();
	    
	    beginShape(TRIANGLES);
	      
	    for (int i = 0; i < meshtriangles.size(); i++) {
	    	
	      int p0 = meshtriangles.get(i)[0]; 
	      int p1 = meshtriangles.get(i)[1];
	      int p2 = meshtriangles.get(i)[2];
      
          vertex(meshpoints.get(p0)[0], meshpoints.get(p0)[1], meshpoints.get(p0)[2]);
          vertex(meshpoints.get(p1)[0], meshpoints.get(p1)[1], meshpoints.get(p1)[2]);
          vertex(meshpoints.get(p2)[0], meshpoints.get(p2)[1], meshpoints.get(p2)[2]);
	        
	    }
	    endShape();
	    popMatrix();
  }

  
  /**
   * draws the time, frame and other textual info
   */
  private void drawTime(){
	  DecimalFormat df2 = new DecimalFormat( "##0.00" );
	  fill(0, 255, 0);
	  textFont(verdana);
	  textMode(SCREEN);
	  textAlign(LEFT); 
	  text("Action: "+queryString,10,50);
	  text("Duration: "+df2.format(startTime)+"-"+df2.format(endTime),10,62);
	  text("Frame: "+currentFrame+" / "+(numberFrames-1),10,86);
	  
	  if(framesToTimes != null)
		  text("Time: "+df2.format(framesToTimes[currentFrame]), 10, 98);
  }
  
  /**
   * prints a message.
   * @param message
   */
  private void displayMessage(String message){
	  System.out.println(message);
	  return;
  }
  public void setPrologVisCanvas(PrologVisualizationCanvas c){
	  prologVisCanvas = c;
  }


  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  // 
  // ADD-FUNCTIONS AS INTERFACE TO EXTERNAL VISUALISATIONS
  // 
 
  /**
   * queries prolog for kitchen layout and draws / initializes the kitchen
   * called by setup()
   * can also be used to reset the applet completely.
   */
  public void drawBackground() {
	  clear();
	  addObjectWithChildren("'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls'");
	  addObjectWithChildren("'http://ias.cs.tum.edu/kb/ias_map_addons.owl#table0'");
  }
  
    
    public void clear() {
    	allItems.clear();
    	animatedItemsRef.clear();
    	startTime = 0;
    	endTime = 0;
    	numberFrames = 1;
    	currentFrame = 0;
    	grayLevelCounter = 0;
    }
  
  	/**
  	 * displays one PROLOG-instance of an action BY IT'S DYNAMIC IDENTIFIER,
  	 * for example "'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching50'"
  	 * 
  	 * @param identifier e.g.: "'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching50'" (WITH single quotes!)
  	 */
	public void displayAction(String identifier)
  	{
		// reset everything
	  	clear();
	  	addObjectWithChildren("'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls'");
	  	addObjectWithChildren("'http://ias.cs.tum.edu/kb/ias_map_addons.owl#table0'");
	  	
	  	
	  	addObject("'http://ias.cs.tum.edu/kb/knowrob.owl#placemat1'");
	  	addObject("'http://ias.cs.tum.edu/kb/ias_entities.owl#napkin1'");
	  	addObject("'http://ias.cs.tum.edu/kb/ias_entities.owl#spoon1'");
	  	addObject("'http://ias.cs.tum.edu/kb/ias_entities.owl#plate1'");
	  	addObject("'http://ias.cs.tum.edu/kb/ias_entities.owl#fork1'");
	  	addObject("'http://ias.cs.tum.edu/kb/ias_entities.owl#knife1'");
	  	
	  	try{
			queryString = identifier;
			HashMap<String, Vector<Object>> c = PrologVisualizationCanvas.executeQuery(
					"rdf_has("+identifier+", 'http://ias.cs.tum.edu/kb/knowrob.owl#startTime', S), "
					+"rdf_has("+identifier+", 'http://ias.cs.tum.edu/kb/knowrob.owl#endTime', E)",null);	
			
			String startTimeStr = (String)c.get("S").get(0);
			startTimeStr = startTimeStr.substring(1,startTimeStr.length()-1);
			
			if(startTimeStr.contains("#")) {
				startTimeStr=startTimeStr.split("#")[1];
				startTimeStr=startTimeStr.split("_")[1];
			}
			System.out.println(startTimeStr);
			
			float startTime = Float.parseFloat(startTimeStr);
			this.startTime = startTime;
			
			
			String endTimeStr = (String)c.get("E").get(0);
			endTimeStr = endTimeStr.substring(1,endTimeStr.length()-1);
			if(endTimeStr.contains("#")) {
				endTimeStr=endTimeStr.split("#")[1];
				endTimeStr=endTimeStr.split("_")[1];
			}
			float endTime = Float.parseFloat(endTimeStr);
			this.endTime = endTime;
			
		
			c = PrologVisualizationCanvas.executeQuery(
					"rdf_has(A, rdf:type, knowrob:'Posture-Configuration'), rdf_has(A, knowrob:bodyPartsUsed, 'left'), rdf_triple(knowrob:startTime, A, T)",null);	
		
			Vector<Object> allTimePoints = c.get("T");
			BodyPoseSequence skeleton = new BodyPoseSequence();
			float[] poses = null;
			int totalFrames = 0;
			
//			System.out.println("total timepoints: "+allTimePoints.size()+", start: "+startTime+", end: "+endTime);
			
			for(Object s : allTimePoints) {
		
				
				String timePointStr = ((String)s).substring(1, ((String)s).length()-1);
				timePointStr = timePointStr.substring(1,timePointStr.length()-1);
				if(timePointStr.contains("#")) {
					timePointStr=timePointStr.split("#")[1];
					timePointStr=timePointStr.split("_")[1];
				}
				float timePoint = Float.parseFloat(timePointStr);
				
				if(timePoint >= startTime && timePoint <= endTime) {
					try{		
						HashMap<String, Vector<Object>> frameInfo = PrologVisualizationCanvas.executeQuery(
								"rdf_triple(ias_human:postureAtTimePoint,  "+s.toString()+", P)",
								null);	
						
						if(frameInfo.get("P") == null || frameInfo.get("P").size() == 0){
							if(poses != null){
								skeleton.addPose(0, timePoint, "", poses);
								System.out.println("pose at point "+s.toString()+" not found; re-added previous pose.");
							}
							else
							{
								System.out.println("pose at point "+s.toString()+" not found.");
							}
							continue;
						}
						
						String[] split = ((String)frameInfo.get("P").get(0)).split("\\)-\\(");
						
						poses = new float[84];			
					  	
						for(int j=0;j<83;j++) {
					  		poses[j] = Float.parseFloat(split[j+4]);
						}
						poses[83] = Float.parseFloat(split[87].substring(0,split[87].length()-2));
						skeleton.addPose(0, timePoint, "", poses);
						totalFrames++;
						//System.out.println("added pose at point: "+s.toString());
					}catch (Exception e) {
						e.printStackTrace();
						System.out.println("ERROR adding point: "+s.toString());
					}					
				}
		
			}
			
			allItems.add(skeleton);
			animatedItemsRef.put("skeleton", skeleton);
			
			numberFrames = totalFrames;
			
			// add fromLocation and toLocation as colored spheres, if they are present
			c = PrologVisualizationCanvas.executeQuery(
					"rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#fromLocation', "+identifier+", From)",null);
			if(c!=null && c.get("From")!= null) {
				String fromLoc = (String)c.get("From").get(0);
				if(fromLoc!=null) {
					addObject(fromLoc);
				}
			}
			
			c = PrologVisualizationCanvas.executeQuery(
					"rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#toLocation', "+identifier+", To)",null);
			if(c!=null && c.get("To")!=null) {
				String toLoc = (String)c.get("To").get(0);
				if(toLoc!=null) {
					addObject(toLoc);
				}
			}
			
	  	}
	  	catch(NullPointerException e)
	  	{
	  		displayMessage("Action "+identifier+" not found.");
	  		drawBackground();
	  	}
	  	catch(Exception e)
	  	{
	  		displayMessage("Error displaying Action "+identifier+".");
	  		e.printStackTrace();
	  		drawBackground();
	  	}
  	}
	

  	/**
  	 * displays one PROLOG-instance of an action BY IT'S FIXED IDENTIFIER,
  	 * for example "'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2'"
  	 * 
  	 * @param identifier e.g.: "'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2'" (WITH single quotes!)
  	 */
  	public void displayActionFixedIdent(String identifier)
  	{
  		queryString = identifier;
		HashMap<String, Vector<Object>> c = PrologVisualizationCanvas.executeQuery(""+
				"readPostureSeqForAction("+identifier+", P, EPISODENR, OCCURRENCENR, INSTANCENR, TIME, BECX, BECY, BECZ, ULWX, ULWY, ULWZ, OLWX, OLWY, OLWZ, UBWX, UBWY, UBWZ, OBWX, OBWY, OBWZ, UHWX, UHWY, UHWZ, BRKX, BRKY, BRKZ, OHWX, OHWY, OHWZ, KOX, KOY, KOZ, SEHX, SEHY, SEHZ, OSLX, OSLY, OSLZ, USLX, USLY, USLZ, FULX, FULY, FULZ, FBLX, FBLY, FBLZ, OSRX, OSRY, OSRZ, USRX, USRY, USRZ, FURX, FURY, FURZ, FBRX, FBRY, FBRZ, SBLX, SBLY, SBLZ, OALX, OALY, OALZ, UALX, UALY, UALZ, HALX, HALY, HALZ, FILX, FILY, FILZ, SBRX, SBRY, SBRZ, OARX, OARY, OARZ, UARX, UARY, UARZ, HARX, HARY, HARZ, FIRX, FIRY, FIRZ)"
				,null);		
  		
		if(c.get("TIME") == null)
		{
			drawBackground();
			displayMessage("could not find Action "+identifier+"");
			numberFrames = 1;
			return;
		}
		
  		
		Vector<Object> times = c.get("TIME");
		framesToTimes = new float[times.size()];
		for(int i=0;i<framesToTimes.length;i++) framesToTimes[i] = Float.parseFloat(times.get(i).toString());
			
		BodyPoseSequence skeleton = new BodyPoseSequence();
		
		float[][] poses = new float[times.size()][];
		for(int i=0;i<poses.length;i++) poses[i] = new float[84];
		String[] posIdents = "BECX, BECY, BECZ, ULWX, ULWY, ULWZ, OLWX, OLWY, OLWZ, UBWX, UBWY, UBWZ, OBWX, OBWY, OBWZ, UHWX, UHWY, UHWZ, BRKX, BRKY, BRKZ, OHWX, OHWY, OHWZ, KOX, KOY, KOZ, SEHX, SEHY, SEHZ, OSLX, OSLY, OSLZ, USLX, USLY, USLZ, FULX, FULY, FULZ, FBLX, FBLY, FBLZ, OSRX, OSRY, OSRZ, USRX, USRY, USRZ, FURX, FURY, FURZ, FBRX, FBRY, FBRZ, SBLX, SBLY, SBLZ, OALX, OALY, OALZ, UALX, UALY, UALZ, HALX, HALY, HALZ, FILX, FILY, FILZ, SBRX, SBRY, SBRZ, OARX, OARY, OARZ, UARX, UARY, UARZ, HARX, HARY, HARZ, FIRX, FIRY, FIRZ".split(", ");
		
		for(int j=0;j<84;j++) {
	  		Vector<Object> d = c.get(posIdents[j]);
	  		for(int i=0;i<d.size() && i<poses.length;i++) {
	  			poses[i][j] = Float.parseFloat(d.get(i).toString());
	  		}
		}
		
		for(int i=0;i<poses.length;i++) 
			skeleton.addPose(0, Float.parseFloat(times.get(i).toString()), "", poses[i]);
		
		allItems.add(skeleton);
		animatedItemsRef.put("skeleton", skeleton);
  		
  		startTime = Double.parseDouble(times.get(0).toString());
  		endTime = Double.parseDouble(times.get(times.size()-1).toString());
  		numberFrames = times.size();
  		
  		System.out.print("ALL DONE");
  	}
  	
  	/**
  	 * Highlights one object, only if it is present in the scene (does not add anything)
  	 * @param identifier e.g.: "'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#drawer3'" (WITH single quotes!)
  	 */  	
  	public void highlightItem(String identifier, boolean highlight, int color) {
  		ItemBase itm = animatedItemsRef.get(identifier);
  		if(itm == null) {
  			//displayMessage("item "+identifier+" not in scene");
  			return;
  		}
  		
  		if(highlight)
	  		itm.setColor(color);
  		else
  			itm.setColor(itm.defaultColor);
  	}
  	
  	public void highlightItem(String identifier, boolean highlight) {
  		highlightItem(identifier, highlight, convertColor(255, 0, 0, 255));
  	}

  	public void clearHighlights() {
		// draw all Items
  		for(String j : animatedItemsRef.keySet()) {
  			animatedItemsRef.get(j).setColor(animatedItemsRef.get(j).defaultColor);
  		}
  	}
  	
  	public void highlightReachable(String identifier, boolean highlight) {
  		highlightItem(identifier, highlight);
  		
		HashMap<String, Vector<Object>> others = PrologVisualizationCanvas.executeQuery(
					"rdf_reachable("+identifier+", knowrob:properPhysicalPartTypes, I)", null);
		
		Vector<Object> itms = others.get("I");
		if(itms == null) return;
		
		for(Object o : itms)
			highlightItem(o.toString(), highlight);

  	}
  	
  	public void removeObject(String identifier) {
  		ItemBase item = animatedItemsRef.get(identifier);
  		if(item == null) {
  			//displayMessage("item "+identifier+" not in scene");
  			return;
  		}
  		
  		allItems.remove(item);
  		animatedItemsRef.remove(identifier);

  	}
  	
  	public void addObject(String identifier) {
  		ItemBase item = getItem(identifier);
  		if(item != null) {
  			removeObject(identifier);  			
  			allItems.add(item);
  			animatedItemsRef.put(identifier, item);
//  			displayMessage("added Item: "+identifier);
  		}
  	}
  	
  	public void addObjectWithChildren(String identifier) {
  		HashMap<String, Vector<Object>> physicalParts = PrologVisualizationCanvas.executeQuery(
  				"rdf_reachable("+identifier+", knowrob:properPhysicalPartTypes, PART)",null);
  	
     	if(physicalParts.get("PART") != null)
  			for(int i=0;i<physicalParts.get("PART").size();i++)
  				if(!physicalParts.get("PART").get(i).toString().equals(identifier))
  					addObject(physicalParts.get("PART").get(i).toString());
   				
     	
     	
  		HashMap<String, Vector<Object>> mapParts = PrologVisualizationCanvas.executeQuery(
  				"rdf_reachable(PART, knowrob:describedInMap, "+identifier+")",null);
  		

  		if(mapParts.get("PART") != null)
  			for(int i=0;i<mapParts.get("PART").size();i++)
  				if(!mapParts.get("PART").get(i).toString().equals(identifier))
	  			{
	   				HashMap<String, Vector<Object>> parts = PrologVisualizationCanvas.executeQuery(
	   						"rdf_reachable("+mapParts.get("PART").get(i).toString()+", knowrob:properPhysicalPartTypes, P)"
	   						,null);
	  				 				
	  				Vector<Object> p = parts.get("P");
	
	  				if(p != null)
	  		  			for(int j=0;j<p.size();j++)
	  		  				if(!p.get(j).toString().equals(mapParts.get("PART").get(i).toString()))
	  		  					addObject(p.get(j).toString());
	  				
	  				addObject(mapParts.get("PART").get(i).toString());	
	  			}
  		addObject(identifier);

  	}
  	
  	public void removeObjectWithChildren(String identifier) {
  		HashMap<String, Vector<Object>> physicalParts = PrologVisualizationCanvas.executeQuery(
  				"rdf_reachable("+identifier+", knowrob:properPhysicalPartTypes, PART)",null);
  	
     	if(physicalParts.get("PART") != null)
  			for(int i=0;i<physicalParts.get("PART").size();i++)
  				if(!physicalParts.get("PART").get(i).toString().equals(identifier))
  					removeObject(physicalParts.get("PART").get(i).toString());
   				
     	
     	
  		HashMap<String, Vector<Object>> mapParts = PrologVisualizationCanvas.executeQuery(
  				"rdf_reachable(PART, knowrob:describedInMap, "+identifier+")",null);
  		

  		if(mapParts.get("PART") != null)
  			for(int i=0;i<mapParts.get("PART").size();i++)
  				if(!mapParts.get("PART").get(i).toString().equals(identifier))
	  			{
	   				HashMap<String, Vector<Object>> parts = PrologVisualizationCanvas.executeQuery(
	   						"rdf_reachable("+mapParts.get("PART").get(i).toString()+", knowrob:properPhysicalPartTypes, P)"
	   						,null);
	  				 				
	  				Vector<Object> p = parts.get("P");
	
	  				if(p != null)
	  		  			for(int j=0;j<p.size();j++)
	  		  				if(!p.get(j).toString().equals(mapParts.get("PART").get(i).toString()))
	  		  					addObject(p.get(j).toString());
	  				
	  				removeObject(mapParts.get("PART").get(i).toString());	
	  			}
  		removeObject(identifier);

  	}
  	
  	public void reFetchInfo() {
  		String[] items = new String[allItems.size()];
  		for(int i=0;i<items.length;i++) {
  			items[i] = allItems.get(i).name;
  		}
  		clear();
  		for(int i=0;i<items.length;i++) {
  			addObject(items[i]);
  		}
  	}
  	
    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    // 
    // INTERNAL HELPERS
    // 
  	
  	
  	private float[] getOrientationOfItem(String identifier) {
  		try{
	  		// get orientation
	  		HashMap<String, Vector<Object>> nfo = PrologVisualizationCanvas.executeQuery(
	  				"rdf_triple(knowrob:orientation,"+identifier+",Or), " +
	  				
	  				"rdf_triple(knowrob:m00,Or,_M00l), actionmodel:am_strip_literal_type(_M00l, _M00), term_to_atom(M00,_M00)," +
	  				"rdf_triple(knowrob:m01,Or,_M01l), actionmodel:am_strip_literal_type(_M01l, _M01), term_to_atom(M01,_M01)," +
	  				"rdf_triple(knowrob:m02,Or,_M02l), actionmodel:am_strip_literal_type(_M02l, _M02), term_to_atom(M02,_M02)," +
	  				"rdf_triple(knowrob:m03,Or,_M03l), actionmodel:am_strip_literal_type(_M03l, _M03), term_to_atom(M03,_M03)," +
	  				
	  				"rdf_triple(knowrob:m10,Or,_M10l), actionmodel:am_strip_literal_type(_M10l, _M10), term_to_atom(M10,_M10)," +
	  				"rdf_triple(knowrob:m11,Or,_M11l), actionmodel:am_strip_literal_type(_M11l, _M11), term_to_atom(M11,_M11)," +
	  				"rdf_triple(knowrob:m12,Or,_M12l), actionmodel:am_strip_literal_type(_M12l, _M12), term_to_atom(M12,_M12)," +
	  				"rdf_triple(knowrob:m13,Or,_M13l), actionmodel:am_strip_literal_type(_M13l, _M13), term_to_atom(M13,_M13)," +
	  				
	  				"rdf_triple(knowrob:m20,Or,_M20l), actionmodel:am_strip_literal_type(_M20l, _M20), term_to_atom(M20,_M20)," +
	  				"rdf_triple(knowrob:m21,Or,_M21l), actionmodel:am_strip_literal_type(_M21l, _M21), term_to_atom(M21,_M21)," +
	  				"rdf_triple(knowrob:m22,Or,_M22l), actionmodel:am_strip_literal_type(_M22l, _M22), term_to_atom(M22,_M22)," +
	  				"rdf_triple(knowrob:m23,Or,_M23l), actionmodel:am_strip_literal_type(_M23l, _M23), term_to_atom(M23,_M23)," +
	  				
	  				"rdf_triple(knowrob:m30,Or,_M30l), actionmodel:am_strip_literal_type(_M30l, _M30), term_to_atom(M30,_M30)," +
	  				"rdf_triple(knowrob:m31,Or,_M31l), actionmodel:am_strip_literal_type(_M31l, _M31), term_to_atom(M31,_M31)," +
	  				"rdf_triple(knowrob:m32,Or,_M32l), actionmodel:am_strip_literal_type(_M32l, _M32), term_to_atom(M32,_M32)," +
	  				"rdf_triple(knowrob:m33,Or,_M33l), actionmodel:am_strip_literal_type(_M33l, _M33), term_to_atom(M33,_M33)", null);
	  		return new float[] {
	  				Float.parseFloat(nfo.get("M00").get(0).toString()),
	  				Float.parseFloat(nfo.get("M01").get(0).toString()),
	  				Float.parseFloat(nfo.get("M02").get(0).toString()),
	  				100*Float.parseFloat(nfo.get("M03").get(0).toString()),
	  				Float.parseFloat(nfo.get("M10").get(0).toString()),
	  				Float.parseFloat(nfo.get("M11").get(0).toString()),
	  				Float.parseFloat(nfo.get("M12").get(0).toString()),
	  				100*Float.parseFloat(nfo.get("M13").get(0).toString()),
	  				Float.parseFloat(nfo.get("M20").get(0).toString()),
	  				Float.parseFloat(nfo.get("M21").get(0).toString()),
	  				Float.parseFloat(nfo.get("M22").get(0).toString()),
	  				100*Float.parseFloat(nfo.get("M23").get(0).toString()),
	  				Float.parseFloat(nfo.get("M30").get(0).toString()),
	  				Float.parseFloat(nfo.get("M31").get(0).toString()),
	  				Float.parseFloat(nfo.get("M32").get(0).toString()),
	  				Float.parseFloat(nfo.get("M33").get(0).toString())};
  		} catch(Exception e) {
  			return null;
  		}
  	}
  	
  	

  	private ItemBase getItem(String identifier) {
  		
  		// get type
  		HashMap<String, Vector<Object>> tpe = PrologVisualizationCanvas.executeQuery(
  				"rdf_has("+identifier+", rdf:type, OBJECTCLASS)",null);
  		String type = null;
  		

  		
  		
  		// check if exists
  		try{
  			type = tpe.get("OBJECTCLASS").get(0).toString();
  		} catch(Exception e) {
  			displayMessage("item "+identifier+" not found");
  			return null;
  		}
  		
  		// ignore list (not displayable):
  		if(type.equals("'http://ias.cs.tum.edu/kb/knowrob.owl#SemanticEnvironmentMap'")
  				|| type.equals("'http://ias.cs.tum.edu/kb/knowrob.owl#HingedJoint'")
  				|| type.equals("'http://ias.cs.tum.edu/kb/knowrob.owl#Door'")
  				|| type.equals("'http://ias.cs.tum.edu/kb/knowrob.owl#WallOfAConstruction'")
  				|| type.equals("'http://ias.cs.tum.edu/kb/knowrob.owl#JAN'")
  				|| type.equals("'http://ias.cs.tum.edu/kb/knowrob.owl#JAN'")
  				) {
  			return null;
  			
  		}
  			
  		// simple types
  		ItemBase it =itemForObjType(type);
  		
  		
  		if(it!=null) {
  			float[] o = getOrientationOfItem(identifier);
  			
  			if(o!=null) {
  				it.setTrafoMatrix(o);
  			} else {System.out.println("ORIENTATION NULL");}
  			it.name = identifier;
  			return it;
  		}
  		
  		
  		// check if it is a storage facility, if it is: create; WITHOUT handles
  		HashMap<String, Vector<Object>> storage = PrologVisualizationCanvas.executeQuery(
  				"rdf_has("+identifier+", rdf:type, OBJECTCLASS)," +
  				"rdf_reachable(OBJECTCLASS, rdfs:subClassOf, knowrob:'StorageConstruct')," +
  				"rdf_has("+identifier+",knowrob:widthOfObject,literal(type(_,_W)))," + 
  				"rdf_has("+identifier+",knowrob:heightOfObject,literal(type(_,_H))), " + 
  				"rdf_has("+identifier+",knowrob:depthOfObject,literal(type(_,_D))), " +
  				
  				"rdf_triple(knowrob:orientation,"+identifier+",Or), " +
  				
  				"rdf_triple(knowrob:m00,Or,literal(type(_,_M00))), term_to_atom(M00,_M00)," +
  				"rdf_triple(knowrob:m01,Or,literal(type(_,_M01))), term_to_atom(M01,_M01)," +
  				"rdf_triple(knowrob:m02,Or,literal(type(_,_M02))), term_to_atom(M02,_M02)," +
  				"rdf_triple(knowrob:m03,Or,literal(type(_,_M03))), term_to_atom(M03,_M03)," +
  				
  				"rdf_triple(knowrob:m10,Or,literal(type(_,_M10))), term_to_atom(M10,_M10)," +
  				"rdf_triple(knowrob:m11,Or,literal(type(_,_M11))), term_to_atom(M11,_M11)," +
  				"rdf_triple(knowrob:m12,Or,literal(type(_,_M12))), term_to_atom(M12,_M12)," +
  				"rdf_triple(knowrob:m13,Or,literal(type(_,_M13))), term_to_atom(M13,_M13)," +
  				
  				"rdf_triple(knowrob:m20,Or,literal(type(_,_M20))), term_to_atom(M20,_M20)," +
  				"rdf_triple(knowrob:m21,Or,literal(type(_,_M21))), term_to_atom(M21,_M21)," +
  				"rdf_triple(knowrob:m22,Or,literal(type(_,_M22))), term_to_atom(M22,_M22)," +
  				"rdf_triple(knowrob:m23,Or,literal(type(_,_M23))), term_to_atom(M23,_M23)," +
  				
  				"rdf_triple(knowrob:m30,Or,literal(type(_,_M30))), term_to_atom(M30,_M30)," +
  				"rdf_triple(knowrob:m31,Or,literal(type(_,_M31))), term_to_atom(M31,_M31)," +
  				"rdf_triple(knowrob:m32,Or,literal(type(_,_M32))), term_to_atom(M32,_M32)," +
  				"rdf_triple(knowrob:m33,Or,literal(type(_,_M33))), term_to_atom(M33,_M33)," +
  				 
  				"atom_to_term(_W,W,_), atom_to_term(_H,H,_), atom_to_term(_D,D,_)", null);
  		if( storage.get("W") != null && storage.get("W").size() > 0) {
  			StorageFacility c = null;
			if(storage.get("OBJECTCLASS").get(0).toString().endsWith("Cupboard'")) {
				c = new Cupboard(
					Float.valueOf(storage.get("M00").get(0).toString()),
					Float.valueOf(storage.get("M01").get(0).toString()),
					Float.valueOf(storage.get("M02").get(0).toString()),
					100*Float.valueOf(storage.get("M03").get(0).toString()),

					Float.valueOf(storage.get("M10").get(0).toString()),
					Float.valueOf(storage.get("M11").get(0).toString()),
					Float.valueOf(storage.get("M12").get(0).toString()),
					100*Float.valueOf(storage.get("M13").get(0).toString()),

					Float.valueOf(storage.get("M20").get(0).toString()),
					Float.valueOf(storage.get("M21").get(0).toString()),
					Float.valueOf(storage.get("M22").get(0).toString()),
					100*Float.valueOf(storage.get("M23").get(0).toString()),

					Float.valueOf(storage.get("M30").get(0).toString()),
					Float.valueOf(storage.get("M31").get(0).toString()),
					Float.valueOf(storage.get("M32").get(0).toString()),
					Float.valueOf(storage.get("M33").get(0).toString()),

					100*Float.valueOf(storage.get("D").get(0).toString()),
					100*Float.valueOf(storage.get("W").get(0).toString()),
					100*Float.valueOf(storage.get("H").get(0).toString()));
			} else {
				c = new Drawer(
					Float.valueOf(storage.get("M00").get(0).toString()),
					Float.valueOf(storage.get("M01").get(0).toString()),
					Float.valueOf(storage.get("M02").get(0).toString()),
					100*Float.valueOf(storage.get("M03").get(0).toString()),

					Float.valueOf(storage.get("M10").get(0).toString()),
					Float.valueOf(storage.get("M11").get(0).toString()),
					Float.valueOf(storage.get("M12").get(0).toString()),
					100*Float.valueOf(storage.get("M13").get(0).toString()),

					Float.valueOf(storage.get("M20").get(0).toString()),
					Float.valueOf(storage.get("M21").get(0).toString()),
					Float.valueOf(storage.get("M22").get(0).toString()),
					100*Float.valueOf(storage.get("M23").get(0).toString()),

					Float.valueOf(storage.get("M30").get(0).toString()),
					Float.valueOf(storage.get("M31").get(0).toString()),
					Float.valueOf(storage.get("M32").get(0).toString()),
					Float.valueOf(storage.get("M33").get(0).toString()),

					100*Float.valueOf(storage.get("D").get(0).toString()),
					100*Float.valueOf(storage.get("W").get(0).toString()),
					100*Float.valueOf(storage.get("H").get(0).toString()));
  				}
			int col = grayValues[(++grayLevelCounter) % grayValues.length];
			
			c.defaultColor = convertColor(col, col, col, 255);
			c.setColor(c.defaultColor);
  			c.name = identifier;
  			
			return c;
  		}
	
  		
  		
  		// check if Handle
		HashMap<String, Vector<Object>> handles = PrologVisualizationCanvas.executeQuery(
					"rdf_has("+identifier+", rdf:type, knowrob:'Handle'), " +
					"rdf_has("+identifier+",knowrob:widthOfObject,literal(type(_,W))), " + 
					"rdf_has("+identifier+",knowrob:heightOfObject,literal(type(_,H))), " + 
					"rdf_has("+identifier+",knowrob:depthOfObject,literal(type(_,D))), " + 
					"rdf_triple(knowrob:orientation,"+identifier+",Or), " +
					"rdf_triple(knowrob:m03, Or, literal(type(_,POS_X)))," +  
					"rdf_triple(knowrob:m13, Or, literal(type(_,POS_Y))), " + 
					"rdf_triple(knowrob:m23, Or, literal(type(_,POS_Z))), " + 
					"atom_to_term(W,NUM_WIDTH,_), atom_to_term(H,NUM_HEIGHT,_)," +  
					"atom_to_term(D,NUM_DEPTH,_), atom_to_term(POS_X,NUM_POS_X,_), " + 
					"atom_to_term(POS_Y,NUM_POS_Y,_), " + 
					"atom_to_term(POS_Z,NUM_POS_Z,_)", null);
		if (handles.get("NUM_WIDTH") != null && handles.get("NUM_WIDTH").size() > 0)
		{
			ItemBase item =  new BoxHandle(
					100*Float.valueOf(handles.get("NUM_POS_X").get(0).toString()),
					100*Float.valueOf(handles.get("NUM_POS_Y").get(0).toString()),
					100*Float.valueOf(handles.get("NUM_POS_Z").get(0).toString()),
					100*Float.valueOf(handles.get("NUM_WIDTH").get(0).toString()),
					100*Float.valueOf(handles.get("NUM_HEIGHT").get(0).toString()),
					100*Float.valueOf(handles.get("NUM_DEPTH").get(0).toString())
					);
  			item.name = identifier;
  			return item;
		}

		// check if knob
		HashMap<String, Vector<Object>> knobs = PrologVisualizationCanvas.executeQuery(
				"rdf_has("+identifier+", rdf:type, knowrob:'ControlKnob'), " +
				"rdf_has("+identifier+",knowrob:radius,literal(type(_,R))), " + 
				"rdf_triple(knowrob:orientation,"+identifier+",Or), " +
				"rdf_triple(knowrob:m03, Or, literal(type(_,POS_X)))," +  
				"rdf_triple(knowrob:m13, Or, literal(type(_,POS_Y))), " + 
				"rdf_triple(knowrob:m23, Or, literal(type(_,POS_Z))), " + 
				"atom_to_term(R,RADIUS,_), atom_to_term(POS_X,NUM_POS_X,_), " + 
				"atom_to_term(POS_Y,NUM_POS_Y,_), " + 
				"atom_to_term(POS_Z,NUM_POS_Z,_)", null);
		if (knobs.get("RADIUS") != null && knobs.get("RADIUS").size() > 0)
		{
			ItemBase item = new SphereHandle(
					100*Float.valueOf(knobs.get("NUM_POS_X").get(0).toString()),
					100*Float.valueOf(knobs.get("NUM_POS_Y").get(0).toString()),
					100*Float.valueOf(knobs.get("NUM_POS_Z").get(0).toString()),
					100*Float.valueOf(knobs.get("RADIUS").get(0).toString())
					);

  			item.name = identifier;
  			return item;
		}


  		displayMessage("could not find how to add Item "+identifier+"\nType was: "+type);
  		return null;

  	}
  	

  
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////
  // 
  // EVENT HANDLER
  // 
  
  public void mousePressed(MouseEvent e) {
	  
	  // general: save mouse positions for calculating rotation and translation
	  if(e.getButton()==MouseEvent.BUTTON1) {
			leftMouseX = e.getX();
			leftMouseY = e.getY();
	  } else if(e.getButton()==MouseEvent.BUTTON3) {
			rightMouseX = e.getX();
			rightMouseY = e.getY();
	  } else if (e.getButton()==MouseEvent.BUTTON2) {
			centerMouseY = e.getY();
	  }
	  


//	  // ------------- menu stuff -------------- 
//	  // play forward
//	  if(menu_human.mouseInTriangle(e.getX(), e.getY(), menu_human.xpl1, menu_human.ypl1, menu_human.xpl2, menu_human.ypl2, menu_human.xpl3, menu_human.ypl3)){
//		  playingForward = true;
//		  timer.start();		 	  
//	  }
//	  
//	  // pause
//	  else if(menu_human.mouseInRectangle(e.getX(), e.getY(), menu_human.xps, menu_human.yps, menu_human.widthps, menu_human.heightps)){
//		  timer.stop();
//	  }
//	  
//	  // play backward
//	  else if(menu_human.mouseInTriangle(e.getX(), e.getY(), menu_human.xrew1, menu_human.yrew1, menu_human.xrew2, menu_human.yrew2, menu_human.xrew3, menu_human.yrew3)){		  
//		  playingForward = false;
//		  timer.start();	  
//	  }
//	  
//	  // stop button
//	  else if(menu_human.mouseInRectangle(e.getX(), e.getY(), menu_human.xstop, menu_human.ystop, menu_human.widthstop, menu_human.heightstop)){
//		  timer.stop();
//		  currentFrame = 0;
//	  }
//	  
//	  // step forward
//	  else if(menu_human.mouseInRectangle(e.getX(), e.getY(), menu_human.xpls, menu_human.ypls, menu_human.widthpls, menu_human.heightpls)){
//		  timer.stop();
//		  currentFrame++;
//	  }
//	  
//	  // step backward
//	  else if(menu_human.mouseInRectangle(e.getX(), e.getY(), menu_human.xrews, menu_human.yrews, menu_human.widthrews, menu_human.heightrews)){
//		  timer.stop();
//		  currentFrame--;
//	  } 
	  
	  

	  
	  
	  if(currentFrame < 0) currentFrame = 0;
	  if(currentFrame>= numberFrames) currentFrame = numberFrames-1;
	  
	  
	  
	  
	  
	  
	  
	  redraw();
  }
  
  public void mouseClicked(MouseEvent e) {
	  String clickedOn = getItemAt(e.getX(), e.getY());
	  displayMessage(clickedOn+"");
	  if(clickedOn != null) {
		  //if(currentlyActiveItem != null) highlightItem(currentlyActiveItem, false);
		  currentlyActiveItem = clickedOn;
		  //highlightItem(clickedOn, true);
		  prologVisCanvas.displayInfoFor(clickedOn);
	  }
	  delay(100);
	  redraw();
  }
  
  
  
  
	public void mouseReleased(MouseEvent e) {
		
		if(e.getButton()==MouseEvent.BUTTON1) {	// reset the buffers
			leftMouseX = -1.0f;
			leftMouseY = -1.0f;
		} else if(e.getButton()==MouseEvent.BUTTON3) {	// reset the buffers
			rightMouseX = -1.0f;
			rightMouseY = -1.0f;
		} else if (e.getButton()==MouseEvent.BUTTON2) {
			centerMouseY = -1.0f;
		}
		redraw();
	}

    public void mouseDragged(MouseEvent e) {


    	if(leftMouseX != -1.0f) {	// change rotation
    		  yRotDisplay-= (e.getY()-leftMouseY) * 0.05;
    		  xRotDisplay+= (e.getX()-leftMouseX) * 0.05;
  			leftMouseX = e.getX();
			leftMouseY = e.getY();
    		
		}else if(rightMouseX != -1.0f) {	// change translation
  		  yShiftDisplay+= (e.getY()-rightMouseY) * 0.5;
  		  xShiftDisplay+= (e.getX()-rightMouseX) * 0.5;
			rightMouseX = e.getX();
			rightMouseY = e.getY();
  		
		} else if (centerMouseY != -1.0f) {	// zoom
			  zoomDisplay+= (e.getY()-centerMouseY) * 0.02;
			  if(zoomDisplay<0.01){zoomDisplay=0.01f;}
			  centerMouseY = e.getY();
		}
    	
    	redraw();
    }
    

    public void keyPressed(){
    	// hook: when running as slave, just propagate key events for proper synchronization
  	    //if(prologVisCanvas != null) {
  	    //	prologVisCanvas.keyPressed(keyCode);
  	    //	return;
  	    //}
		switch(keyCode) {
		case RIGHT:
			currentFrame++;
			break;
		case LEFT:
			currentFrame--;
			break;
		case java.awt.event.KeyEvent.VK_PAGE_UP:
			currentFrame+=10;
			break;
		case java.awt.event.KeyEvent.VK_PAGE_DOWN:
			currentFrame-=10;
			timer.stop();
			break;
		case java.awt.event.KeyEvent.VK_SPACE:
			if(timer.isRunning())
				timer.stop();
			else
				timer.start();
			break;
		case KeyEvent.VK_C:
			clear();		
			break;
		case KeyEvent.VK_B:
			drawBackground();
			break;	
		case KeyEvent.VK_R:
			reFetchInfo();
			break;
			
		// debug stuff
//		case KeyEvent.VK_H:
//			highlightItem("'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#drawer6'", true);
//			break;
//		case KeyEvent.VK_U:
//			clearHighlights();
//			break;
//		case KeyEvent.VK_D:
//			displayActionFixedIdent("'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2'");
//			break;
//		case KeyEvent.VK_V:
//			highlightReachable("'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#drawer6'", true);
//			break;
//		case KeyEvent.VK_I:
//			addObject("'http://ias.cs.tum.edu/kb/map_addons.owl#table0'");
//			break;	
//		case KeyEvent.VK_M:
//			removeObject("'http://ias.cs.tum.edu/kb/ias_entities.owl#fork1'");
//			break;	
//		case KeyEvent.VK_T:
//			
//			displayMessage("JAN");
//			addObjectWithChildren("'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls'");
//			//addObjectWithChildren("'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#cupboard3'");
//			
//			break;
		case KeyEvent.VK_X:
			bufferFrame.setVisible(!bufferFrame.isVisible());
			break;
		case KeyEvent.VK_Y:
			buffer.resetMatrix();
			this.resetMatrix();
			
			break;
		}
		if(currentFrame < 0) currentFrame = 0;
		if(currentFrame>= numberFrames) currentFrame = numberFrames-1;
		delay(100);
    	redraw();
	} 
    
    
    
    // ////////////////////////////////////////////////////////////////////////////////////////////
    // ////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    // 
    // HELPER FUNCTIONS
    // 
    
    public void setViewParameters(float xShift, float yShift, float xRot, float yRot, float zoom) {
    	this.xShiftDisplay = xShift;
    	this.yShiftDisplay = yShift;
    	this.xRotDisplay = xRot;
    	this.yRotDisplay = yRot;
    	this.zoomDisplay   = zoom;
    } 

    // sort the array jointTrajectoriesData by time
    public ArrayList<Point[]> sortArray(ArrayList<Point[]> actionsArray){
  	  	 	 
  	  for(int i = 0; i < actionsArray.size()-1; i++){
  		  for(int j = i+1; j < actionsArray.size(); j++){
  			  if(actionsArray.get(i)[0].time > actionsArray.get(j)[0].time){
  				  Point aux = new Point();
  				  for (int l = 0; l < 28; l++){
  					  aux = actionsArray.get(j)[l];
  					  actionsArray.get(j)[l] = actionsArray.get(i)[l];
  					  actionsArray.get(i)[l] = aux;
  				  }
  			  }
  		  }
  	  }
  	  return actionsArray;
    }
    
    
    public int[] makeColor(int r,int g,int b){
  	  int[] color = new int[3];
  	  color[0] = r; color[1] = g; color[2] = b;
  	  return color;
    }
    
    
    //fills the array colors with two colors for each action, one for the trajectory and one for the human pose
    public void setColors(){
  	  
  	  colors.add(makeColor(0,0,255));	  	  	  
  	  colors.add(makeColor(30,144,255));
  	  
  	  colors.add(makeColor(0,255,255));
  	  colors.add(makeColor(180,255,255));
  	  
  	  colors.add(makeColor(255,215,0));
  	  colors.add(makeColor(238,221,130));
  	  
  	  colors.add(makeColor(250,128,114));
  	  colors.add(makeColor(255,160,122));

  	  colors.add(makeColor(255,105,180));
  	  colors.add(makeColor(255,228,225));
  	  
  	  colors.add(makeColor(186,85,211));
  	  colors.add(makeColor(221,160,221));	  
    }       

    
    public StandaloneKitchenVisApplet() {

    }
    
    final Timer timer = new Timer(40, new ActionListener() {
    	  public void actionPerformed(ActionEvent event) {
    		int frameBefore = currentFrame;
    		if(playingForward == true) currentFrame++;
    		if(playingForward == false) currentFrame--;
			if(currentFrame < 0) currentFrame = 0;
			if(currentFrame>= numberFrames) currentFrame = numberFrames-1;   
			if(currentFrame != frameBefore) redraw();
    	  }
    	});
    
    // menu for the visualization jointTrajectories
    public class MenuHuman{
  		
  	  	// tracking button
  	  	int xtr = 10;		int ytr = 10;
  	  	int widthtr = 70;  	int heighttr = 20;
  	  	int xtxt = 15;	  	int ytxt = 15;
  	  	int widthtxt = 60; 	int heighttxt = 10;
  		
  	  	// rewind button
  	  	int xrew1 = 90;		int xrew2 = 115;	int xrew3 = 115;
  	  	int yrew1 = 20; 	int yrew2 = 10; 	int yrew3 = 30;
  	  	
  	  	// pause button
  	  	int xps = 125;		int xps1 = 128;		int xps2 = 137;
  	  	int yps = 10;		int yps1 = 10;		int yps2 = 10;
  	  	int widthps = 20;	int widthps1 = 5;	int widthps2 = 5;
  	  	int heightps = 20;	int heightps1 = 20;	int heightps2 = 20;
  	  	
  	  	
  	  	// play button
  	  	int xpl1 = 155;		int xpl2 = 155;		int xpl3 = 180;
  		int ypl1 = 10;		int ypl2 = 30;		int ypl3 = 20;
  		
  		// stop button
  		int xstop = 190;	int ystop = 10;
  		int widthstop = 20;	int heightstop = 20;	
  		
  		// rewind stepwise
  		int xrews = 220;	int yrews = 10;
  		int widthrews = 25;	int heightrews = 20;
  		
  		int xrews1 = 220;	int xrews2 = 235;	int xrews3 = 235;	int xrews4 = 239;
  		int yrews1 = 20;	int yrews2 = 10;	int yrews3 = 30;	int yrews4 = 10;
  		int widthrews4 = 5;	int heightrews4 = 20;
  		
  		// play stepwise
  		int widthpls = 25;	int heightpls = 20;	int widthpls4 = 5;	int heightpls4 = 20;
  		
  		int xpls = 255;		int xpls1 = 264;	int xpls2 = 264;	int xpls3 = 279;	int xpls4 = 255;
  		int ypls = 10;		int ypls1 = 10;		int ypls2 = 30;		int ypls3 = 20;		int ypls4 = 10;
  				
  									
  		public void drawMenu(){						
  			noStroke();

  			fill(238,238,224);
  			rect(xtr, ytr, widthtr, heighttr);
  			fill(238,238,224);
  			triangle(xrew1, yrew1, xrew2, yrew2, xrew3, yrew3);			
  			rect(xps1, yps1, widthps1, heightps1);
  			rect(xps2, yps2, widthps2, heightps2);
  			triangle(xpl1, ypl1, xpl2, ypl2, xpl3, ypl3);
  			rect(xstop, ystop, widthstop, heightstop);			
  			triangle(xpls1, ypls1, xpls2, ypls2, xpls3, ypls3);
  			rect(xpls4, ypls4, widthpls4, heightpls4);			
  			triangle(xrews1, yrews1, xrews2, yrews2, xrews3, yrews3);
  			rect(xrews4, yrews4, widthrews4, heightrews4);
  			
  			
  			fill(0,0,0);stroke(0);
  			textFont(verdanaBold);
  			textMode(SCREEN);
  			textAlign(LEFT);
  			String s = "Trajectory";
  			text(s, xtxt, ytxt, widthtxt, heighttxt);
  		}
  				
  		public boolean mouseInTriangle(int xm, int ym, int x1, int y1, int x2, int y2, int x3, int y3){			 	      
  			  int fAB = (ym-y1)*(x2-x1) - (xm-x1)*(y2-y1);
  			  int fCA = (ym-y3)*(x1-x3) - (xm-x3)*(y1-y3);
  			  int fBC = (ym-y2)*(x3-x2) - (xm-x2)*(y3-y2);

  			  if ((fAB*fBC > 0) && (fBC*fCA>0))
  			         return true;
  			  return false;
  		  }
  		
  		public boolean mouseInRectangle(int xm, int ym, int xr, int yr, int width, int height){
  			if ((xm >= xr) && (xm <= xr+width) && (ym >= yr) && (ym <= yr+height)) {
  				    return true;
  				  } else {
  				    return false;
  				  }
  		}
  		
  		public int trackingDialog(){
  			  
  			String[] possibilities = {"BEC", "ULW", "OLW", "UBW", "OBW", "UHW", "BRK", "OHW", "KO", "SEH", "OSL", "USL", "FUL", "FBL", 
  					"OST", "USR", "FUR", "FBR", "SBL", "OAL", "UAL", "HAL", "FIL", "SBR", "OAR", "UAR", "HAR", "FIR"};
  			
  			
  			for(int i = 0; i < possibilities.length; i++){
  				for (int j = i+1; j < 28; j++){
  					if (possibilities[i].compareTo(possibilities[j]) > 0){
  						String aux = possibilities[i];
  						possibilities[i] = possibilities[j];
  						possibilities[j] = aux;
  					}
  				}
  			}
  			JOptionPane.showInputDialog(null, "Select the body part you want to track:\n","Customized Dialog",		                    
  			                    JOptionPane.PLAIN_MESSAGE, null, possibilities, "");
// TOFIX  			
//  			ArrayList<Point[]> jointPositions = bodyPoseSeq.getSequence();
//  			if ((s != null) && (s.length() > 0))
//  				for (int i = 0 ; i < 28; i++){
//  					if (jointPositions.get(0)[i].name == s)
//  						return i;
//  				}	
  			return -1;		
  		}

  	}  

	public static int convertColor(int red, int green, int blue, int alpha) {
		return (((((alpha << 8) + red) << 8) + green) << 8) + blue;
	}


  	ItemBase itemForObjType(String type) {
  		
  		
  		/////////////////////////////////////////////
  		// tableware
  		
  		if(type.endsWith("#Cup'")) {
  			return new Cup(0,0,0);

  		} else if(type.endsWith("#DinnerPlate'")) {
  			return new Plate(0,0,0);

  		} else if (type.endsWith("#DrinkingGlass'")) {
  			return new DrinkingGlass(0,0,0);

  		} else if (type.endsWith("#Bowl-Eating'")) {
  			return new BowlEating(0,0,0);
  	  		  			
  			
  			
  	  	/////////////////////////////////////////////
  		// silverware	
  			
  		} else if(type.endsWith("#DinnerFork'")) {
  			return new Fork(0,0,0);
  		
  		} else if(type.endsWith("#TableKnife'")) {
  			return new Knife(0,0,0);

  		} else if(type.endsWith("#SoupSpoon'")) {
  			return new Spoon(0,0,0);
  		
  			
  			
  		/////////////////////////////////////////////
  		// serving and cooking
  			
  		} else if(type.endsWith("#Napkin'")) {
  			return new Napkin(0,0,0);
  		
  		} else if(type.endsWith("#PlaceMat'")) {
  			return new PlaceMat(0,0,0);
  			
  		} else if (type.endsWith("#Platter'")) {
			return new Platter(0,0,0);  		
  			
  		} else if(type.endsWith("#CookingPot'")) {
  			return new CookingPot(0,0,0);

  			
  			
  		/////////////////////////////////////////////
  		// breakfast consumables	
  			
  		} else if (type.endsWith("#Bread'")) {
  	  		return new Bread(0,0,0);

  		} else if (type.endsWith("#Cheese'")) {
  			return new Cheese(0,0,0);

  		} else if (type.endsWith("#Sausage'")) {
  	  		return new Sausage(0,0,0);

  		} else if (type.endsWith("#Cake'")) {
  			return new Cake(0,0,0);
  			
  		} else if (type.endsWith("#BreakfastCereal'")) {
  			return new CerealBox(0,0,0);
  			
  			
  			
  		/////////////////////////////////////////////
  		// lunch/dinner consumables
  			
  		} else if (type.endsWith("#Pizza'")) {
  			return new Pizza(0,0,0);

  		} else if (type.endsWith("#Salad'")) {
  			return new SaladBowl(0,0,0);

  		} else if (type.endsWith("#Pasta'")) {
  			return new SaladBowl(0,0,0);

  		} else if (type.endsWith("#Soup'")) {
  			return new SoupPlate(0,0,0);
  			
  			
  			
  		/////////////////////////////////////////////
  		// drinks
  			
  		} else if (type.endsWith("#Water'")) {
  			return new Bottle(0,0,0);

  		} else if (type.endsWith("#Pitcher'")) {
  			return new Thermos(0,0,0);
  			
  		} else if (type.endsWith("#Tea-Beverage'")) {
  			return new Bread(0,0,0);

  		} else if (type.endsWith("#Coffee-Beverage'")) {
  			return new Thermos(0,0,0);

  		} else if (type.endsWith("#Juice'")) {
  			return new Tetrapak(0,0,0);
  			
  		} else if (type.endsWith("#Tea-Iced'")) {
  			return new Tetrapak(0,0,0);

  		} else if(type.endsWith("#Tetrapak'")) {
  			return new Tetrapak(0,0,0);

  		} else if (type.endsWith("#CardboardBox'")) {
  			return new Bread(0,0,0);


  			
  		/////////////////////////////////////////////
  		// furniture
  			
  		} else if(type.endsWith("#KitchenTable'")) {
  			return new Table(0,0,0);
  	  		
  		} else if(type.endsWith("#Chair-PieceOfFurniture'")) {
  			return new Chair(0,0,0);
  		
			
  		/////////////////////////////////////////////
  		// dummies
  		
  		} else if(type.endsWith("#SpatialThing-Localized'")) {
			return new Ellipse(0f,0f,0f,10f, 10f, convertColor(255, 255, 0, 255));

  		} else if(type.endsWith("#Place'")) {
			return new Ellipse(0f,0f,0f,10f, 10f, convertColor(255, 255, 0, 255));
		}
		return null;	
  		//return new Ellipse(0f,0f,0f,10f, 10f, convertColor(255, 255, 255, 255));
  	}

    
  	
	
}


class EmptyCanvas extends Canvas {
	private int[] snapshot;
	public boolean drawSelf = false;
	private static final long serialVersionUID = 1L;
	public EmptyCanvas(int width, int height, String renderer) {
		super();
		this.width = width;
		this.height = height;		
		init();
		snapshot = new int[width*height];
		noLoop();
		try{super.size(width, height, P3D);}catch(RendererChangeException e) {};
		noLoop();
		//this.resize(width, height);
	}

	
	@Override
	public void draw() {
		if(drawSelf)
			for(int i=0;i<width;i++)
				for(int j=0;j<height;j++)
					set(i, j, snapshot[i*height+j]);
	};
	@Override
	public void keyPressed() {};
	@Override
	public void mousePressed(MouseEvent e) {};
	@Override
	public void mouseReleased(MouseEvent e) {};
	@Override
	public void mouseDragged(MouseEvent e) {};
	@Override
	public void zoom(int delta) {};
	@Override
	public void runMain() {};
	@Override
	public void size(int width, int height, String renderer) {};

	
	public void freeze() {
		for(int i=0;i<width;i++)
			for(int j=0;j<height;j++)
				snapshot[i*height+j] = get(i, j);
		drawSelf = true;
	};
	
	public void unFreeze() {
		drawSelf = false;
	}
}

