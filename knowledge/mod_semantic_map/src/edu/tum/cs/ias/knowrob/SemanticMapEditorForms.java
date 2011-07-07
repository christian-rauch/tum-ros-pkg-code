package edu.tum.cs.ias.knowrob;

import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.semanticweb.owlapi.model.*;

import controlP5.Button;
import controlP5.ControlEvent;
import controlP5.ControlP5;
import controlP5.ListBox;
import controlP5.ListBoxItem;
import controlP5.Textfield;
import controlP5.Textlabel;
import de.tum.in.fipm.kipm.gui.visualisation.applets.StandaloneKitchenVisApplet;
import de.tum.in.fipm.kipm.gui.visualisation.base.PrologVisualizationCanvas;
import de.tum.in.fipm.kipm.gui.visualisation.items.*;
import edu.tum.cs.ias.knowrob.utils.owl.MapObject;
import edu.tum.cs.ias.knowrob.utils.owl.OWLImportExport;
import edu.tum.cs.ias.knowrob.utils.owl.OWLFileUtils;
import processing.core.PApplet;
import ros.NodeHandle;
import ros.Ros;



/**
 * User interface for the semantic map editor
 * 
 * This class provides the input forms and interaction tools to enter the 
 * object information to the editor and to update the visualization. 
 * 
 * 
 * @author tenorth@cs.tum.edu
 *
 */
public class SemanticMapEditorForms extends PApplet {

	private static final long serialVersionUID = -4266603809510554926L;

	///////////////////////////////////////////////////////////////
	// visualization and user interface stuff
	
	private PrologVisualizationCanvas prologVisCanvas = null;
	public ControlP5 controlP5;
	boolean[] keys = new boolean[526];
	boolean selectParentObj = false; // switch mode from normal obj selection to parent obj selection
	
	private int[] grayValues = new int[] {160,190,210,170,220,180,200,165,185,205,175,195,215};
	private static int grayLevelCounter = 0;
	
	
	// form elements
	Textfield t_id;
	ListBox t_types;
	
	Textfield[] t_dim;
	Textfield[] t_matrix;
	Textfield[] t_quat;
	Textfield[] t_pos;
	Textfield[] t_qpos;
	Textfield t_filename;
	Textfield t_parent;
	ArrayList<Textfield> t_all_simple; // for using the tab key
	ArrayList<Textfield> t_all_matrix; // for using the tab key
	ArrayList<Textfield> t_all_quat; // for using the tab key
	
	public enum Tab {SIMPLE, MATRIX, QUATERNION};
	Tab tab;

	static Ros ros;
	static NodeHandle n;
	
	///////////////////////////////////////////////////////////////
	
	
	
	/**
	 * Internal storage of the MapObject instances
	 */
	HashMap<String, MapObject> objects;
	
	/**
	 * List of objects to be displayed in the object classes selector
	 */
	protected static final HashMap<Integer, String> objClasses = new HashMap<Integer, String>();
	static {
		objClasses.put(1022,   "Cupboard");
		objClasses.put(1023,   "Dishwasher");
		objClasses.put(1024,   "Drawer");
		objClasses.put(1025,   "Oven");
		objClasses.put(1026,   "Refrigerator");
		objClasses.put(1027,   "Door");
		objClasses.put(1028,   "Handle");
		objClasses.put(1029,   "ControlKnob");
		objClasses.put(1030,   "HingedJoint");
		objClasses.put(1031,   "PrismaticJoint");
		
	}
	
	
	/**
	 * Initialize the interface and datastructures 
	 */
	public void setup() {

		size(240, 600, P2D);
		lights();
		objects = new HashMap<String, MapObject>();
		initRos();
		tab=Tab.SIMPLE;

		initControlP5Forms();

		draw(); 
		prologVisCanvas.setSize(950, 600);
		prologVisCanvas.validate();
		
		// dialog for selecting the input file
	    String filename = selectInput();
	    if (filename != null) {
	    	this.objects = OWLImportExport.readMapObjectFromOWL(filename);			
	    }
	}

	
	/**
	 * Draw-method: call controlP5.draw to update the content of the forms.
	 */
	public void draw() {

		background(20, 20, 20);
		controlP5.draw();
		
	}
	
	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

		ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
			ros.init("knowrob_semantic_map_to_owl");
		}
		n = ros.createNodeHandle();

	}



	////////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	// INITIALIZE FORMS
	//
	//
	////////////////////////////////////////////////////////////////////////////////////////////	
	
	
	/**
	 * Initialize the tabs and form fields
	 */
	private void initControlP5Forms() {

		// location definitions:
		
		int OBJ_ID_X=10;
		int OBJ_ID_Y=25;
		
		int OBJ_DIM__X=10;
		int OBJ_DIM__Y=200;

		int TAB_X=10;
		int TAB_Y=280;
		
		int UPDATE_X = 10;
		int UPDATE_Y = 500;

		t_all_simple = new ArrayList<Textfield>();
		t_all_matrix = new ArrayList<Textfield>();
		t_all_quat = new ArrayList<Textfield>();
		
		
		
		controlP5 = new ControlP5(this);

		controlP5.tab("default").setId(0);
		controlP5.tab("default").setLabel("simple");
		controlP5.tab("default").activateEvent(true);
		
		controlP5.tab("matrix").setId(1);
		controlP5.tab("matrix").activateEvent(true);
		
		controlP5.tab("quaternion").setId(2);
		controlP5.tab("quaternion").activateEvent(true);

		// text field for object name
		t_id = controlP5.addTextfield("object id",       OBJ_ID_X, OBJ_ID_Y, 200, 20);
		t_id.moveTo("global");
		t_all_simple.add(t_id); t_all_matrix.add(t_id); t_all_quat.add(t_id);
		t_id.setAutoClear(false);
		

		Button new_obj = controlP5.addButton("new",    17, OBJ_ID_X,      OBJ_ID_Y+40, 24, 20);
		new_obj.moveTo("global");

		Button del_obj = controlP5.addButton("delete", 18, OBJ_ID_X + 30, OBJ_ID_Y+40, 34, 20);
		del_obj.moveTo("global");

		Button update = controlP5.addButton("update vis", 23, OBJ_ID_X+70, OBJ_ID_Y+40, 55, 20);
		update.setId(23);
		update.moveTo("global");

		
		t_types = controlP5.addListBox("object class",   OBJ_ID_X, OBJ_ID_Y+100,200,60);
		t_types.setId(42);
		t_types.setItemHeight(15);
		t_types.setBarHeight(15);
		t_types.captionLabel().style().marginTop = 3;
		t_types.valueLabel().style().marginTop = 3;
		t_types.captionLabel().style().marginBottom = 3;
		t_types.valueLabel().style().marginBottom = 3;
		t_types.setColorBackground(color(255,128));
		t_types.moveTo("global");
		
		ListBoxItem b;
		for(int i: objClasses.keySet()) {
			b = t_types.addItem(objClasses.get(i),i);
			b.setId(i);
		}
		
		// object dimensions
		Textlabel l_dim = controlP5.addTextlabel("l_dim","OBJECT DIMENSIONS",OBJ_DIM__X,OBJ_DIM__Y);
		l_dim.setWidth(200);
		l_dim.moveTo("global");
		
		t_dim = new Textfield[3];
		t_dim[0] = controlP5.addTextfield("x_dim",OBJ_DIM__X,     OBJ_DIM__Y+15, 40,20);
		t_dim[1] = controlP5.addTextfield("y_dim",OBJ_DIM__X+50,  OBJ_DIM__Y+15,40,20);
		t_dim[2] = controlP5.addTextfield("z_dim",OBJ_DIM__X+100, OBJ_DIM__Y+15,40,20);
		
		for(Textfield t : t_dim) {
			t.moveTo("global");
			t.setAutoClear(false);
			t_all_simple.add(t);
			t_all_matrix.add(t);
			t_all_quat.add(t);
			
		}
		

		Button rotx = controlP5.addButton("rot_x", 20, UPDATE_X,    UPDATE_Y-40, 32, 20);
		rotx.setId(20);
		rotx.moveTo("global");

		Button roty = controlP5.addButton("rot_y", 21, UPDATE_X+40, UPDATE_Y-40, 32, 20);
		roty.setId(21);
		roty.moveTo("global");

		Button rotz = controlP5.addButton("rot_z", 22, UPDATE_X+80, UPDATE_Y-40, 32, 20);
		rotz.setId(22);
		rotz.moveTo("global");
		
		
		Button parent = controlP5.addButton("select parent", 24, UPDATE_X, UPDATE_Y, 72, 20);
		parent.moveTo("global");

		t_parent = controlP5.addTextfield("parent", UPDATE_X+85, UPDATE_Y, 130, 20);
		t_parent.moveTo("global");
		
		
		Button export = controlP5.addButton("export to owl", 25, UPDATE_X, UPDATE_Y+40, 72, 20);
		export.moveTo("global");
		
		t_filename = controlP5.addTextfield("filename", UPDATE_X+85, UPDATE_Y+40, 130, 20);
		t_filename.setValue("map.owl");
		t_filename.moveTo("global");
		
		
		/////////////////////////////////////////////////////////////
		// TABS

		// simple position tab
		Textlabel l_pos = controlP5.addTextlabel("l_pos","OBJECT POSITION",TAB_X,TAB_Y);
		l_pos.setWidth(200);

		t_pos = new Textfield[3];
		t_pos[0] = controlP5.addTextfield("x_pos",TAB_X,    TAB_Y+15, 40, 20);
		t_pos[1] = controlP5.addTextfield("y_pos",TAB_X+50, TAB_Y+15, 40, 20);
		t_pos[2] = controlP5.addTextfield("z_pos",TAB_X+100,TAB_Y+15, 40, 20);
		
		for(Textfield t : t_pos) {
			t.setAutoClear(false);
			t_all_simple.add(t);
		}


		// pose matrix tab
		Textlabel l_matrix = controlP5.addTextlabel("l_matrix","POSE MATRIX ",TAB_X,TAB_Y);
		l_matrix.setWidth(200);
		l_matrix.moveTo("matrix");

		t_matrix = new Textfield[16];
		for(int i=0;i<4;i++) {
			for(int j=0;j<4;j++) {

				t_matrix[i*4+j] = controlP5.addTextfield("m"+i+j, TAB_X+j*50, TAB_Y+15 + 40*i, 40, 20);
				t_matrix[i*4+j].moveTo("matrix");
				//t_matrix[i*4+j].setColorBackground(color(255,60));
				t_all_matrix.add(t_matrix[i*4+j]);
			}
		}


		// quaternion pose tab
		Textlabel l_quat = controlP5.addTextlabel("l_qpos","POSE QUATERNION",TAB_X,TAB_Y);
		l_quat.setWidth(200);
		l_quat.moveTo("quaternion");

		t_qpos = new Textfield[3];
		t_qpos[0] = controlP5.addTextfield("x_qpos",TAB_X,    TAB_Y+15, 40,20);
		t_qpos[0].setLabel("x_pos");
		t_qpos[1] = controlP5.addTextfield("y_qpos",TAB_X+50, TAB_Y+15, 40,20);
		t_qpos[1].setLabel("y_pos");
		t_qpos[2] = controlP5.addTextfield("z_qpos",TAB_X+100,TAB_Y+15, 40,20);
		t_qpos[2].setLabel("z_pos");
		
		for(Textfield t : t_qpos) {
			t.moveTo("quaternion");
			t.setAutoClear(false);
			t_all_quat.add(t);
		}
		
		t_quat = new Textfield[4];
		t_quat[0] = controlP5.addTextfield("w",TAB_X,     TAB_Y+55, 40,20);
		t_quat[1] = controlP5.addTextfield("x",TAB_X+50,  TAB_Y+55,40,20);
		t_quat[2] = controlP5.addTextfield("y",TAB_X+100, TAB_Y+55,40,20);
		t_quat[3] = controlP5.addTextfield("z",TAB_X+150, TAB_Y+55,40,20);

		for(Textfield t : t_quat) {
			t.moveTo("quaternion");
			t.setAutoClear(false);
			t_all_quat.add(t);
		}
		
	}


	////////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	// HANDLE USER EVENTS
	//
	//
	////////////////////////////////////////////////////////////////////////////////////////////	
	
	/**
	 * Event handlers for the form elements (buttons, tabs)
	 */
	
	void controlEvent(ControlEvent theControlEvent) {
		
		
		if(theControlEvent.isController()) {
			
			
			// update button
			if(theControlEvent.name().equals("update vis")) {
				
				// read information from the forms, create/update objects[] entry
				readFormData();

				// create visualization item
				updateVisualization();
				
			}
			

			if(theControlEvent.name().equals("new")) {

				clearFormFields(); // new object is implicitly created when a new identifier is used
			}
			

			if(theControlEvent.name().equals("delete")) {

				if(t_id.getText() != null) {
					objects.remove(t_id.getText());
					clearFormFields(); // reset interface
					updateVisualization();
				}
			}


			if(theControlEvent.name().equals("rot_x")) {
				MapObject cur = objects.get(t_id.getText()); 
				cur.pose_matrix.mul(new Matrix4d(
						1, 0, 0,  0,
						0, 0, -1, 0,
						0, 1, 0,  0,
						0, 0, 0,  1));
				updateVisualization();
				writeFormData(cur);
			}
			
			if(theControlEvent.name().equals("rot_y")) {
				MapObject cur = objects.get(t_id.getText()); 
				cur.pose_matrix.mul(new Matrix4d(
						0,	0,	1, 0,
						0,	1,	0, 0,
					   -1,	0,	0, 0,
						0,  0,  0, 1));
				updateVisualization();
				writeFormData(cur);
			}
			
			if(theControlEvent.name().equals("rot_z")) {
				MapObject cur = objects.get(t_id.getText()); 
				cur.pose_matrix.mul(new Matrix4d(
						0,-1, 0,  0,
						1, 0, 0, 0,
						0, 0, 1,  0,
						0, 0, 0,  1));
				updateVisualization();
				writeFormData(cur);
			}
			

			if(theControlEvent.name().equals("export to owl")) {
				
				OWLImportExport exp = new OWLImportExport();
				OWLOntology ont = exp.createOWLMapDescription("ias_map", new ArrayList<MapObject>(this.objects.values()));
				OWLFileUtils.saveOntologyToFile(ont, t_filename.getText());
				
			}

			if(theControlEvent.name().equals("select parent")) {

				selectParentObj=true;
				cursor(HAND);
				this.prologVisCanvas.getKitchenVisApplet().cursor(HAND);
				
			}
			
			
			
		// switch tabs (update information in the tabs)
		} else if (theControlEvent.isTab()) {
			
			if(theControlEvent.name().equals("default")) {
				this.tab = Tab.SIMPLE;
			} else if (theControlEvent.name().equals("matrix")) {
				this.tab = Tab.MATRIX;
			} else if (theControlEvent.name().equals("quaternion")) {
				this.tab = Tab.QUATERNION;
			}
			
			
		} else if (theControlEvent.isGroup()) {
			
			// handle listbox events
			if(theControlEvent.group().id()==42) {			
				t_types.captionLabel().set( objClasses.get( (int) theControlEvent.group().value()) );
				
			}
		}
		redraw();
	}


	
	/**
	 * Keyboard shortcuts
	 * 
	 * <ul>
	 * 	 <li> ENTER updates the visualization with the data entered into the form fields
	 *   <li> TAB/SHIFT-TAB circulate through the text fields forward/backward
	 * 	 <li> PAGE UP/PAGE DOWN increase/decrease the value of a text field containing numeric values
	 *   </ul>
	 */
	public void keyPressed() {
		
		keys[keyCode] = true;
		
		// ENTER key updates visualization
		if (key == ENTER || key == RETURN) {
			readFormData();
			updateVisualization();
		}
		
		
		// switch through the text fields using TAB
		if (keyCode == KeyEvent.VK_TAB) {
			
			boolean fwd = true; // direction to skip through the fields
			if(checkKey(KeyEvent.VK_SHIFT)) {
				fwd=false;
			}
			
			if(this.tab==Tab.SIMPLE) {
				switchFocus(t_all_simple, fwd);
			} else if(this.tab==Tab.MATRIX) {
				switchFocus(t_all_matrix, fwd);
			} else  if(this.tab==Tab.QUATERNION) {
				switchFocus(t_all_quat, fwd);
			}

		}
		
		if (keyCode == KeyEvent.VK_PAGE_UP) {
			
			if(this.tab==Tab.SIMPLE) {
				incTextfieldValue(t_all_simple, 0.05f);
			} else if(this.tab==Tab.MATRIX) {
				incTextfieldValue(t_all_matrix, 0.05f);
			} else  if(this.tab==Tab.QUATERNION) {
				incTextfieldValue(t_all_quat, 0.05f);
			}
			readFormData();
			updateVisualization();
		}
		
		if (keyCode == KeyEvent.VK_PAGE_DOWN) {
			
			if(this.tab==Tab.SIMPLE) {
				incTextfieldValue(t_all_simple, -0.05f);
			} else if(this.tab==Tab.MATRIX) {
				incTextfieldValue(t_all_matrix, -0.05f);
			} else  if(this.tab==Tab.QUATERNION) {
				incTextfieldValue(t_all_quat, -0.05f);
			}
			readFormData();
			updateVisualization();
		}
		
	}

	
	
	/** 
	 * Helper method: update keys[] array when a key is released 
	 * (needed to use modifying keys like SHIFT in addition to TAB)
	 */
	public void keyReleased() { 
	  keys[keyCode] = false; 
	}
	
	
	/**
	 * Check whether a key is currently held down
	 *  
	 * @param k The key to be checked (Java key code)
	 * @return true if the key is currently pressed
	 */
	boolean checkKey(int k) {
		return keys[k];
	}

	
	/** 
	 * Iterate over form fields (used e.g. for the TAB key). Due to the
	 * different tabs, there are multiple ArrayLists containing the
	 * fields for each tab so that the iteration is done correctly.
	 * 
	 * @param t_all ArrayList of text fields to iterate over (for the current tab)
	 * @param forward if true, iterate forwards, otherwise backwards
	 */
	protected void switchFocus(ArrayList<Textfield> t_all, boolean forward) {
		
		for(int i=0;i<t_all.size();i++) {
			if(t_all.get(i).isFocus()){
				
				if(forward) {
					t_all.get(i).setFocus(false);
					t_all.get((i+1)%t_all.size()).setFocus(true);
				} else {
					t_all.get(i).setFocus(false);
					
					if(i>0)
						t_all.get(i-1).setFocus(true);
					else 
						t_all.get(t_all.size()-1).setFocus(true);
				}
				break;
			}
		}
	}
	
	
	/**
	 * Increment the value of a numeric text field by adding the value inc
	 * 
	 * @param t_all ArrayList of the text fields in the current tab
	 * @param inc Increment to add to the value
	 */
	protected void incTextfieldValue(ArrayList<Textfield> t_all, float inc) {
		
		for(int i=0;i<t_all.size();i++) {
			if(t_all.get(i).isFocus()){
				
				try{
					float val = Float.valueOf(t_all.get(i).getText());
					t_all.get(i).setText((val+inc)+"");
				} catch(NumberFormatException e) {}
				
			}
		}
	}
	
	


	////////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	// READ AND WRITE FORM DATA
	//
	//
	////////////////////////////////////////////////////////////////////////////////////////////	
	
	
	
	/**
	 * Read information from the forms and save it in a MapObject instance
	 */
	void readFormData() {
		
		// check if the object already exists or if we should create a new one
		
		MapObject cur;
		String id =  t_id.getText();
		
		if(objects.containsKey(id)) {
			cur = objects.get(id);
		} else {
			cur = new MapObject();
			objects.put(id, cur);
		}
		
		 
		// set global values
		cur.id = id;

		if(t_parent.getText() != null)
			if(objects.get(t_parent.getText())!=null)
				objects.get(t_parent.getText()).physicalParts.add(cur);

		
		cur.types.clear(); // only one type for now
		cur.types.add(t_types.captionLabel().getText());

		
		double xdim=0.0, ydim=0.0, zdim=0.0;
		try{xdim = Double.valueOf(t_dim[0].getText());} catch(NumberFormatException e) {}
		try{ydim = Double.valueOf(t_dim[1].getText());} catch(NumberFormatException e) {}
		try{zdim = Double.valueOf(t_dim[2].getText()); } catch(NumberFormatException e) {}
		cur.setDimensions(new Vector3d(xdim, ydim, zdim));

		
		
		// set per-tab values
		switch(tab) {
		
			case SIMPLE:
				// set only position
				double x=0.0, y=0.0, z=0.0;
				try{x = Double.valueOf(t_pos[0].getText());} catch(NumberFormatException e) {}
				try{y = Double.valueOf(t_pos[1].getText());} catch(NumberFormatException e) {}
				try{z = Double.valueOf(t_pos[2].getText());} catch(NumberFormatException e) {}
				
				cur.setPosition(new Vector3d(x,	y, z ));
				updateQuaternionForm(cur.getPosition(), cur.getPoseQuat());
				updateMatrixForm(cur.getPoseMatrix());
				break;
				
			case MATRIX:
				
				double[] m = new double[16];
				
				try{m[0] = Double.valueOf(t_matrix[0].getText());} catch(NumberFormatException e) {m[0]=0.0;}
				try{m[1] = Double.valueOf(t_matrix[1].getText());} catch(NumberFormatException e) {m[1]=0.0;}
				try{m[2] = Double.valueOf(t_matrix[2].getText());} catch(NumberFormatException e) {m[2]=0.0;}
				try{m[3] = Double.valueOf(t_matrix[3].getText());} catch(NumberFormatException e) {m[3]=0.0;}
				
				try{m[4] = Double.valueOf(t_matrix[4].getText());} catch(NumberFormatException e) {m[4]=0.0;}
				try{m[5] = Double.valueOf(t_matrix[5].getText());} catch(NumberFormatException e) {m[5]=0.0;}
				try{m[6] = Double.valueOf(t_matrix[6].getText());} catch(NumberFormatException e) {m[6]=0.0;}
				try{m[7] = Double.valueOf(t_matrix[7].getText());} catch(NumberFormatException e) {m[7]=0.0;}
				
				try{m[8] = Double.valueOf(t_matrix[8].getText());} catch(NumberFormatException e) {m[8]=0.0;}
				try{m[9] = Double.valueOf(t_matrix[9].getText());} catch(NumberFormatException e) {m[9]=0.0;}
				try{m[10] = Double.valueOf(t_matrix[10].getText());} catch(NumberFormatException e) {m[10]=0.0;}
				try{m[11] = Double.valueOf(t_matrix[11].getText());} catch(NumberFormatException e) {m[11]=0.0;}
				
				try{m[12] = Double.valueOf(t_matrix[12].getText());} catch(NumberFormatException e) {m[12]=0.0;}
				try{m[13] = Double.valueOf(t_matrix[13].getText());} catch(NumberFormatException e) {m[13]=0.0;}
				try{m[14] = Double.valueOf(t_matrix[14].getText());} catch(NumberFormatException e) {m[14]=0.0;}
				try{m[15] = Double.valueOf(t_matrix[15].getText());} catch(NumberFormatException e) {m[15]=0.0;}
				
				cur.setPoseMatrix(new Matrix4d(m));
				updateQuaternionForm(cur.getPosition(), cur.getPoseQuat());
				updatePositionForm(cur.getPosition());
				break;
				
			case QUATERNION:
				
				try{x = Double.valueOf(t_qpos[0].getText());} catch(NumberFormatException e) {x=0.0;}
				try{y = Double.valueOf(t_qpos[1].getText());} catch(NumberFormatException e) {y=0.0;}
				try{z = Double.valueOf(t_qpos[2].getText());} catch(NumberFormatException e) {z=0.0;}
				
				double qx=0.0, qy=0.0, qz=0.0, qw=1.0;
				try{qw = Double.valueOf(t_quat[0].getText());} catch(NumberFormatException e) {}
				try{qy = Double.valueOf(t_quat[1].getText());} catch(NumberFormatException e) {}
				try{qz = Double.valueOf(t_quat[2].getText());} catch(NumberFormatException e) {}
				try{qz = Double.valueOf(t_quat[3].getText());} catch(NumberFormatException e) {}
				
				cur.setPoseQuat(new Vector3d(x,y,z), 
								new Quat4d(qx,qy,qz,qw),1.0);
				
				updateMatrixForm(cur.getPoseMatrix());
				updatePositionForm(cur.getPosition());
				break;
		}

	}
	
	/** 
	 * Load the information from the MapObject cur into the forms
	 * 
	 * @param cur Current MapObject providing the information to be filled into the forms 
	 */
	void writeFormData(MapObject cur) {
		
		t_id.setText(cur.id);
		
		
		// find object that has this object as physical part
		for(MapObject o : objects.values()) {

			if(o.physicalParts.contains(cur)) {
				t_parent.setText(o.getId());
				break;
			} else {
				t_parent.setText("");
			}
		}
		
		t_types.captionLabel().set(cur.types.get(0));

		t_dim[0].setText(""+cur.dimensions.x);
		t_dim[1].setText(""+cur.dimensions.y);
		t_dim[2].setText(""+cur.dimensions.z);
		
		updateQuaternionForm(cur.getPosition(), cur.getPoseQuat());
		updateMatrixForm(cur.getPoseMatrix());
		updatePositionForm(cur.getPosition());
	}
	
	
	/**
	 * Update the matrix form fields (if data in another form has been changed)
	 * 
	 * @param m Matrix4d with the values to be filled in
	 */
	void updateMatrixForm(Matrix4d m) {
		
		t_matrix[0].setText(""+m.m00);
		t_matrix[1].setText(""+m.m01);
		t_matrix[2].setText(""+m.m02);
		t_matrix[3].setText(""+m.m03);
		
		t_matrix[4].setText(""+m.m10);
		t_matrix[5].setText(""+m.m11);
		t_matrix[6].setText(""+m.m12);
		t_matrix[7].setText(""+m.m13);
		
		t_matrix[8].setText(""+m.m20);
		t_matrix[9].setText(""+m.m21);
		t_matrix[10].setText(""+m.m22);
		t_matrix[11].setText(""+m.m23);
		
		t_matrix[12].setText(""+m.m30);
		t_matrix[13].setText(""+m.m31);
		t_matrix[14].setText(""+m.m32);
		t_matrix[15].setText(""+m.m33);
	}

	/**
	 * Update the position form fields (if data in another form has been changed)
	 * 
	 * @param pos Vector3d with the position data to be filled in
	 */
	void updatePositionForm(Vector3d pos) {
		
		t_pos[0].setText(""+pos.x);
		t_pos[1].setText(""+pos.y);
		t_pos[2].setText(""+pos.z);
	}
	
	
	/**
	 * Update the quaternion form fields (if data in another form has been changed)
	 * 
	 * @param pos Vector3d with the position data to be filled in
	 * @param q   Quat4d with the orientation data to be filled in
	 */
	void updateQuaternionForm(Vector3d pos, Quat4d q) {
		
		t_qpos[0].setText(""+pos.x);
		t_qpos[1].setText(""+pos.y);
		t_qpos[2].setText(""+pos.z);
		
		t_quat[0].setText(""+q.w);
		t_quat[1].setText(""+q.x);
		t_quat[2].setText(""+q.y);
		t_quat[3].setText(""+q.z);
	}
	
	
	void clearFormFields(){
		
		for(Textfield t : t_all_simple) {
			t.setText("");
		}
		for(Textfield t : t_all_matrix) {
			t.setText("");
		}
		for(Textfield t : t_all_quat) {
			t.setText("");
		}
		
	}
	

	////////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	// MANAGE OBJECT INSTANCES
	//
	//
	////////////////////////////////////////////////////////////////////////////////////////////	

	/**
	 * Update the displayed map visualization with all objects in the buffer
	 */
	protected void updateVisualization() {
		
		prologVisCanvas.getKitchenVisApplet().clear();
		
		for(MapObject o : objects.values()) {
		
			ItemBase item = createItem(o);
			
			if(item!=null) {
				item.name=o.id;
				
				prologVisCanvas.getKitchenVisApplet().addItem(item);
				
			} else {
				System.err.println("Object "+o.id+"not found.");
			}			
		}
		prologVisCanvas.getKitchenVisApplet().redraw();

	}
	
	

	/** 
	 * Create an 'item', the Java object for visualizing a map object
	 * 
	 * This methods maps OWL classes to the corresponding Java visualization items  
	 * 
	 * @param obj MapObject to be visualized
	 * @return    Visualization item created from the MapObject
	 */
	Item createItem(MapObject obj) {

		String type = obj.types.get(0);
		
		int col = grayValues[(++grayLevelCounter) % grayValues.length];      
		
		
		/////////////////////////////////////////////
		// tableware

		if(type.endsWith("Cup")) {
			return new Cup(obj.pose_matrix, obj.dimensions);

		} else if(type.endsWith("DinnerPlate")) {
			return new Plate(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("DrinkingGlass")) {
			return new DrinkingGlass(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Bowl-Eating")) {
			return new BowlEating(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("DrinkingBottle")) {
			return new Sphere(obj.pose_matrix, new Vector3d(0.05f,0.05f,0.15f));


			
			/////////////////////////////////////////////
			// silverware	

		} else if(type.endsWith("DinnerFork")) {
			return new Fork(obj.pose_matrix, obj.dimensions);

		} else if(type.endsWith("TableKnife")) {
			return new Knife(obj.pose_matrix, obj.dimensions);

		} else if(type.endsWith("SoupSpoon")) {
			return new Spoon(obj.pose_matrix, obj.dimensions);


			/////////////////////////////////////////////
			// serving and cooking

		} else if(type.endsWith("Napkin")) {
			return new Napkin(obj.pose_matrix, obj.dimensions);

		} else if(type.endsWith("PlaceMat")) {
			return new PlaceMat(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Platter")) {
			return new Platter(obj.pose_matrix, obj.dimensions);	

		} else if(type.endsWith("CookingPot")) {
			return new CookingPot(obj.pose_matrix, obj.dimensions);

		} else if(type.endsWith("Spatula")) {
			return new Spatula(obj.pose_matrix, obj.dimensions);

		} else if(type.endsWith("PancakeMaker")) {
			return new PancakeMaker(obj.pose_matrix, obj.dimensions);


			/////////////////////////////////////////////
			// breakfast consumables	

		} else if (type.endsWith("Bread")) {
			return new Bread(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Cheese")) {
			return new Cheese(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Sausage")) {
			return new Sausage(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Cake")) {
			return new Cake(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("BreakfastCereal")) {
			return new CerealBox(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("PancakeMix")) {
			return new PancakeMix(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Pancake")) {
			return new Pancake(obj.pose_matrix, obj.dimensions);


			/////////////////////////////////////////////
			// lunch/dinner consumables

		} else if (type.endsWith("Pizza")) {
			return new Pizza(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Salad")) {
			return new SaladBowl(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Pasta")) {
			return new SaladBowl(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Soup")) {
			return new SoupPlate(obj.pose_matrix, obj.dimensions);



			/////////////////////////////////////////////
			// drinks

		} else if (type.endsWith("Water")) {
			return new Bottle(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Pitcher")) {
			return new Thermos(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Tea-Beverage")) {
			return new Bread(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Coffee-Beverage")) {
			return new Thermos(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Juice")) {
			return new Tetrapak(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("Tea-Iced")) {
			return new Tetrapak(obj.pose_matrix, obj.dimensions);

		} else if(type.endsWith("Tetrapak")) {
			return new Tetrapak(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("CardboardBox")) {
			return new Bread(obj.pose_matrix, obj.dimensions);

		} else if (type.endsWith("CowsMilk-Product")) {
			return new Tetrapak(obj.pose_matrix, obj.dimensions);


			/////////////////////////////////////////////
			// furniture

		} else if(type.endsWith("Chair-PieceOfFurniture")) {
			Chair c = new Chair(obj.pose_matrix, obj.dimensions);
			col = grayValues[(++grayLevelCounter) % grayValues.length];
			c.setColor(StandaloneKitchenVisApplet.convertColor(col, col, col, 255));
			return c; 

		} else if(type.endsWith("Cupboard")) {
			Cupboard c = new Cupboard(obj.pose_matrix, obj.dimensions);
			col = grayValues[(++grayLevelCounter) % grayValues.length];
			c.setColor(StandaloneKitchenVisApplet.convertColor(col, col, col, 255));
			return c;

		} else if(type.endsWith("Drawer")) {
			Cupboard c = new Cupboard(obj.pose_matrix, obj.dimensions);
			col = grayValues[(++grayLevelCounter) % grayValues.length];
			c.setColor(StandaloneKitchenVisApplet.convertColor(col, col, col, 255));
			return c;

		} else if(type.endsWith("Dishwasher")) {
			Cupboard c = new Cupboard(obj.pose_matrix, obj.dimensions);
			col = grayValues[(++grayLevelCounter) % grayValues.length];
			c.setColor(StandaloneKitchenVisApplet.convertColor(col, col, col, 255));
			return c;

		} else if(type.endsWith("Oven")) {
			Cupboard c = new Cupboard(obj.pose_matrix, obj.dimensions);
			col = grayValues[(++grayLevelCounter) % grayValues.length];
			c.setColor(StandaloneKitchenVisApplet.convertColor(col, col, col, 255));
			return c;

		} else if(type.endsWith("Refrigerator")) {
			Cupboard c = new Cupboard(obj.pose_matrix, obj.dimensions);
			col = grayValues[(++grayLevelCounter) % grayValues.length];
			c.setColor(StandaloneKitchenVisApplet.convertColor(col, col, col, 255));
			return c;

		} else if(type.endsWith("Handle")) {
			BoxHandle b = new BoxHandle(obj.pose_matrix, obj.dimensions);
			col = grayValues[(++grayLevelCounter) % grayValues.length];
			b.setColor(StandaloneKitchenVisApplet.convertColor(col, col, col, 255));
			return b;

		} else if(type.endsWith("ControlKnob")) {
			Sphere s = new Sphere(obj.pose_matrix, obj.dimensions);
			col = grayValues[(++grayLevelCounter) % grayValues.length];
			s.setColor(StandaloneKitchenVisApplet.convertColor(col, col, col, 255));
			return s; 
			
		} else if(type.endsWith("Door")) {
			Door d = new Door(obj.pose_matrix, obj.dimensions);
			d.setColor(StandaloneKitchenVisApplet.convertColor(255, 175, 0, 255));
			return d;

		} else if(type.endsWith("HingedJoint")) {
			HingedJoint h = new HingedJoint(obj.pose_matrix, obj.dimensions);
			h.setColor(StandaloneKitchenVisApplet.convertColor(70, 120, 255, 255));
			return h; 
			
		} else if(type.endsWith("PrismaticJoint")) {
			PrismaticJoint h = new PrismaticJoint(obj.pose_matrix, new Vector3d(0.02f, 0.02f, 0.02f));
			h.setColor(StandaloneKitchenVisApplet.convertColor(70, 255, 120, 255));
			return h; 

			/////////////////////////////////////////////
			// dummies

		} else if(type.endsWith("SpatialThing-Localized")) {
			return new Ellipse(obj.pose_matrix, obj.dimensions);

		} else if(type.endsWith("Place")) {
			return new Ellipse(obj.pose_matrix, obj.dimensions);

		} else if(type.endsWith("Point3D")) {
			return new Sphere(obj.pose_matrix, new Vector3d(3f,3f,3f));
		}
		return null;	

	}
	
	


	////////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	// INTERFACE WITH PROLOG CANVAS
	//
	//
	////////////////////////////////////////////////////////////////////////////////////////////	

	/**
	 * Get the internal reference to the overlying {@link PrologVisualizationCanvas} 
	 */
	
	public PrologVisualizationCanvas getPrologVisCanvas() {
		return prologVisCanvas;
	}

	
	/**
	 * Set the internal reference to the overlying {@link PrologVisualizationCanvas}
	 * @param prologVisCanvas Reference to a {@link PrologVisualizationCanvas}
	 */
	
	public void setPrologVisCanvas(PrologVisualizationCanvas prologVisCanvas) {
		this.prologVisCanvas = prologVisCanvas;
	}

	
	/**
	 * Click-interface with higher-level PrologVisualizationCanvas: forward
	 * information which item in the visualization the user clicked on
	 * 
	 * Depending on the state, the click is either interpreted as selection
	 * of the current object or of the parent object (if selectParentObj=true)
	 * 
	 * @param identifier String-identifier of the clicked-on object
	 */
	
	void editObject(String identifier) {

		// special mode: select the parent object for the currently edited object
		if(selectParentObj) {
			
			MapObject obj = objects.get(t_id.getText());
			
			// clear previous parent objects
			for(MapObject o : objects.values()) {
				if(o.physicalParts.contains(obj)) {
					o.physicalParts.remove(obj);
				}
			}
			
			objects.get(identifier).physicalParts.add(obj);
			this.t_parent.setText(identifier);
			
			selectParentObj=false;
			cursor(ARROW);
			this.prologVisCanvas.getKitchenVisApplet().cursor(ARROW);
			
			
		} else {
			// normal mode: load information about the object the user clicked on into the form fields 
			writeFormData(objects.get(identifier));
		}
	}
	
}
