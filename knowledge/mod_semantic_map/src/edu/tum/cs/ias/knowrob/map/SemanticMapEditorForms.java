package edu.tum.cs.ias.knowrob.map;

import java.awt.Container;
import java.awt.Frame;
import java.awt.event.KeyEvent;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Vector;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;

import controlP5.Button;
import controlP5.ControlP5;
import controlP5.ListBox;
import controlP5.ListBoxItem;
import controlP5.Textfield;
import controlP5.Textlabel;
import edu.tum.cs.ias.knowrob.owl.JointInstance;
import edu.tum.cs.ias.knowrob.owl.OWLClass;
import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.owl.ObjectInstance;
import edu.tum.cs.ias.knowrob.owl.utils.OWLFileUtils;
import edu.tum.cs.ias.knowrob.owl.utils.OWLImportExport;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.prolog.PrologQueryUtils;
import edu.tum.cs.ias.knowrob.vis.applets.SemanticMapVisApplet;
import edu.tum.cs.ias.knowrob.vis.themes.GreyTheme;
import processing.core.PApplet;



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

	private SemanticMapVisApplet mapVisApplet = null;
	public ControlP5 controlP5;
	boolean[] keys = new boolean[526];
	boolean selectParentObj = false; // switch mode from normal obj selection to parent obj selection
	boolean selectChildObj = false;  // switch mode from normal obj selection to child obj selection
	
	// form elements
	Textfield t_id;
	ListBox t_types;
	
	Textfield[] t_dim;
	Textfield[] t_matrix;
	Textfield[] t_quat;
	Textfield[] t_pos;
	Textfield[] t_qpos;
	Textfield[] t_jointdir;
	Textfield[] t_jointlim;
	public Textfield t_filename;
	Textfield t_namespace;
	Textfield t_map_id;
	Textfield t_parent;
	Textfield t_radius;
	Textlabel l_jointdir;
	Textlabel l_jointlim;
	Textlabel l_radius;
	Textlabel l_dim;
	Button[] b_rot;
	Button b_child;
	Button b_new_obj;
	Button b_del_obj;
	Button b_update;
	Textfield t_child;
	ArrayList<Textfield> t_all_simple; // for using the tab key
	ArrayList<Textfield> t_all_matrix; // for using the tab key
	ArrayList<Textfield> t_all_quat; // for using the tab key
	ArrayList<Textfield> t_all_joint; // for using the tab key
	
	public enum Tab {SIMPLE, MATRIX, QUATERNION, JOINTS, SETTINGS};
	Tab tab;
	
	///////////////////////////////////////////////////////////////
	
	
	
	/**
	 * Internal storage of the ObjectInstance instances
	 */
	HashMap<String, ObjectInstance> objects;

	boolean cp5_initialized = false;
	
	String map_id = "";
	String map_namespace = "";
	
	
	/**
	 * List of objects to be displayed in the object classes selector
	 */
	protected static final HashMap<Integer, String> objClasses = new HashMap<Integer, String>();
	static {
		objClasses.put(1022,   "Sushi");
		objClasses.put(1023,   "BentoBox");
		objClasses.put(1024,   "Riceball");
		objClasses.put(1025,   "Pizza");
		objClasses.put(1026,   "Sandwich");
		objClasses.put(1027,   "HamburgerSandwich");
		objClasses.put(1028,   "Beer");
		objClasses.put(1029,   "Water");
		objClasses.put(1030,   "Coffee-Beverage");
		objClasses.put(1031,   "ColaSoftDrink");
		objClasses.put(1032,   "LemonLimeSoftDrink");
		objClasses.put(1033,   "GreenTea");
		objClasses.put(1034,   "EnergyDrink");
		objClasses.put(1035,   "Chip-Food");
		objClasses.put(1036,   "Cracker-FoodItem");
		objClasses.put(1037,   "Pretzel");
		objClasses.put(1038,   "InstantRamenNoodles");
		objClasses.put(1039,   "Cake");
		objClasses.put(1040,   "Bread");
		objClasses.put(1041,   "ChocolateCandy");
		objClasses.put(1042,   "CandyBar");
		objClasses.put(1043,   "Caramel");
		objClasses.put(1044,   "ChewingGum");
		objClasses.put(1045,   "Shampoo");
		objClasses.put(1046,   "BodyWash");
		objClasses.put(1047,   "Toothpaste");
		objClasses.put(1048,   "ShavingCream");
		objClasses.put(1049,   "ToiletPaper");
		objClasses.put(1050,   "Envelope");
		objClasses.put(1051,   "SheetOfPaper");
		objClasses.put(1052,   "Pencil");
		objClasses.put(1053,   "Notepad");
		objClasses.put(1054,   "Toy");
		objClasses.put(1055,   "CashRegister");
		objClasses.put(1056,   "ShelfInABuilding");
		objClasses.put(1057,   "Robot");
		
//		objClasses.put(1022,   "Cupboard");
//		objClasses.put(1023,   "Dishwasher");
//		objClasses.put(1024,   "Drawer");
//		objClasses.put(1025,   "Oven");
//		objClasses.put(1026,   "Refrigerator");
//		objClasses.put(1027,   "Door");
//		objClasses.put(1028,   "Handle");
//		objClasses.put(1029,   "ControlKnob");
//		objClasses.put(1030,   "Cabinet-PieceOfFurniture");
//		objClasses.put(1031,   "Bed-PieceOfFurniture");
//		objClasses.put(1032,   "HingedJoint");
//		objClasses.put(1033,   "PrismaticJoint");
//		objClasses.put(1034,   "RoomInAConstruction");
//		objClasses.put(1035,   "ShelfInABuilding");
//		objClasses.put(1036,   "Sushi");
//		objClasses.put(1037,   "Tea-Beverage");
//		objClasses.put(1038,   "Computer");
//		objClasses.put(1039,   "CashRegister");
//		objClasses.put(1040,   "Whiteboard");
//		objClasses.put(1041,   "Desk");
		
	}
	
	
	/**
	 * Initialize the interface and data structures 
	 */
	public void setup() {

		size(320, 600, P2D);
		frameRate(20);
		objects = new HashMap<String, ObjectInstance>();
		tab=Tab.SIMPLE;

		initControlP5Forms();
		
		frameRate(20);
		
		draw(); 
		
	}
	
	public Frame findFrame() {	
		Container f = this.getParent();
		while (!(f instanceof Frame) && f!=null)
			f = f.getParent();
		return (Frame) f;
	}
	
	/**
	 * Draw-method: call controlP5.draw to update the content of the forms.
	 */
	public void draw() {

		background(40);
		controlP5.draw();
		
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
		
		int OBJ_DIM_X=10;
		int OBJ_DIM_Y=200;

		int TAB_X=10;
		int TAB_Y=300;
		
		int UPDATE_X = 10;
		int UPDATE_Y = 500;

		// for using the tab key
		t_all_simple = new ArrayList<Textfield>();
		t_all_matrix = new ArrayList<Textfield>();
		t_all_quat = new ArrayList<Textfield>();
		t_all_joint = new ArrayList<Textfield>();
		
		
		// wait till window has been created
		while(findFrame()==null) {
			try { Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace(); } 
		}
		this.frame = findFrame();
		
		
		controlP5 = new ControlP5(this);
		GreyTheme.applyStyle(controlP5);
		
		GreyTheme.applyStyle(controlP5.getTab("default")).setLabel("simple").setWidth(35);
		
		GreyTheme.applyStyle(controlP5.addTab("matrix")).setWidth(35);
		
		GreyTheme.applyStyle(controlP5.addTab("quaternion")).setWidth(50);

		GreyTheme.applyStyle(controlP5.addTab("joint properties")).setWidth(80);

		GreyTheme.applyStyle(controlP5.addTab("settings")).setWidth(45);
		
		
		// text field for object name
		t_id = controlP5.addTextfield("object id",       OBJ_ID_X, OBJ_ID_Y, 200, 20);
		t_id.moveTo("global");
		t_all_simple.add(t_id); t_all_matrix.add(t_id); t_all_quat.add(t_id);
		t_id.setAutoClear(false);
		

		b_new_obj = GreyTheme.applyStyle(controlP5.addButton("new",    17, OBJ_ID_X,      OBJ_ID_Y+40, 24, 20));
		b_new_obj.moveTo("global");

		b_del_obj = GreyTheme.applyStyle(controlP5.addButton("delete", 18, OBJ_ID_X + 30, OBJ_ID_Y+40, 34, 20));
		b_del_obj.moveTo("global");

		b_update = GreyTheme.applyStyle(controlP5.addButton("add to map", 23, OBJ_ID_X+70, OBJ_ID_Y+40, 55, 20));
		b_update.setId(23);
		b_update.moveTo("global");

		
		t_types = GreyTheme.applyStyle(controlP5.addListBox("object class",   OBJ_ID_X, OBJ_ID_Y+100,200,60), 15);
		t_types.setId(42);

		t_types.moveTo("global");
		
		ListBoxItem b;
		OWLClass cl;
		for(int i: objClasses.keySet()) {
			
			// add to OWLClass identifier buffer to enable later resolution by shortname
			cl = OWLClass.getOWLClass("http://ias.cs.tum.edu/kb/knowrob.owl#" + objClasses.get(i));
			
			b = t_types.addItem(cl.getShortName(),i);
			
			b.setId(i);
		}
		
		// object dimensions
		l_dim = GreyTheme.applyStyle(controlP5.addTextlabel("l_dim","OBJECT DIMENSIONS",OBJ_DIM_X,OBJ_DIM_Y));
		l_dim.setWidth(200);
		l_dim.moveTo("global");
		
		t_dim = new Textfield[3];
		t_dim[0] = GreyTheme.applyStyle(controlP5.addTextfield("x_dim",OBJ_DIM_X,     OBJ_DIM_Y+15, 40,20));
		t_dim[1] = GreyTheme.applyStyle(controlP5.addTextfield("y_dim",OBJ_DIM_X+50,  OBJ_DIM_Y+15,40,20));
		t_dim[2] = GreyTheme.applyStyle(controlP5.addTextfield("z_dim",OBJ_DIM_X+100, OBJ_DIM_Y+15,40,20));
		
		for(Textfield t : t_dim) {
			t.moveTo("global");
			t.setAutoClear(false);
			t_all_simple.add(t);
			t_all_matrix.add(t);
			t_all_quat.add(t);
		}
		b_rot = new Button[3];
		b_rot[0] = GreyTheme.applyStyle(controlP5.addButton("rot_x", 20, OBJ_DIM_X,    OBJ_DIM_Y+55, 32, 20));
		b_rot[0].setId(20);
		b_rot[0].moveTo("global");

		b_rot[1] = GreyTheme.applyStyle(controlP5.addButton("rot_y", 21, OBJ_DIM_X+50, OBJ_DIM_Y+55, 32, 20));
		b_rot[1].setId(21);
		b_rot[1].moveTo("global");

		b_rot[2] = GreyTheme.applyStyle(controlP5.addButton("rot_z", 22, OBJ_DIM_X+100, OBJ_DIM_Y+55, 32, 20));
		b_rot[2].setId(22);
		b_rot[2].moveTo("global");
		
		
		
		
		
		Button parent = GreyTheme.applyStyle(controlP5.addButton("select parent", 24, UPDATE_X, UPDATE_Y, 72, 20));
		parent.moveTo("global");

		t_parent = GreyTheme.applyStyle(controlP5.addTextfield("parent", UPDATE_X+85, UPDATE_Y, 130, 20));
		t_parent.moveTo("global");
		
		
		Button export = GreyTheme.applyStyle(controlP5.addButton("export to owl", 25, UPDATE_X, UPDATE_Y+40, 72, 20));
		export.moveTo("global");
		
		t_filename = GreyTheme.applyStyle(controlP5.addTextfield("filename", UPDATE_X+85, UPDATE_Y+40, 130, 20));
		t_filename.moveTo("global");
		
		
		/////////////////////////////////////////////////////////////
		// TABS

		// simple position tab
		Textlabel l_pos = GreyTheme.applyStyle(controlP5.addTextlabel("l_pos","OBJECT POSITION",TAB_X,TAB_Y));
		l_pos.setWidth(200);

		t_pos = new Textfield[3];
		t_pos[0] = GreyTheme.applyStyle(controlP5.addTextfield("x_pos",TAB_X,    TAB_Y+15, 40, 20));
		t_pos[1] = GreyTheme.applyStyle(controlP5.addTextfield("y_pos",TAB_X+50, TAB_Y+15, 40, 20));
		t_pos[2] = GreyTheme.applyStyle(controlP5.addTextfield("z_pos",TAB_X+100,TAB_Y+15, 40, 20));
		
		for(Textfield t : t_pos) { // for using the tab key
			t.setAutoClear(false);
			t_all_simple.add(t);
		}


		// pose matrix tab
		Textlabel l_matrix = GreyTheme.applyStyle(controlP5.addTextlabel("l_matrix","POSE MATRIX ",TAB_X,TAB_Y));
		l_matrix.setWidth(200);
		l_matrix.moveTo("matrix");

		t_matrix = new Textfield[16];
		for(int i=0;i<4;i++) {
			for(int j=0;j<4;j++) {

				t_matrix[i*4+j] = GreyTheme.applyStyle(controlP5.addTextfield("m"+i+j, TAB_X+j*50, TAB_Y+15 + 40*i, 40, 20));
				t_matrix[i*4+j].moveTo("matrix");
				//t_matrix[i*4+j].setColorBackground(color(255,60));
				t_all_matrix.add(t_matrix[i*4+j]);
			}
		}


		// quaternion pose tab
		Textlabel l_quat = GreyTheme.applyStyle(controlP5.addTextlabel("l_qpos","POSE QUATERNION",TAB_X,TAB_Y));
		l_quat.setWidth(200);
		l_quat.moveTo("quaternion");

		t_qpos = new Textfield[3];
		t_qpos[0] = GreyTheme.applyStyle(controlP5.addTextfield("x_qpos",TAB_X,    TAB_Y+15, 40,20)).setCaptionLabel("x_pos");
		t_qpos[1] = GreyTheme.applyStyle(controlP5.addTextfield("y_qpos",TAB_X+50, TAB_Y+15, 40,20)).setCaptionLabel("y_pos");
		t_qpos[2] = GreyTheme.applyStyle(controlP5.addTextfield("z_qpos",TAB_X+100,TAB_Y+15, 40,20)).setCaptionLabel("z_pos");
		
		for(Textfield t : t_qpos) { // for using the tab key
			t.moveTo("quaternion");
			t.setAutoClear(false);
			t_all_quat.add(t);
		}
		
		t_quat = new Textfield[4];
		t_quat[0] = GreyTheme.applyStyle(controlP5.addTextfield("w",TAB_X,     TAB_Y+55, 40,20));
		t_quat[1] = GreyTheme.applyStyle(controlP5.addTextfield("x",TAB_X+50,  TAB_Y+55,40,20));
		t_quat[2] = GreyTheme.applyStyle(controlP5.addTextfield("y",TAB_X+100, TAB_Y+55,40,20));
		t_quat[3] = GreyTheme.applyStyle(controlP5.addTextfield("z",TAB_X+150, TAB_Y+55,40,20));

		for(Textfield t : t_quat) { // for using the tab key
			t.moveTo("quaternion");
			t.setAutoClear(false);
			t_all_quat.add(t);
		}
		

		// joint properties tab
		l_jointdir = GreyTheme.applyStyle(controlP5.addTextlabel("l_joint","DIRECTION (PRISMATIC ONLY)",TAB_X,TAB_Y));
		l_jointdir.setWidth(200);
		l_jointdir.moveTo("joint properties");

		t_jointdir = new Textfield[3];
		t_jointdir[0] = GreyTheme.applyStyle(controlP5.addTextfield("dir_x",TAB_X,    TAB_Y+15, 40,20).setCaptionLabel("dir_x"));
		t_jointdir[1] = GreyTheme.applyStyle(controlP5.addTextfield("dir_y",TAB_X+50, TAB_Y+15, 40,20).setCaptionLabel("dir_y"));
		t_jointdir[2] = GreyTheme.applyStyle(controlP5.addTextfield("dir_z",TAB_X+100,TAB_Y+15, 40,20).setCaptionLabel("dir_z"));
		
		for(Textfield t : t_jointdir) { // for using the tab key
			t.moveTo("joint properties");
			t.setAutoClear(false);
			t_all_joint.add(t);
		}

		
		l_jointlim = GreyTheme.applyStyle(controlP5.addTextlabel("l_jointlim","JOINT LIMITS",TAB_X,TAB_Y+60));
		l_jointlim.setWidth(200);
		l_jointlim.moveTo("joint properties");

		t_jointlim = new Textfield[2];
		t_jointlim[0] = GreyTheme.applyStyle(controlP5.addTextfield("q_min",TAB_X,    TAB_Y+75, 40,20).setCaptionLabel("q_min"));
		t_jointlim[1] = GreyTheme.applyStyle(controlP5.addTextfield("q_max",TAB_X+50, TAB_Y+75, 40,20).setCaptionLabel("q_max"));
		
		for(Textfield t : t_jointlim) { // for using the tab key
			t.moveTo("joint properties");
			t.setAutoClear(false);
			t_all_joint.add(t);
		}
		

		l_radius = GreyTheme.applyStyle(controlP5.addTextlabel("l_radius","RADIUS",TAB_X+100,TAB_Y+60));
		l_radius.setWidth(200);
		l_radius.moveTo("joint properties");

		t_radius = GreyTheme.applyStyle(controlP5.addTextfield("radius",TAB_X+100,    TAB_Y+75, 40,20).setCaptionLabel("radius"));
		t_radius.moveTo("joint properties");
		

		b_child = GreyTheme.applyStyle(controlP5.addButton("select child", 26, UPDATE_X, UPDATE_Y-40, 72, 20));
		b_child.moveTo("joint properties");

		t_child = GreyTheme.applyStyle(controlP5.addTextfield("child", UPDATE_X+85, UPDATE_Y-40, 130, 20));
		t_child.moveTo("joint properties");
		
		

		// settings tab
		Textlabel l_settings = GreyTheme.applyStyle(controlP5.addTextlabel("l_settings","GLOBAL SETTINGS",OBJ_ID_X,OBJ_ID_Y));
		l_settings.setWidth(200);
		l_settings.moveTo("settings");
		
		t_namespace = GreyTheme.applyStyle(controlP5.addTextfield("OWL NAMESPACE",OBJ_ID_X, OBJ_ID_Y+15, 250, 20));
		
		if(map_namespace!=null && !map_namespace.equals("")) {
			t_namespace.setText(map_namespace);
		} else {
			t_namespace.setText("http://ias.cs.tum.edu/kb/ias_semantic_map.owl#");
		}
		t_namespace.moveTo("settings");
		
		// init semantic map ID with a generated quasi-unique string
		t_map_id = GreyTheme.applyStyle(controlP5.addTextfield("MAP IDENTIFIER",OBJ_ID_X, OBJ_ID_Y+55, 250, 20));
		
		if(map_id!=null && !map_id.equals("")) {
			t_map_id.setText(map_id);
		} else {
			t_map_id.setText("SemanticEnvironmentMap" + 
					new SimpleDateFormat("yyyyMMddHHmmss").
					format(Calendar.getInstance().getTime()));
		}
		t_map_id.moveTo("settings");
		
		
		cp5_initialized  = true;
		
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
	
	public void controlEvent(controlP5.ControlEvent theControlEvent) {
		
		if(theControlEvent.isController()) {
			
			// update button
			if(theControlEvent.getName().equals("add to map")) {
				
				// read information from the forms, create/update objects[] entry
				String obj = readFormData();

				if(obj!=null)
					mapVisApplet.addObject(obj);
				
			}
			

			if(theControlEvent.getName().equals("new")) {

				clearFormFields(); // new object is implicitly created when a new identifier is used
			}
			

			if(theControlEvent.getName().equals("delete")) {

				if(t_id.getText() != null) {
					objects.remove(t_id.getText());
					
					String id = t_id.getText();
					OWLThing rem = OWLThing.getOWLThingByShortname(id);
					ObjectInstance reminst = (ObjectInstance) rem; 
					
					for(ObjectInstance o : objects.values()) {
						
						if(o.hasPhysicalPart(reminst)) {
							o.removePhysicalPart(reminst);
						}
						
						Vector<String> propval = o.getObjPropValues("http://ias.cs.tum.edu/kb/knowrob.owl#connectedTo-Rigidly");
						if(propval!=null && propval.contains(reminst.getIRI())) {
							propval.remove(reminst.getIRI());
						}
						
						propval = o.getObjPropValues("http://ias.cs.tum.edu/kb/knowrob.owl#hingedTo");
						if(propval!=null && propval.contains(reminst.getIRI())) {
							propval.remove(reminst.getIRI());
						}
						
						propval = o.getObjPropValues("http://ias.cs.tum.edu/kb/knowrob.owl#prismaticallyConnectedTo");
						if(propval!=null && propval.contains(reminst.getIRI())) {
							propval.remove(reminst.getIRI());
						}
					}
					
					PrologQueryUtils.deleteObjectInstance(reminst.getIRI());
					clearFormFields(); // reset interface
					
					mapVisApplet.removeObject(rem.getIRI());
					mapVisApplet.redraw();
				}
			}


			if(theControlEvent.getName().equals("rot_x")) {
				ObjectInstance cur = ((ObjectInstance) OWLThing.getOWLThingByShortname(t_id.getText())); 
				cur.getPoseMatrix().mul(new Matrix4d(
						1, 0, 0,  0,
						0, 0, -1, 0,
						0, 1, 0,  0,
						0, 0, 0,  1));
				cur.setSaveToProlog(true);
				cur.writeToProlog();
				

				mapVisApplet.removeObject(cur.getIRI());
				mapVisApplet.addObject(cur.getIRI());
				mapVisApplet.redraw();
				writeFormData(cur);
			}
			
			if(theControlEvent.getName().equals("rot_y")) {
				ObjectInstance cur = ((ObjectInstance) OWLThing.getOWLThingByShortname(t_id.getText())); 
				cur.getPoseMatrix().mul(new Matrix4d(
						0,	0,	1, 0,
						0,	1,	0, 0,
					   -1,	0,	0, 0,
						0,  0,  0, 1));
				cur.setSaveToProlog(true);
				cur.writeToProlog();
				

				mapVisApplet.removeObject(cur.getIRI());
				mapVisApplet.addObject(cur.getIRI());
				mapVisApplet.redraw();
				writeFormData(cur);
			}
			
			if(theControlEvent.getName().equals("rot_z")) {
				ObjectInstance cur = ((ObjectInstance) OWLThing.getOWLThingByShortname(t_id.getText())); 
				cur.getPoseMatrix().mul(new Matrix4d(
						0,-1, 0,  0,
						1, 0, 0, 0,
						0, 0, 1,  0,
						0, 0, 0,  1));
				cur.setSaveToProlog(true);
				cur.writeToProlog();
				

				mapVisApplet.removeObject(cur.getIRI());
				mapVisApplet.addObject(cur.getIRI());
				mapVisApplet.redraw();
				writeFormData(cur);
			}
			

			if(theControlEvent.getName().equals("export to owl")) {
				saveMapToFile(t_filename.getText());
			}

			if(theControlEvent.getName().equals("select parent")) {

				selectParentObj=true;
				cursor(HAND);
				this.mapVisApplet.cursor(HAND);
				
			}
			
			if(theControlEvent.getName().equals("select child")) {

				selectChildObj=true;
				cursor(HAND);
				this.mapVisApplet.cursor(HAND);
				
			}
			
			
			
		// switch tabs (update information in the tabs)
		} else if (theControlEvent.isTab()) {
			
			if(theControlEvent.getName().equals("default")) {
				this.tab = Tab.SIMPLE;
				showObjSpecificElements();
				
			} else if (theControlEvent.getName().equals("matrix")) {
				this.tab = Tab.MATRIX;
				showObjSpecificElements();
				
			} else if (theControlEvent.getName().equals("quaternion")) {
				this.tab = Tab.QUATERNION;
				showObjSpecificElements();

			} else if (theControlEvent.getName().equals("settings")) {
				this.tab = Tab.SETTINGS;
				hideObjSpecificElements();
				
			} else if (theControlEvent.getName().equals("joint properties")) {
				this.tab = Tab.JOINTS;
				showObjSpecificElements();
				
				// deactivate fields that are not available for the current object type
				showJointSpecificFormFields();

			}
			
			
		} else if (theControlEvent.isGroup()) {
			
			// handle listbox events
			if(theControlEvent.getGroup().getId()==42) {
				t_types.getCaptionLabel().set( objClasses.get( (int) theControlEvent.getGroup().getValue()) );
				
			}
		}
		redraw();
	}


	public void saveMapToFile(String filename) {
		
		OWLImportExport exp = new OWLImportExport();
		
		RDFXMLOntologyFormat format = new RDFXMLOntologyFormat();
		format.setPrefix("knowrob:", "http://ias.cs.tum.edu/kb/knowrob.owl#");
		format.setPrefix("owl:", "http://www.w3.org/2002/07/owl#");
		format.setPrefix("rdfs:", "http://www.w3.org/2000/01/rdf-schema#");
		format.setPrefix("map:", t_namespace.getText());
		
		OWLOntology ont = exp.createOWLMapDescription(map_namespace, map_id, new ArrayList<ObjectInstance>(this.objects.values()));
		
		OWLFileUtils.saveOntologyToFile(ont, format, filename);
	}


	private void hideObjSpecificElements() {
		

		b_new_obj.hide();
		b_del_obj.hide();
		b_update.hide();
		t_id.hide();
		t_types.hide();
		
		l_dim.hide();
		for(Textfield t : t_dim)
			t.hide();
				
		for(Button b : b_rot)
			b.hide();

	}
	
	private void showObjSpecificElements() {

		b_new_obj.show();
		b_del_obj.show();
		b_update.show();
		t_id.show();
		t_types.show();
		
		l_dim.show();
		for(Textfield t : t_dim)
			t.show();
				
		for(Button b : b_rot)
			b.show();
	}
	
	
	private void showJointSpecificFormFields() {
		// draw joint limits and child selection for all kinds of joints
		if(t_types.getCaptionLabel().getText().equals("PrismaticJoint") || 
				t_types.getCaptionLabel().getText().equals("HingedJoint")) {
			l_jointlim.show();
			for(Textfield t : t_jointlim) {
				t.show();
			}
			t_child.show();
			b_child.show();
		} else {
			l_jointlim.hide();
			for(Textfield t : t_jointlim) {
				t.hide();
			}
			t_child.hide();
			b_child.hide();
		}

		// draw direction fields only for prismatic joints
		if(t_types.getCaptionLabel().getText().equals("PrismaticJoint")) {
			l_jointdir.show();
			for(Textfield t : t_jointdir) {
				t.show();
			}
		} else {
			l_jointdir.hide();
			for(Textfield t : t_jointdir) {
				t.hide();
			}
		}


		// draw direction fields only for rotational joints
		if(t_types.getCaptionLabel().getText().equals("HingedJoint")) {
			l_radius.show();
			t_radius.show();
		} else {
			l_radius.hide();
			t_radius.hide();
		}
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
			String obj = readFormData();
			
			if(obj!=null) {
				mapVisApplet.removeObject(obj);
				mapVisApplet.addObject(obj);
			}
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
			} else if(this.tab==Tab.QUATERNION) {
				switchFocus(t_all_quat, fwd);
			} else if(this.tab==Tab.JOINTS) {
				switchFocus(t_all_joint, fwd);
			}

		}
		
		if (keyCode == KeyEvent.VK_PAGE_UP) {
			
			if(this.tab==Tab.SIMPLE) {
				incTextfieldValue(t_all_simple, 0.05f);
			} else if(this.tab==Tab.MATRIX) {
				incTextfieldValue(t_all_matrix, 0.05f);
			} else if(this.tab==Tab.QUATERNION) {
				incTextfieldValue(t_all_quat, 0.05f);
			} else if(this.tab==Tab.JOINTS) {
				incTextfieldValue(t_all_joint, 0.05f);
			}
			String obj = readFormData();
			
			if(obj!=null) {
				mapVisApplet.removeObject(obj);
				mapVisApplet.addObject(obj);
			}
		}
		
		if (keyCode == KeyEvent.VK_PAGE_DOWN) {
			
			if(this.tab==Tab.SIMPLE) {
				incTextfieldValue(t_all_simple, -0.05f);
			} else if(this.tab==Tab.MATRIX) {
				incTextfieldValue(t_all_matrix, -0.05f);
			} else if(this.tab==Tab.QUATERNION) {
				incTextfieldValue(t_all_quat, -0.05f);
			} else if(this.tab==Tab.JOINTS) {
				incTextfieldValue(t_all_joint, -0.05f);
			}
			String obj = readFormData();
			
			if(obj!=null) {
				mapVisApplet.removeObject(obj);
				mapVisApplet.addObject(obj);
			}
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
			if(t_all.get(i).isActive()){
				
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
			if(t_all.get(i).isActive()){
				
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
	 * Read information from the forms and save it in a ObjectInstance instance
	 */
	protected String readFormData() {
		
		// check if the object already exists or if we should create a new one
		
		ObjectInstance cur;
		String shortname =  t_id.getText();
		String namespace = t_namespace.getText();
		
		// skip this if the ID or class are not set
		if(t_id.equals("") || t_types.getCaptionLabel().getText().equals("object class")){
			return null;
		}
		
		if(objects.containsKey(shortname)) {
			cur = objects.get(shortname);
		} else {
			
			if(t_types.getCaptionLabel().getText().equals("HingedJoint") ||
			   t_types.getCaptionLabel().getText().equals("PrismaticJoint")) {
				
				cur = JointInstance.getMapJoint(namespace + shortname);
				
			} else {
				cur = ObjectInstance.getObjectInstance(namespace + shortname);
			}
			objects.put(shortname, cur);
		}
		
		 
		// get OWL class based on their shortname (which is used for display)
		cur.addType((OWLClass) OWLThing.getOWLThingByShortname(t_types.getCaptionLabel().getText()));

		
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
				updateQuaternionForm(cur.getPosition(), cur.getPoseQuaternion());
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
				updateQuaternionForm(cur.getPosition(), cur.getPoseQuaternion());
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
				
				cur.setPoseQuaternion(new Vector3d(x,y,z), 
								new Quat4d(qx,qy,qz,qw),1.0);
				
				updateMatrixForm(cur.getPoseMatrix());
				updatePositionForm(cur.getPosition());
				break;
				
			case JOINTS:
				
				
				if(t_types.getCaptionLabel().getText().equals("HingedJoint") ||
				   t_types.getCaptionLabel().getText().equals("PrismaticJoint")) {
					
					// set direction of prismatic joints
					try{x = Double.valueOf(t_jointdir[0].getText());} catch(NumberFormatException e) {x=0.0;}
					try{y = Double.valueOf(t_jointdir[1].getText());} catch(NumberFormatException e) {y=0.0;}
					try{z = Double.valueOf(t_jointdir[2].getText());} catch(NumberFormatException e) {z=0.0;}

					((JointInstance) cur).setDirection(new Vector3d(x,y,z));

					// set joint limits
					double q_min=0.0, q_max=0.0;
					try{q_min = Double.valueOf(t_jointlim[0].getText());} catch(NumberFormatException e) {}
					try{q_max = Double.valueOf(t_jointlim[1].getText());} catch(NumberFormatException e) {}
					
					((JointInstance) cur).setQ_min(q_min);
					((JointInstance) cur).setQ_max(q_max);
					
					double radius=0.0;
					try{
						radius = Double.valueOf(t_radius.getText());} 
					catch(NumberFormatException e) {
						e.printStackTrace();
					}
					((JointInstance) cur).setRadius(radius);
					
					
					// set child/parent fields (hinges have only one each)
					if(t_child.getText() != null) {
						if(objects.get(t_child.getText())!=null)
							((JointInstance) cur).setChild(objects.get(t_child.getText()));
					}

					if(t_parent.getText() != null) {
						if(objects.get(t_parent.getText())!=null)
							((JointInstance) cur).setParent(objects.get(t_parent.getText()));
					}

				}
				updateMatrixForm(cur.getPoseMatrix());
				updateQuaternionForm(cur.getPosition(), cur.getPoseQuaternion());
				updatePositionForm(cur.getPosition());
				break;
		}

		// set object as physicalPart of parent object
		if(t_parent.getText() != null && !t_parent.getText().isEmpty()) {
			if(objects.get(t_parent.getText())!=null) {
				objects.get(t_parent.getText()).addPhysicalPart(cur);
			}
		}
		
		// synchronize with Prolog if initialized
		if(PrologInterface.isInitialized()) {
			cur.setSaveToProlog(true);
			cur.writeToProlog();
		}
		return cur.getIRI();
	}
	
	/** 
	 * Load the information from the ObjectInstance cur into the forms
	 * 
	 * @param cur Current ObjectInstance providing the information to be filled into the forms 
	 */
	void writeFormData(ObjectInstance cur) {
		
		t_id.setText(cur.getShortName());
		
		
		// find object that has this object as physical part
		for(ObjectInstance o : objects.values()) {

			if(o.hasPhysicalPart(cur)) {
				t_parent.setText(o.getShortName());
				break;
			} else {
				t_parent.setText("");
			}
		}
		
		t_types.getCaptionLabel().set(cur.getTypes().get(0).getShortName());

		t_dim[0].setText(""+cur.getDimensions().x);
		t_dim[1].setText(""+cur.getDimensions().y);
		t_dim[2].setText(""+cur.getDimensions().z);
		
		updateQuaternionForm(cur.getPosition(), cur.getPoseQuaternion());
		updateMatrixForm(cur.getPoseMatrix());
		updatePositionForm(cur.getPosition());
		updateJointForm(cur);
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
	
	/**
	 * Update the joint information form fields (if data has been changed)
	 * 
	 * @param cur MapJoint with the joint information
	 */
	void updateJointForm(ObjectInstance cur) {
		
		showJointSpecificFormFields();
		
		if(cur instanceof JointInstance) {
			t_jointlim[0].setText(((JointInstance)cur).getQ_min()+"");
			t_jointlim[1].setText(((JointInstance)cur).getQ_max()+"");
			
			t_jointdir[0].setText(((JointInstance)cur).getDirection().x+"");
			t_jointdir[1].setText(((JointInstance)cur).getDirection().y+"");
			t_jointdir[2].setText(((JointInstance)cur).getDirection().z+"");
			
			if(((JointInstance)cur).getChild()!=null && ((JointInstance)cur).getChild().getShortName() != null)
				t_child.setText(((JointInstance)cur).getChild().getShortName());
			
			if(((JointInstance)cur).getParent()!=null && ((JointInstance)cur).getParent().getShortName() != null)
				t_parent.setText(((JointInstance)cur).getParent().getShortName());
			
			t_radius.setText(((JointInstance)cur).getRadius()+"");
		}
		
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
	 * Show dialog for selecting the input file, parse it and update the visualization
	 */
	public void selectAndLoadInputFile() {

		String filename = selectInput();
		if (filename != null) {
			loadInputFile(filename);
		}
	}
	
	/**
	 * Load an OWL file into the editor-internal data structures
	 * 
	 * @param filename OWL file to be loaded
	 */
	public void loadInputFile(String filename) {

		// clean up if re-loading a map
		for(ObjectInstance o : objects.values()) {
			OWLThing.removeOWLThing(o.getIRI());
		}
		this.objects.clear();
		
		if(map_id!=null && !map_id.equals("")) {
			PrologQueryUtils.deleteObjectInstanceWithChildren(map_namespace + map_id);
			mapVisApplet.clear();
		}
		
		// load map from OWL
		if (filename != null) {
			this.objects = OWLImportExport.readObjectInstanceFromOWL(filename);	
		}
		PrologQueryUtils.parseOwlFile(filename);
		
		String mapInstance = PrologQueryUtils.getSemanticMapInstance(null, null, null, null);		
		if(mapInstance != null) {
			mapInstance = PrologInterface.removeSingleQuotes(mapInstance);
			setMapIdentifier(OWLThing.getShortNameOfIRI(mapInstance));
			setNamespace(OWLThing.getPrefixOfIRI(mapInstance) + "#");
			
			if(t_map_id!=null) {
				t_map_id.setText(map_id);
			}
			if(t_namespace!=null) {
				t_namespace.setText(map_namespace);
			}
		}
		
		for(ObjectInstance o : objects.values()) {
			mapVisApplet.addObjectWithChildren(o.getIRI());
		}
		mapVisApplet.redraw();
	}
	

	////////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	// INTERFACE WITH PROLOG CANVAS
	//
	//
	////////////////////////////////////////////////////////////////////////////////////////////	

	
	/**
	 * Set the internal reference to the  {@link SemanticMapVisApplet}
	 * @param app Reference to a {@link SemanticMapVisApplet}
	 */
	
	public void setMapVisApplet(SemanticMapVisApplet app) {
		this.mapVisApplet = app;
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
	
	public void editObject(String identifier) {

		if(identifier==null)
			return;
		
		// special mode: select the parent object for the currently edited object
		if(selectParentObj) {
			
			ObjectInstance cur = objects.get(t_id.getText());
			
			// clear previous parent objects
			for(ObjectInstance o : objects.values()) {
				if(o.getPhysicalParts().contains(cur)) {
					o.removePhysicalPart(cur);
				}
			}
			
			objects.get(identifier).addPhysicalPart(cur);
			
			if(cur instanceof JointInstance)
				((JointInstance) cur).parent = objects.get(identifier);
				
			this.t_parent.setText(identifier);
			
			selectParentObj=false;
			cursor(ARROW);
			mapVisApplet.cursor(ARROW);
			
			
			
		} else if(selectChildObj) {
				
				ObjectInstance cur = objects.get(t_id.getText());
				
				// clear previous parent objects
				for(ObjectInstance o : objects.values()) {
					if(o.hasPhysicalPart(cur)) {
						o.removePhysicalPart(cur);
					}
				}

				if(cur instanceof JointInstance)
					((JointInstance) cur).child = objects.get(identifier);
				
				this.t_child.setText(identifier);
				
				selectChildObj=false;
				cursor(ARROW);
				mapVisApplet.cursor(ARROW);
			

				
		} else {
			// normal mode: load information about the object the user clicked on into the form fields 
			writeFormData(objects.get(OWLThing.getShortNameOfIRI(identifier)));
		}
	}


	public void setMapIdentifier(String mapID) {
		map_id = mapID;
	}

	public void setNamespace(String namespace) {
		map_namespace  = namespace;
	}
	
}
