package edu.tum.cs.ias.knowrob.map;

import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import processing.core.*;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.vis.applets.PrologVisualizationCanvas;



/**
 * Graphical editor for the OWL semantic map format
 * 
 * The editor is based on the KitchenVisApplet visualization and extends it
 * with input forms to enter object information.
 * 
 * @author tenorth@cs.tum.edu
 *
 */
public class SemanticMapEditor extends PrologVisualizationCanvas implements MouseListener, MouseMotionListener {

	private static final long serialVersionUID = 4575739930038583994L;
	protected SemanticMapEditorForms forms_applet;
	
	
	/**
	 * Constructor: Create applets for the kitchen visualization 
	 * and for entering the object information
	 */
	public SemanticMapEditor() {

		this.setSize(1050, 600);
		this.remove(AVObject);
		//this.controlWindow.hide();
		this.KVObject.setViewParameters(4.0f, 4.0f, 134f, -15f, 100);
		
		PrologInterface.initJPLProlog("mod_vis");
		
		forms_applet = new SemanticMapEditorForms();
		forms_applet.frame = this.frame;
		forms_applet.setSize(390, 600);
		forms_applet.init();
		forms_applet.setMapVisApplet(KVObject);
		
		this.add(forms_applet);

		this.cursor(ARROW);
		this.forms_applet.cursor(ARROW);
		this.KVObject.cursor(ARROW);
		

		background(40);
		this.draw();
		this.setVisible(true);
		this.setSize(1050, 600);
		this.validate();
		

		forms_applet.selectAndLoadInputFile();
	
	}
	
	public void draw() {
		background(40);
	}

	/**
	 * Override generic method to get information about the objects the user has clicked on
	 * 
	 * @param identifier
	 */
    public void mapObjectClicked(String identifier) {

    	forms_applet.editObject(identifier);
    }
    		
	public static void main(String args[]) {
		PApplet.main(new String[] { "edu.tum.cs.ias.knowrob.map.SemanticMapEditor" });
	}
}

