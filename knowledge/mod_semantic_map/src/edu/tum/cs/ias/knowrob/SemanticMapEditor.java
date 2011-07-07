package edu.tum.cs.ias.knowrob;

import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import processing.core.*;
import de.tum.in.fipm.kipm.gui.visualisation.base.PrologVisualizationCanvas;



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

		this.setSize(950, 600);
		this.remove(AVObject);
		this.controlWindow.hide();
		this.KVObject.setViewParameters(4.0f, 4.0f, 134f, -15f, 100);
		
		forms_applet = new SemanticMapEditorForms();
		forms_applet.setSize(240, 600);
		forms_applet.init();
		forms_applet.setPrologVisCanvas(this);
		
		this.add(forms_applet);

		this.cursor(ARROW);
		this.forms_applet.cursor(ARROW);
		this.KVObject.cursor(ARROW);
		
		this.draw();
		this.setVisible(true);
		this.setSize(950, 600);
		this.validate();
				
	}
	
	public void draw() {
		background(20, 20, 20);
	}

	/**
	 * Override generic method to get information about the objects the user has clicked on
	 * 
	 * @param identifier
	 */
    public void displayInfoFor(String identifier) {

    	forms_applet.editObject(identifier);
    }
    		
	public static void main(String args[]) {
		PApplet.main(new String[] { "edu.tum.cs.ias.knowrob.SemanticMapEditor" });
	}
}

