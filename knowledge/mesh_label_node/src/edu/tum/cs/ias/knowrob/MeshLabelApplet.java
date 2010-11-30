package edu.tum.cs.ias.knowrob;

import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import controlP5.ControlP5;
import controlP5.Textfield;
import controlP5.ControlEvent;
import processing.core.*;


public class MeshLabelApplet extends PApplet  implements MouseListener, MouseMotionListener {

	private static final long serialVersionUID = 8549782324368323084L;
	
	private ObjectLabelingCanvas objLabelCanvas;
	public ControlP5 controlP5;

	public void setup() {

		size(700, 80);
		frameRate(10);

		initControlP5();
		draw();

	}

	public void draw() {
		background(20, 20, 20);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	// 
	// USER INTERFACE
	// 
	
	private void initControlP5() {

		controlP5 = new ControlP5(this);

		controlP5.addTextlabel("label","ASSIGN CLASS LABEL:",150,10);
		Textfield input = controlP5.addTextfield("", 150,25,400,20);
		input.setId(23);
		input.setColorBackground(0xFF000000);

		input.setAutoClear(true);
		input.setFocus(true);
		input.keepFocus(true);

	}


	void controlEvent(ControlEvent theEvent) {
		
		if(theEvent.controller() instanceof Textfield) {

			String label = theEvent.controller().stringValue()+"\n";
			objLabelCanvas.setLabel(label);
			objLabelCanvas.mesh_applet.clear();

			
		}
	}


	public void setObjectLabelingCanvas(ObjectLabelingCanvas parent) {
		this.objLabelCanvas = parent;
	}


	public static void main(String args[]) {
		PApplet.main(new String[] { "de.tum.in.fipm.kipm.gui.visualisation.applets.MeshLabelApplet" });
	}

	
}


