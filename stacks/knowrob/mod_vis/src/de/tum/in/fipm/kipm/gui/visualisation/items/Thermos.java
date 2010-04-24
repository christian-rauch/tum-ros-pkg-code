package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;

public class Thermos extends Item {

	/**
	 * initializes a STATIC Instance
	 */
	public Thermos(float x, float y, float z) { super(x,y,z); }

	/**
	 * initializes a STATIC Instance
	 */
	public Thermos(float x, float y, float z, float rotX, float rotY, float rotZ, float scale, int color){ super(x, y, z, rotX, rotY, rotZ, scale, color); }
	

	@Override
	public void drawIt(Canvas c) {
		c.pushMatrix();
		if(trafoMatrix != null)
			c.applyMatrix(trafoMatrix[0], trafoMatrix[1], trafoMatrix[2], trafoMatrix[3], 
					trafoMatrix[4], trafoMatrix[5], trafoMatrix[6], trafoMatrix[7], 
					trafoMatrix[8], trafoMatrix[9], trafoMatrix[10], trafoMatrix[11], 
					trafoMatrix[12], trafoMatrix[13], trafoMatrix[14], trafoMatrix[15]);
		
		c.translate(currentData[0], currentData[1], currentData[2]);
		

		c.fill(colorOverride!=0 ? colorOverride : currentColor);
		if(currentData[6] != 1.0f) c.scale(currentData[6]);
		if(currentData[3] != 0) c.rotateX(currentData[3]);
		if(currentData[4] != 0) c.rotateY(currentData[4]); 
		if(currentData[5] != 0) c.rotateZ(currentData[5]);
		
		// RSS: use box instead of jug for coffee
		c.translate(0, 0, 10);
		c.box(-10f, 6f, 20f);
		
		
//		(new Cylinder(new Vector3f(0f, 0f, 0f), new Vector3f(0f, 0f, 27f), 7f)).draw(c); // body
//		(new Cylinder(new Vector3f(7f, 1f, 15f), new Vector3f(7f, -1f, 15f),7f)).draw(c); // handle
//		
		c.popMatrix();
	}
}
