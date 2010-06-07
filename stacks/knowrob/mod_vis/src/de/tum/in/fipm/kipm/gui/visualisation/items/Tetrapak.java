package de.tum.in.fipm.kipm.gui.visualisation.items;

import processing.core.PConstants;
import edu.tum.cs.vis.Canvas;

public class Tetrapak extends Item {

	/**
	 * initializes a STATIC Instance
	 */
	public Tetrapak(float x, float y, float z) { super(x,y,z); }

	/**
	 * initializes a STATIC Instance
	 */
	public Tetrapak(float x, float y, float z, float rotX, float rotY, float rotZ, float scale, int color){ super(x, y, z, rotX, rotY, rotZ, scale, color); }
	
	@Override
	public void drawIt(Canvas c) {
		c.pushMatrix();
		if(trafoMatrix != null) {
			c.applyMatrix(trafoMatrix[0], trafoMatrix[1], trafoMatrix[2], trafoMatrix[3], 
					trafoMatrix[4], trafoMatrix[5], trafoMatrix[6], trafoMatrix[7], 
					trafoMatrix[8], trafoMatrix[9], trafoMatrix[10], trafoMatrix[11], 
					trafoMatrix[12], trafoMatrix[13], trafoMatrix[14], trafoMatrix[15]);


    } else {
      c.translate(currentData[0], currentData[1], currentData[2]-0.125f);
      c.fill(colorOverride!=0 ? colorOverride : currentColor);
      if(currentData[6] != 1.0f) c.scale(currentData[6]);
      if(currentData[3] != 0) c.rotateX(currentData[3]);
      if(currentData[4] != 0) c.rotateY(currentData[4]);
      if(currentData[5] != 0) c.rotateZ(currentData[5]);
		}

    // translate it down by half its height
    c.translate(0.0f, 0.0f, -12.5f);

		// "wrapper"
		c.beginShape(PConstants.QUAD_STRIP);
		c.vertex(5,5,0); c.vertex(-5,5,0);
		c.vertex(5,5,20); c.vertex(-5,5,20);
		c.vertex(5,0,25); c.vertex(-5,0,25);
		c.vertex(5,-5,20); c.vertex(-5,-5,20);
		c.vertex(5,-5,0); c.vertex(-5,-5,0);
		c.endShape(PConstants.CLOSE);
		
		// front side
		c.beginShape(PConstants.TRIANGLE_STRIP);
		c.vertex(-5,5,0);
		c.vertex(-5,-5,0);
		c.vertex(-5,5,20);
		c.vertex(-5,-5,20);
		c.vertex(-5,0,25);
		c.endShape();
		
		// back side
		c.beginShape(PConstants.TRIANGLE_STRIP);
		c.vertex(5,5,0);
		c.vertex(5,-5,0);
		c.vertex(5,5,20);
		c.vertex(5,-5,20);
		c.vertex(5,0,25);
		c.endShape();
		
		c.popMatrix();
	}
}
