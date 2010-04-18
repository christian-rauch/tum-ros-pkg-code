package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;

public class Chair extends Item {

	
	/**
	 * initializes a STATIC Instance
	 * normal position is 320.0, 200.33, 0.0
	 */
	public Chair(float x, float y, float z) { super(x,y,z); }

	/**
	 * initializes a STATIC Instance
	 * normal position is 320.0, 200.33, 0.0
	 */
	public Chair(float x, float y, float z, float rotX, float rotY, float rotZ, float scale, int color){ super(x, y, z, rotX, rotY, rotZ, scale, color); }
	
	
	@Override
	public void drawIt(Canvas c) {
		c.pushMatrix();
		c.translate(currentData[0], currentData[1], currentData[2]);
		c.fill(colorOverride!=0 ? colorOverride : currentColor);
		if(currentData[6] != 1.0f) c.scale(currentData[6]);
		if(currentData[3] != 0) c.rotateX(currentData[3]);
		if(currentData[4] != 0) c.rotateY(currentData[4]); 
		if(currentData[5] != 0) c.rotateZ(currentData[5]);
		

		// draw legs
		/*
		 
		c.translate(0, 0, -73.177f);	  
		c.rect(-40.24f, -59.89f, 80.46f, 119.76f);
		  
		c.translate (299.875f, 155.41f, 36.5885f);		c.box(5f, 5f, 73.177f);
		c.translate (0f, 89.82f, 0f);	  				c.box(5f, 5f, 73.177f);
		c.translate (40.23f, 0f, 0f);	  				c.box(5f, 5f, 73.177f);
		c.translate (0f, -89.82f, 0f);  				c.box(5f, 5f, 73.177f);
		*/
		
		c.translate(0, 0, 45f);	  
		c.box(40f, 40f, -2f);
		c.translate(20.0f, 20.0f, -20.0f);	c.box(-5f, 5f, 45f);
		c.translate(-40.0f,0,22.5f);		c.box(-5f, 5f, 90f);
		// back rest
		c.translate(0.0f,-20f,30f);c.box(-2f, 40f, 20f);
		
		c.translate(0,-20.0f,-30);			c.box(-5f, 5f, 90f);
		c.translate(40f,0,-22.5f);			c.box(-5f, 5f, 45f);

		
		
		c.popMatrix();
	}

}
