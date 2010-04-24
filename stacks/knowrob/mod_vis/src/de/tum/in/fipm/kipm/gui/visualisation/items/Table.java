package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;

public class Table extends Item {

	
	/**
	 * initializes a STATIC Instance
	 * normal position is 320.0, 200.33, 0.0
	 */
	public Table(float x, float y, float z) { super(x,y,z);
		this.currentColor = ItemBase.convertColor(150,150,150,255);
	}

	/**
	 * initializes a STATIC Instance
	 * normal position is 320.0, 200.33, 0.0
	 */
	public Table(float x, float y, float z, float rotX, float rotY, float rotZ, float scale, int color){ super(x, y, z, rotX, rotY, rotZ, scale, color); }
	
	
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
		

		// draw legs
		/*
		 
		c.translate(0, 0, -73.177f);	  
		c.rect(-40.24f, -59.89f, 80.46f, 119.76f);
		  
		c.translate (299.875f, 155.41f, 36.5885f);		c.box(5f, 5f, 73.177f);
		c.translate (0f, 89.82f, 0f);	  				c.box(5f, 5f, 73.177f);
		c.translate (40.23f, 0f, 0f);	  				c.box(5f, 5f, 73.177f);
		c.translate (0f, -89.82f, 0f);  				c.box(5f, 5f, 73.177f);
		*/
		
		c.translate(0, 0, 72.177f);	  
		c.box(80.46f, 119.76f, -2f);
		c.translate (20.115f, 44.91f, -36.5885f);	c.box(-5f, 5f, 73.175f);
		c.translate(-40.23f,0,0);		c.box(-5f, 5f, 73.175f);
		c.translate(0,-89.82f,0);		c.box(-5f, 5f, 73.175f);
		c.translate(40.23f,0,0);		c.box(-5f, 5f, 73.175f);

		
		c.popMatrix();
	}

}
