package de.tum.in.fipm.kipm.gui.visualisation.items;


import edu.tum.cs.vis.Canvas;


public class Ellipse extends Item {

	/**
	 * initializes a STATIC Instance.
	 * xVar / yVar is the RADIUS.
	 */
	public Ellipse(float x, float y, float z, float xVar, float yVar, int color) { 
		super(x,y,z,xVar,yVar,0f,0f,color); 
	}
	
	@Override
	public void drawIt(Canvas c) {
		c.pushMatrix();
		if(trafoMatrix != null)
			c.applyMatrix(trafoMatrix[0], trafoMatrix[1], trafoMatrix[2], trafoMatrix[3], 
					trafoMatrix[4], trafoMatrix[5], trafoMatrix[6], trafoMatrix[7], 
					trafoMatrix[8], trafoMatrix[9], trafoMatrix[10], trafoMatrix[11], 
					trafoMatrix[12], trafoMatrix[13], trafoMatrix[14], trafoMatrix[15]);
		c.translate(0, 0, currentData[2]);
		c.fill(colorOverride!=0 ? colorOverride : currentColor);
		c.ellipse(currentData[0],currentData[1],currentData[3],currentData[4]);
	    c.popMatrix();
	}

}
