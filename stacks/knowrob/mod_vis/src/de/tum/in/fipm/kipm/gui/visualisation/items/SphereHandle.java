package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;

public class SphereHandle extends Handle {

	private float radius;
	
	public SphereHandle(float x, float y, float z, float radius) {
		super(x,y,z);
		this.radius = radius;
	}
	
	@Override
	public void draw(Canvas c) {
		c.pushMatrix();
		if(trafoMatrix != null)
			c.applyMatrix(trafoMatrix[0], trafoMatrix[1], trafoMatrix[2], trafoMatrix[3], 
					trafoMatrix[4], trafoMatrix[5], trafoMatrix[6], trafoMatrix[7], 
					trafoMatrix[8], trafoMatrix[9], trafoMatrix[10], trafoMatrix[11], 
					trafoMatrix[12], trafoMatrix[13], trafoMatrix[14], trafoMatrix[15]);
		c.noStroke();
		c.translate(coordinates[0], coordinates[1], coordinates[2]);
		c.fill(colorOverride!=0 ? colorOverride : currentColor);
		c.sphere(radius);
		c.popMatrix();
	}
}

