package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;

public class BoxHandle extends Handle {

	private float[] sizes;
	
	public BoxHandle(float x, float y, float z, float xdepth, float ywidth, float zheight) {
		super(x,y,z);
		sizes = new float[] {xdepth, ywidth, zheight};
	}
	
	@Override
	public void draw(Canvas c) {
		c.pushMatrix();
		c.noStroke();
		c.translate(coordinates[0], coordinates[1], coordinates[2]);
		c.fill(colorOverride!=0 ? colorOverride : currentColor);
		c.box(sizes[0], sizes[1], sizes[2]);
		c.popMatrix();
	}
}

