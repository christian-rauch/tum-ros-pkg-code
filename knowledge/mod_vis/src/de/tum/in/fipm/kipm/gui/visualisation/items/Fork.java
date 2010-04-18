package de.tum.in.fipm.kipm.gui.visualisation.items;

import processing.core.PConstants;
import edu.tum.cs.vis.Canvas;
import edu.tum.cs.util.math.Vector3f;
import edu.tum.cs.vis.items.Cylinder;

public class Fork extends Item {

	/**
	 * initializes a STATIC Instance
	 */
	public Fork(float x, float y, float z) { super(x,y,z); }

	/**
	 * initializes a STATIC Instance
	 */
	public Fork(float x, float y, float z, float rotX, float rotY, float rotZ, float scale, int color){ super(x, y, z, rotX, rotY, rotZ, scale, color); }
	

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
		

		(new Cylinder(new Vector3f(-7.8f, 0f, -1.04f), new Vector3f(1.69f, 0f, -1.04f), 1.04f)).draw(c);  // main spree
		  
		c.beginShape(PConstants.QUAD_STRIP);
		c.vertex(1.3f,1.04f,-0.39f); c.vertex(1.3f,1.04f,0.39f);
		c.vertex(2.6f,1.04f,-0.39f); c.vertex(2.6f,1.04f,0.39f);
		c.vertex(2.6f,2.34f,-0.39f); c.vertex(2.6f,2.34f,0.39f);
		c.vertex(9.1f,2.34f,-0.39f); c.vertex(9.1f,2.34f,0.39f);
		c.vertex(9.1f,1.56f,-0.39f); c.vertex(9.1f,1.56f,0.39f);
		c.vertex(3.9f,1.56f,-0.39f); c.vertex(3.9f,1.56f,0.39f);
		c.vertex(3.9f,0.52f,-0.39f); c.vertex(3.9f,0.52f,0.39f);
		c.vertex(9.1f,0.52f,-0.39f); c.vertex(9.1f,0.52f,0.39f);
		c.vertex(9.1f,-0.52f,-0.39f); c.vertex(9.1f,-0.52f,0.39f);
		c.vertex(3.9f,-0.52f,-0.39f); c.vertex(3.9f,-0.52f,0.39f);
		c.vertex(3.9f,-1.56f,-0.39f); c.vertex(3.9f,-1.56f,0.39f);
		c.vertex(9.1f,-1.56f,-0.39f); c.vertex(9.1f,-1.56f,0.39f);
		c.vertex(9.1f,-2.34f,-0.39f); c.vertex(9.1f,-2.34f,0.39f);
		c.vertex(2.6f,-2.34f,-0.39f); c.vertex(2.6f,-2.34f,0.39f);
		c.vertex(2.6f,-1.04f,-0.39f); c.vertex(2.6f,-1.04f,0.39f);
		c.vertex(1.3f,-1.04f,-0.39f); c.vertex(1.3f,-1.04f,0.39f);
		c.endShape(PConstants.CLOSE);
		
		c.beginShape();
		c.vertex(1.3f,1.04f,-0.39f);
		c.vertex(2.6f,1.04f,-0.39f);
		c.vertex(2.6f,2.34f,-0.39f);
		c.vertex(9.1f,2.34f,-0.39f);
		c.vertex(9.1f,1.56f,-0.39f);
		c.vertex(3.9f,1.56f,-0.39f);
		c.vertex(3.9f,0.52f,-0.39f);
		c.vertex(9.1f,0.52f,-0.39f);
		c.vertex(9.1f,-0.52f,-0.39f);
		c.vertex(3.9f,-0.52f,-0.39f);
		c.vertex(3.9f,-1.56f,-0.39f);
		c.vertex(9.1f,-1.56f,-0.39f);
		c.vertex(9.1f,-2.34f,-0.39f);
		c.vertex(2.6f,-2.34f,-0.39f);
		c.vertex(2.6f,-1.04f,-0.39f);
		c.vertex(1.3f,-1.04f,-0.39f);
		c.endShape(PConstants.CLOSE);
		
		c.beginShape();
		c.vertex(1.3f,1.04f,0.39f);
		c.vertex(2.6f,1.04f,0.39f);
		c.vertex(2.6f,2.34f,0.39f);
		c.vertex(9.1f,2.34f,0.39f);
		c.vertex(9.1f,1.56f,0.39f);
		c.vertex(3.9f,1.56f,0.39f);
		c.vertex(3.9f,0.52f,0.39f);
		c.vertex(9.1f,0.52f,0.39f);
		c.vertex(9.1f,-0.52f,0.39f);
		c.vertex(3.9f,-0.52f,0.39f);
		c.vertex(3.9f,-1.56f,0.39f);
		c.vertex(9.1f,-1.56f,0.39f);
		c.vertex(9.1f,-2.34f,0.39f);
		c.vertex(2.6f,-2.34f,0.39f);
		c.vertex(2.6f,-1.04f,0.39f);
		c.vertex(1.3f,-1.04f,0.39f);
		c.endShape(PConstants.CLOSE);	

		c.popMatrix();
	}

}
