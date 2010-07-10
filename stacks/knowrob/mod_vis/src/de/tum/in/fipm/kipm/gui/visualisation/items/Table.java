package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;

public class Table extends Item {

	private float[] sizes;

	/**
	 * initializes a STATIC Instance
	 * normal position is 320.0, 200.33, 0.0
	 */
	public Table(float x, float y, float z) { super(x,y,z);
		this.currentColor = ItemBase.convertColor(100,100,100,255);
	}

	/**
	 * initializes a STATIC Instance
	 * normal position is 320.0, 200.33, 0.0
	 */
	public Table(float x, float y, float z, float rotX, float rotY, float rotZ, float scale, int color){ super(x, y, z, rotX, rotY, rotZ, scale, color); }


  public Table(float x, float y, float z, float width, float depth, float height) {
    super(x,y,z);
    this.currentColor = ItemBase.convertColor(100,100,100,255);
    this.sizes = new float[] {width, depth, height};
  }
	
	@Override
	public void drawIt(Canvas c) {
		c.pushMatrix();

		if(trafoMatrix != null)
			c.applyMatrix(trafoMatrix[0],  trafoMatrix[1],  trafoMatrix[2],  trafoMatrix[3], 
                    trafoMatrix[4],  trafoMatrix[5],  trafoMatrix[6],  trafoMatrix[7], 
                    trafoMatrix[8],  trafoMatrix[9],  trafoMatrix[10], trafoMatrix[11], 
                    trafoMatrix[12], trafoMatrix[13], trafoMatrix[14], trafoMatrix[15]);

		c.translate(currentData[0], currentData[1], currentData[2]);
		c.fill(colorOverride!=0 ? colorOverride : currentColor);

		if(currentData[6] != 1.0f) c.scale(currentData[6]);
// 		if(currentData[3] != 0) c.rotateX(currentData[3]);
// 		if(currentData[4] != 0) c.rotateY(currentData[4]); 
// 		if(currentData[5] != 0) c.rotateZ(currentData[5]);
		

		// draw legs
    float depth = 60f;
    float width = 120f;
    float height= 90f;

    if(sizes != null) {
      depth = sizes[0];
      width = sizes[1];
      height= sizes[2];
    } 
    float legs  = 5f;
    float ttop  = 2f;

    c.translate(0, 0, 0.5f*height);
    c.box(depth, width, -ttop);
    c.translate (0.45f*depth, 0.45f*width, -0.5f*height); c.box(-legs, legs, height);
    c.translate(-0.9f*depth,0,0);                         c.box(-legs, legs, height);
    c.translate(0,-0.9f*width,0);                         c.box(-legs, legs, height);
    c.translate(0.9f*depth,0,0);                          c.box(-legs, legs, height);

		
		c.popMatrix();
	}

}
