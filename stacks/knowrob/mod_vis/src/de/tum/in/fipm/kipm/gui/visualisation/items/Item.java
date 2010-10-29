package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.vis.Canvas;

/**
 * TODO This class should probably be rewritten to be based on edu.tum.cs.vis.items.GenericItem (sooner or later), since that class handles transformations applied to the renderer that may change over time much more generically  
 */
public abstract class Item extends ItemBase {
	
	
	public Item(float m00,float m01,float m02,float m03,
			float m10,float m11,float m12,float m13, 
			float m20,float m21,float m22,float m23, 
			float m30,float m31,float m32,float m33, 
			float xdim, float ydim, float zdim){
	
		this.trafoMatrix = new float[] {
			m00, m01, m02, m03,
			m10, m11, m12, m13, 
			m20, m21, m22, m23, 
			m30, m31, m32, m33	};
		this.xdim=xdim;
		this.ydim=ydim;
		this.zdim=zdim;
	}
	
	
	public Item(float m00,float m01,float m02,float m03,
				float m10,float m11,float m12,float m13, 
				float m20,float m21,float m22,float m23, 
				float m30,float m31,float m32,float m33, 
				float xdim, float ydim, float zdim, int color){
		
		this( m00,   m01,   m02,  m03,
			  m10,   m11,   m12,  m13, 
			  m20,   m21,   m22,  m23, 
			  m30,   m31,   m32,  m33, 
			  xdim,  ydim,  zdim);

		this.color=color;
	}

	
	
	@Override
	public void draw(Canvas c) {

		c.noStroke();		
		c.pushMatrix();
		
			if(trafoMatrix != null)
				c.applyMatrix(trafoMatrix[0],  trafoMatrix[1],  trafoMatrix[2],  trafoMatrix[3], 
							  trafoMatrix[4],  trafoMatrix[5],  trafoMatrix[6],  trafoMatrix[7], 
							  trafoMatrix[8],  trafoMatrix[9],  trafoMatrix[10], trafoMatrix[11], 
							  trafoMatrix[12], trafoMatrix[13], trafoMatrix[14], trafoMatrix[15]);
			c.fill(colorOverride!=0 ? colorOverride : color);
			drawIt(c);
			
		c.popMatrix();
	}
	
	
	public void draw(Canvas c, int step) {
		draw(c);
	}
	
	
	protected abstract void drawIt(Canvas c);
}
