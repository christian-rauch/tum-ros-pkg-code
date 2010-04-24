package de.tum.in.fipm.kipm.gui.visualisation.items;


import edu.tum.cs.vis.Canvas;

public class Drawer extends StorageFacility {

	
	public Drawer(float m00,float m01,float m02,float m03,
			float m10,float m11,float m12,float m13, 
			float m20,float m21,float m22,float m23, 
			float m30,float m31,float m32,float m33, 
			float xdepth, float ywidth, float zheight) {

		super( m00, m01, m02, m03, m10, m11, m12, m13, 
			   m20, m21, m22, m23, m30, m31, m32, m33, 
		       xdepth,  ywidth,  zheight);
	}
	
	
	@Override 
	public void addHandle(Handle h) {
		// change coordinates of handle relative to box center.
		float newX = h.coordinates[0]-fixedData[0];
		float newY = h.coordinates[1]-fixedData[1];
		float newZ = h.coordinates[2]-fixedData[2];
		
		h.coordinates = new float[] {newX, newY, newZ};
		super.addHandle(h);
	}
	
	
	@Override
	public void drawIt(Canvas c, int step) {
		
		c.pushMatrix();
		
			c.applyMatrix(fixedData[0],  fixedData[1],  fixedData[2],  fixedData[3],
						  fixedData[4],  fixedData[5],  fixedData[6],  fixedData[7], 
						  fixedData[8],  fixedData[9],  fixedData[10], fixedData[11], 
						  fixedData[12], fixedData[13], fixedData[14], fixedData[15]);
	
			c.fill(colorOverride!=0 ? colorOverride : (currentData[1] | 0xFF000000));
			
			// hack: display problems, therefore negative sign
			c.box(fixedData[16], fixedData[17], -fixedData[18]); 	// draw it!

	
			if(currentData[0] == 0) {		// object is closed - only draw handles
				c.popMatrix();
				for(int i=0;i<handles.size();i++)
					handles.get(i).draw(c,step);

			} else {						// object is opened - draw handles and inner

				
				for(int i=0;i<handles.size();i++)
					handles.get(i).draw(c,step);
				
				c.translate((float)(currentData[0] * 0.6*fixedData[16] / 1000.0f),0f,0f);
				c.translate(0.05f*fixedData[16],0,0);
				c.fill(colorOverride!=0 ? colorOverride : currentData[1]);
				c.box(fixedData[16]*0.9f,-fixedData[17]*0.9f,fixedData[18]*0.9f);
				c.popMatrix();
			}
		
	}
}
