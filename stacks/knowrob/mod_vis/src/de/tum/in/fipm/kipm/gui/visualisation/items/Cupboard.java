package de.tum.in.fipm.kipm.gui.visualisation.items;


import processing.core.PConstants;
import edu.tum.cs.vis.Canvas;

public class Cupboard extends StorageFacility {

	public Cupboard(float m00,float m01,float m02,float m03,
					float m10,float m11,float m12,float m13, 
					float m20,float m21,float m22,float m23, 
					float m30,float m31,float m32,float m33, 
			float xdepth, float ywidth, float zheight) {
		
		super( m00, m01, m02, m03, m10, m11, m12, m13, 
			   m20, m21, m22, m23, m30, m31, m32, m33, 
			   xdepth, ywidth, zheight);
	}
	
	@Override 
	public void addHandle(Handle h) {
		
		// change coordinates of handle relative to hinges.
//		float newX = h.coordinates[0]-fixedData[3];
//		float newY = h.coordinates[1]-fixedData[7];
//		float newZ = h.coordinates[2]-fixedData[11];
		
		// this rotations compensate for the rotation done later when drawn
		// rotate X
//		float nX = newX;	
//		float nY = (float)(Math.cos(-fixedData[6])*newY + Math.sin(-fixedData[6])*newZ);
//		float nZ = (float)(Math.cos(-fixedData[6])*newZ - Math.sin(-fixedData[6])*newY);
	
		// rotateY
//		newX = (float)(Math.cos(-fixedData[7])*nX - Math.sin(-fixedData[7])*nZ);
//		newY = nY;
//		newZ = (float)(Math.cos(-fixedData[7])*nZ + Math.sin(-fixedData[7])*nX);
			
		// rotateZ
//		nX = (float)(Math.cos(fixedData[8])*newX + Math.sin(fixedData[8])*newY);
//		nY = (float)(Math.cos(fixedData[8])*newY - Math.sin(fixedData[8])*newX);
//		nZ = newZ;
		
		

//		newX = nX - fixedData[16]/2;
//		newY = nY + fixedData[17]/2;
//		newZ = nZ;	
		
//		h.coordinates = new float[] {newX, newY, newZ};
		super.addHandle(h);
	}
	
	
	@Override
	public void drawIt(Canvas c, int step) {
		
		c.pushMatrix();

		c.applyMatrix(fixedData[0],  fixedData[1],  fixedData[2],  fixedData[3],
					  fixedData[4],  fixedData[5],  fixedData[6],  fixedData[7], 
					  fixedData[8],  fixedData[9],  fixedData[10], fixedData[11], 
					  fixedData[12], fixedData[13], fixedData[14], fixedData[15]);

		c.fill(colorOverride!=0 ? colorOverride : currentData[1]);
		
		// hack: display problems, therefore negative sign
		c.box(fixedData[16], fixedData[17], -fixedData[18]); 	// draw it!

		
		// draw handles & door (only if opened):

		if(currentData[0] == 0) {		// object is closed - only draw handles
			c.popMatrix();
			for(int i=0;i<handles.size();i++)
				handles.get(i).draw(c,step);
			
		} else {						// object is opened - draw handles and door
			
			c.translate(fixedData[16]/2, -fixedData[17]/2, 0.0f); 		// translate to hinges
			
			c.rotateZ(-(float)currentData[0] * PConstants.HALF_PI / 1000.0f);
			for(int i=0;i<handles.size();i++)
				handles.get(i).draw(c,step);
			c.translate(0,fixedData[17]/2,0);
			
			c.fill(colorOverride>=0 ? colorOverride : currentData[1]);
			c.box(-5,fixedData[17],fixedData[18]);
			c.popMatrix();
		}
		
		
		
	}
}
