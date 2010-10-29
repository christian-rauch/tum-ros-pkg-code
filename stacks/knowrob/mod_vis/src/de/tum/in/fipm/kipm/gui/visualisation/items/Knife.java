package de.tum.in.fipm.kipm.gui.visualisation.items;

import edu.tum.cs.util.math.Vector3f;
import edu.tum.cs.vis.Canvas;
import edu.tum.cs.vis.items.Cylinder;

public class Knife extends Item {


	public Knife(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float xdim,
			float ydim, float zdim) {
		
		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, xdim, ydim, zdim);
	}

	@Override
	public void drawIt(Canvas c) {
	
		(new Cylinder(new Vector3f(-7.8f, 0f, -1.04f), new Vector3f(1.3f, 0f, -1.04f), 1.04f)).draw(c); // handle
		c.translate(3.9f, 0f, -1.04f);	  
		c.box(8.45f, 2.34f, 0.52f);
		
	}

}
