package de.tum.in.fipm.kipm.gui.visualisation.items;

import de.tum.in.fipm.kipm.gui.visualisation.applets.StandaloneKitchenVisApplet;
import edu.tum.cs.vis.Canvas;
import edu.tum.cs.vis.Drawable;
import edu.tum.cs.vis.DrawableAnimated;

/**
 * Abstract base class for items (animated or not) that have a default color and may have
 * a general transformation matrix applied to them   
 * 
 */
public abstract class ItemBase implements Drawable, DrawableAnimated {
	public String name;
	/**
	 * FIXME It is bad practice to use a static method of a class that is completely unrelated to this class. It destroys modularity. The static method convertColor should probably be moved here or some place else that is neutral.
	 */
	public int defaultColor = StandaloneKitchenVisApplet.convertColor(100, 100, 100, 255);
	public int colorOverride = 0;
	
	/**
	 * a transformation matrix to apply before drawing
	 * TODO should describe ordering (rows, columns).
	 */
	protected float[] trafoMatrix = null;
	
	/**
	 * sets the color (for the whole time); 
	 * if color is 0, the item is not drawn at all!; 
	 * @param color
	 */
	public abstract void setColor(int color);

	/**
	 * changes the color for a range of frames; 
	 * makes the object animated; 
	 * in frames where the color is 0, the item is not drawn at all; 
	 * @param color color
	 * @param start first frame to change
	 * @param end last frame to change
	 */
	public abstract void setColor(int color, int start, int end);
	
	public void setTrafoMatrix(float[] m) {
		trafoMatrix = m;
	}
	
	public void draw(Canvas c) {
		draw(c,0);
	}
	
	public int getMaxStep() {
		return -1;
	}		
	public static int convertColor(int red, int green, int blue, int alpha) {
		return (((((alpha << 8) + red) << 8) + green) << 8) + blue;
	}
	

}
