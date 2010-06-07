package de.tum.in.fipm.kipm.gui.visualisation.items;

import java.util.ArrayList;
import java.util.Arrays;
import edu.tum.cs.vis.Canvas;

/**
 * TODO This class should probably be rewritten to be based on edu.tum.cs.vis.items.GenericItem (sooner or later), since that class handles transformations applied to the renderer that may change over time much more generically  
 */
public abstract class Item extends ItemBase {
	
	/**
	 * List of arrays with the following entries:
	 * [0] - [2] x,y and z coordinate of the Bottom-Center of the Object
	 * [3] - [5] rotation around the x,y and z axis
	 * [6] - scale (default: 1)
	 * may be null if animated == false.	 
	 */
	private ArrayList<float[]> data;
	private ArrayList<Integer> color;	
	
	
	protected float[] currentData;
	protected Integer currentColor;
	private boolean animated;
	
	
	/**
	 * initializes a STATIC Instance
	 * TODO What is meant here by "STATIC"? Non-animated? And how is a non-static instance initialized?
	 */
	public Item(float x, float y, float z) {
		this.animated = false;
		this.currentData = new float[] {x,y,z,0,0,0,1};
		this.currentColor = Item.convertColor(220,220,220,255);
		this.defaultColor = currentColor;
	}

	/**
	 * initializes a STATIC Instance
	 */
	public Item(float x, float y, float z, float rotX, float rotY, float rotZ, float scale, int color){
		this.animated = false;
		this.currentData = new float[] {x,y,z,rotX,rotY,rotZ,scale};
		this.currentColor = color;
		this.defaultColor = currentColor;
	}

	/**
	 * sets the position for ONE frame. leaves rotation, scale, color unchanged.
	 */
	public void setFrame(int frame, float x, float y, float z) {
		setFrames(frame,frame,x,y,z);
	}
	
	/**
	 * sets the position / rotation / scale for ONE frame.
	 */
	public void setFrame(int frame, float x, float y, float z, float rotX, float rotY, float rotZ, float scale) {
		setFrames(frame,frame,x,y,z,rotX,rotY,rotZ,scale);
		
	}
	
	/**
	 * sets the position for SEVERAL frames. leaves rotation, scale, color unchanged;
	 * @param lastFrame last frame to be changed; -1 for "until the end".
	 */
	public void setFrames(int firstFrame, int lastFrame, float x, float y, float z) {
		if(lastFrame == -1) {
			lastFrame = Math.max(firstFrame, data.size()-1);
			animated(lastFrame);
		}
		else
			animated(lastFrame+1); // TODO why +1?
		
		for(int i=firstFrame; i<= lastFrame; i++) {
			float[] old = data.get(i);
			float[] nw = new float[] {x,y,z,old[3],old[4],old[5],old[6]};
			if(i>0 && Arrays.equals(nw,data.get(i-1)))
				data.set(i,data.get(i-1));
			else
				data.set(i,nw);
		}
	}
	
	/**
	 * sets the position / rotation / scale for SEVERAL frames
	 */
	public void setFrames(int firstFrame, int lastFrame, float x, float y, float z, float rotX, float rotY, float rotZ, float scale) {
		animated(lastFrame+1);
		float[] nw = new float[] {x,y,z,rotX,rotY,rotZ,scale};
		for(int i=firstFrame; i<= lastFrame; i++)
			data.set(i,nw);
	}
	
	private void animated(int upTo) {
		animated = true;
		if(data == null) {
			data = new ArrayList<float[]>();
			data.add(currentData);
		}
		if(color == null) {
			color = new ArrayList<Integer>();
			color.add(currentColor);
		}
		
		for(int i=data.size();i<=upTo;i++)
			cloneEntry();
	}
	 
	
	/**
	 * Duplicates the last frame (BY REFERENCE!)
	 */
	private void cloneEntry() {
		data.add(data.get(data.size()-1));
		color.add(color.get(color.size()-1));
	}
	
	/*
	 * (non-Javadoc)
	 * @see de.tum.in.fipm.kipm.gui.visualisation.items.ItemBase#setColor(int)
	 */
	@Override
	public void setColor(int color){
		if(!animated) this.currentColor = color;
		else {
			this.currentColor = color;
			for(int i=0;i<this.color.size(); i++)
				this.color.set(i,color);
		}
	}
	
	/*
	 * (non-Javadoc)
	 * @see de.tum.in.fipm.kipm.gui.visualisation.items.ItemBase#setColor(int, int, int)
	 */
	@Override
	public void setColor(int color,int start, int end){
		animated(end+1); // TODO why +1?
		for(int i=start;i<=end;i++)
			this.color.set(i,color);
	}
	@Override
	public void draw(Canvas c) {
		if(animated) { currentData = data.get(0); currentColor = color.get(0);}
		c.noStroke();
		if(currentColor != 0) drawIt(c);
	}
	
	public void draw(Canvas c, int step) {
		if(animated) { 
			currentData = data.get(Math.min(step,data.size()-1)); 
			currentColor = color.get(Math.min(step,color.size()-1));
		}
		c.noStroke();
		// TODO Why isn't translation, scaling and application of the transformation matrix handled here (as well as in draw)? It seems odd to have these members here and not use them. Instead they are applied in every single subclass (in precisely the same way as far as I could tell).  
		if(currentColor != 0) drawIt(c);
	}
	
	@Override
	public int getMaxStep() {
		return data==null ? 0 : data.size()-5; // TODO why -5 not -1?
	}	
	
	protected abstract void drawIt(Canvas c);
}
