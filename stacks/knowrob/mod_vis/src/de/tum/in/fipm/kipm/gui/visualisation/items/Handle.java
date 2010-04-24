package de.tum.in.fipm.kipm.gui.visualisation.items;

import java.util.ArrayList;
import edu.tum.cs.vis.Canvas;

public abstract class Handle extends ItemBase {

	private boolean animated;
	protected int currentColor;
	private ArrayList<Integer> color;
	protected float[] coordinates;
	
	
	@Override
	public abstract void draw(Canvas c);
	
	public Handle(float x, float y, float z) {
		coordinates = new float[] {x,y,z};
		currentColor = ItemBase.convertColor(120,120,120,255);
	}
	
	
	@Override
	public void setColor(int color) {
		
		// as the color is the only thing that changes, this makes the Handle unchanging.
		animated = false;
		currentColor = color;
		this.color = null;
	}
	
	@Override
	public void setColor(int color, int start, int end) {
		prepAnimatedTo(end+1);
		for(int i=start;i<=end;i++)
			this.color.set(i,color);
	}
	
	public void draw(Canvas c, int step) {
		if(animated) 
			currentColor = color.size() > step ? color.get(step) : color.get(color.size() - 1);
		
		draw(c);
	}
	
	@Override
	public int getMaxStep() {
		return color==null ? 0 : color.size();
	}		
	
	/**
	 * extends data up to (including) index t.
	 * @param t
	 */
	private void prepAnimatedTo(int t) {
		if(!animated) {
			color = new ArrayList<Integer>(t+1);
			color.add(currentColor);
			animated = true;
		}
		Integer last = color.get(color.size()-1);
		for(int i=color.size();i<=t;i++)
			color.add(last);
	}
}
