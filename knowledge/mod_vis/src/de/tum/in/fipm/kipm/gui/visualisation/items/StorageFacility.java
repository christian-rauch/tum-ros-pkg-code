package de.tum.in.fipm.kipm.gui.visualisation.items;

import java.util.ArrayList;
import java.util.Arrays;
import edu.tum.cs.vis.Canvas;

public abstract class StorageFacility extends ItemBase {
	

	/**
	 * handles that have to be rotated with the door.
	 */
	protected ArrayList<Handle> handles;
	
	/**
	 * 	 [0] "degree" to which the object is opened. 
	 *       0 = fully closed; 1000 = fully opened.
	 *   [1] color
	 *   
	 *   may be null if animated == false.
	 */
	private ArrayList<int[]> data;
	
	/**
	 * current settings ([0] = openess; [1] = color)
	 */
	protected int[] currentData;
	
	/**
	 * [0]  - [15] Homography-Matrix
	 * [12] - [14] x,y and z coordinate of the (3D-)Center of the Box
	 * [16] - [18] x-extent (depth), y-extend (width), z-extend (height)
	 * 
	 */
	protected float[] fixedData;
	
	
	
	
	private boolean animated;
	
	public static enum Action {OPEN, CLOSE};
	
	public StorageFacility(float m00,float m01,float m02,float m03,
			float m10,float m11,float m12,float m13, 
			float m20,float m21,float m22,float m23, 
			float m30,float m31,float m32,float m33, 
			float xdepth, float ywidth, float zheight) {
		
		this.animated = false;
		this.fixedData = new float[] {m00, m01, m02, m03, m10, m11, m12, m13, 
				 m20, m21, m22, m23, m30, m31, m32, m33, xdepth, ywidth, zheight};
		this.currentData = new int[] {0,ItemBase.convertColor(100,100,100,255)};
		this.handles = new ArrayList<Handle>();
	}


	/**
	 * adds a move (Open / Close).
	 * automatically makes the Storage Facility animated.
	 * @param a
	 * @param frame frame at which the animation STARTS
	 * @param duration how many frames the animation should take (0 -> instant)
	 */
	public void addMove(Action a, int frame, int duration) {
		addMove(a,frame,duration,new int[] {0,Integer.MAX_VALUE});
	}
	
	/**
	 * 
	 * @param frame frame
	 * @param degree 0 = closed; 1 = open
	 */
	public void setOpeness(int frame, float degree) {
		prepAnimatedTo(frame+1);
		
		int[] toAdd = new int[] {(int)(1000.0f * degree),
						((255-((int)(155*degree))) << 24) | (data.get(frame)[1] & 0x00FFFFFF)};
		
		if(frame > 0 && Arrays.equals(toAdd,data.get(frame-1)))
			data.set(frame, data.get(frame-1));
		else
			data.set(frame,toAdd);
	}
	
	public void propagate(int first, int[] constraint, float degree) {
		if(!animated) return;

		// propagate state
		for(int i=first;i<data.size();i++) {
			if(constraint[0] > i || i > constraint[1])
				continue;
			if(data.get(i)[0] == 1000 || data.get(i)[0] == 0)
				setOpeness(i,degree);
			else
				break;
		}
	}
	
	/**
	 * adds a move (Open / Close).
	 * automatically makes the Storage Facility animated.
	 * @param a
	 * @param frame frame at which the animation STARTS
	 * @param duration how many frames the animation should take (0 -> instant)
	 * @param constraint only data for frames f with constraint[0] <= f <= constraint[1] is touched.
	 */
	public void addMove(Action a, int frame, int duration, int[] constraint) {
		prepAnimatedTo(frame+duration+1);
		
		for(int i=frame;i<frame + duration; i++) {
			if(constraint[0] > i || i > constraint[1])
				continue;

			if(a == Action.OPEN)
				setOpeness(i,((float)i-frame+1)/duration);
			else
				setOpeness(i,1.0f-((float)i-frame+1)/duration);
		}
		
	//	int first = frame+duration;
	//	float state = (a == Action.OPEN ? 1 : 0);
		setOpeness(frame-duration, (a == Action.OPEN ? 1.0f : 0f));
			
	}

	/**
	 * sets the color (for the whole time)
	 * @param color
	 */
	@Override
	public void setColor(int color){
		if(!animated)
			currentData[1] = color;
		else
			setColor(color,0,data.size()-1);
	}
	
	/**
	 * changes the color for a range of frames.
	 * makes the object animated
	 * @param color color
	 * @param start first frame to change
	 * @param end last frame to change
	 */
	@Override
	public void setColor(int color, int start, int end) {
		prepAnimatedTo(end+1);
		
		for(int i=start;i<=end;i++) {

			int[] old = data.get(i);
			int[] n = new int[] {old[0],(old[1] & 0xFF000000) | (color & 0x00FFFFFF)}; // keep old alpha!
				
			if(i>0 && Arrays.equals(n,data.get(i-1)))
				data.set(i,data.get(i-1));
			else
				data.set(i,n);
			
		}
	}
	
	/**
	 * extends data up to (including) index t.
	 * @param t
	 */
	private void prepAnimatedTo(int t) {
		if(!animated) {
			data = new ArrayList<int[]>(t+1);
			data.add(currentData);
			animated = true;
		}
		int[] last = data.get(data.size()-1);
		for(int i=data.size();i<=t;i++)
			data.add(last);
	}
	
	
	@Override
	public void draw(Canvas c) {
		c.noStroke();
		drawIt(c,0);
	}

	public void draw(Canvas c, int step) {
		if(animated) { currentData = data.size() > step ? data.get(step) : data.get(data.size()-1);}
		c.noStroke();
		drawIt(c,step);
	}
	
	@Override
	public int getMaxStep() {
		return data==null ? 0 : data.size()-5;
	}	
	
	/**
	 * adds a Handle to the StorageFacility. Handle is ALWAYS moved with when the Container is opened.
	 * coordinates of the handle have to be absolute. the Handle is automatically drawn by the object.
	 * @param h Handle to add
	 */
	public void addHandle(Handle h) {
		handles.add(h);
	}
	
	protected abstract void drawIt(Canvas c, int step);
}
