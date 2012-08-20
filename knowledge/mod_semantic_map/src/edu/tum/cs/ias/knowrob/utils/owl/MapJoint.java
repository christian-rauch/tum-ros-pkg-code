package edu.tum.cs.ias.knowrob.utils.owl;

import javax.vecmath.Vector3d;

public class MapJoint extends MapObject {

	// properties specific for objects that are rotational or prismatic joints
	
	/** 
	 * Only for joints: minimal configuration parameter
	 */
	public double q_min;

	/** 
	 * Only for joints: maximal configuration parameter
	 */
	public double q_max;

	/** 
	 * Radius of a rotational joint
	 */
	public double radius;
	
	/**
	 * Direction of the prismatic joint
	 */
	public Vector3d direction;

	/**
	 * Child object that this joint is connected to
	 */
	public MapObject child;

	/**
	 * Parent object that this joint is connected to
	 */
	public MapObject parent;
	

	public MapJoint () {
		super();
		this.direction = new Vector3d();
		this.child = new MapObject();
		this.parent = new MapObject();
	}
	
	public double getQ_min() {
		return q_min;
	}


	public void setQ_min(double q_min) {
		this.q_min = q_min;
	}


	public double getQ_max() {
		return q_max;
	}


	public void setQ_max(double q_max) {
		this.q_max = q_max;
	}


	public Vector3d getDirection() {
		return direction;
	}


	public void setDirection(Vector3d direction) {
		this.direction = direction;
	}


	public MapObject getChild() {
		return child;
	}


	public void setChild(MapObject child) {
		this.child = child;
	}


	public MapObject getParent() {
		return parent;
	}

	public void setParent(MapObject parent) {
		this.parent = parent;
	}

	public double getRadius() {
		return radius;
	}

	public void setRadius(double radius) {
		this.radius = radius;
	}
}
