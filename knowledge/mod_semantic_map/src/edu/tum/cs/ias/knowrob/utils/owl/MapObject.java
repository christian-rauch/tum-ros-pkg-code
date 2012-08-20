package edu.tum.cs.ias.knowrob.utils.owl;

import java.util.ArrayList;
import java.util.TreeSet;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;


/**
 * Internal storage class for an object instance in the semantic map
 * 
 * @author tenorth@cs.tum.edu
 *
 */
public class MapObject implements Comparable<MapObject>{
	
	/**
	 * Identifier of the object, should be equal to the part 
	 * of the OWL IRI after the Hash sign (e.g. Cup21)
	 */
	public String id;
	
	/**
	 * List of strings representing the object types (OWL classes)
	 */
	public ArrayList<String> types;

	/**
	 * Dimensions of the bounding box of the object
	 */
	public Vector3d dimensions;
	
	/**
	 * Homography matrix describing the position and orientation
	 */
	public Matrix4d pose_matrix;
	
	/**
	 * Set of direct child objects (physical decomposition)
	 */
	public TreeSet<MapObject> physicalParts;	
	

	public MapObject () {
		
		this.types = new ArrayList<String>();
		
		this.dimensions = new Vector3d(); 
		this.pose_matrix = new Matrix4d();
		this.pose_matrix.setIdentity();
		
		this.physicalParts = new TreeSet<MapObject>();
	}


	public String getId() {
		return id;
	}


	public void setId(String id) {
		this.id = id;
	}


	public ArrayList<String> getTypes() {
		return types;
	}


	public void setTypes(ArrayList<String> types) {
		this.types = types;
	}


	public Vector3d getDimensions() {
		return dimensions;
	}


	public void setDimensions(Vector3d dimensions) {
		this.dimensions = dimensions;
	}


	public Matrix4d getPoseMatrix() {
		return pose_matrix;
	}


	public void setPoseMatrix(Matrix4d poseMatrix) {
		pose_matrix = poseMatrix;
	}
	
	/**
	 * Interface for setting/getting only translation
	 * @return position vector
	 */
	public Vector3d getPosition() {
		return new Vector3d(pose_matrix.m03, 
							pose_matrix.m13, 
							pose_matrix.m23);
	}


	/**
	 * Interface for setting/getting only translation. Pose matrix will be reset to identity!
	 */
	public void setPosition(Vector3d position) {
//		pose_matrix.setIdentity();
		pose_matrix.setM03(position.x);
		pose_matrix.setM13(position.y);
		pose_matrix.setM23(position.z);
	}

	
	
	// interface for setting/getting quaternion pose
	public Quat4d getPoseQuat() {
		Quat4d res = new Quat4d();
		res.set(this.pose_matrix);
		return res;
	}

	public void setPoseQuat(Vector3d translation, Quat4d orientation, double scale) {
		pose_matrix = new Matrix4d(orientation, translation, scale);
	}


	@Override
	public int compareTo(MapObject o) {
		return (this.id.compareTo(o.id));
	}
	
}
