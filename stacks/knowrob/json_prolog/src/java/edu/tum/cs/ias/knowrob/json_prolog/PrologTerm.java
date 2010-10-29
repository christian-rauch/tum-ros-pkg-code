package edu.tum.cs.ias.knowrob.json_prolog;

import java.util.Vector;

public class PrologTerm {

	private String name_;
	private Vector<PrologValue> values_;
	

	
	public PrologTerm(String name, Vector<PrologValue> values) {
		this.name_ = name;
		this.values_ = values;
	}

	
	public String getName_() {
		return name_;
	}


	public void setName_(String name) {
		name_ = name;
	}


	public Vector<PrologValue> getValues_() {
		return values_;
	}


	public void setValues_(Vector<PrologValue> values) {
		values_ = values;
	}


	public int arity() { 
		return values_.size(); 
	}
	
}
