package edu.tum.cs.ias.knowrob.json_prolog;

import java.util.Vector;


public class PrologValue {

	public enum value_type { DOUBLE, INT, STRING, LIST, TERM, EMPTY } ;
	
	private Object value_;
	private value_type type_;

		
	public PrologValue() {
		type_=value_type.EMPTY;
	}
	
	    
	public PrologValue(double value) {
		value_ = value;
		type_  = value_type.DOUBLE;
	}
	
	public PrologValue(int value) {
			value_ = value;
			type_  = value_type.INT;
	  }

	public PrologValue(long value) {
			value_ = value;
			type_  = value_type.INT;
	  }
	  
	public PrologValue(String value) {
			value_ = value;
			type_  = value_type.STRING;		  
	  }

	public PrologValue(PrologTerm value) {
			value_ = value;
			type_  = value_type.TERM;
	  }

	public PrologValue(Vector<PrologValue> value) {
		  value_ = value;
		  type_ = value_type.LIST;
	  }
	  
	public value_type type() { return type_; }
	  public  boolean isDouble() { return type_ == value_type.DOUBLE; }
	  public  boolean isInt()    { return type_ == value_type.INT; }
	  public  boolean isString() { return type_ == value_type.STRING; }
	  public  boolean isTerm()   { return type_ == value_type.TERM; }
	  public  boolean islist()   { return type_ == value_type.LIST; }
	  public  boolean isValid()  { return type_ != value_type.EMPTY; }

	
	@Override
	@SuppressWarnings("unchecked")
	public String toString() {
		  
		  switch(type_) {
		    case DOUBLE:
		    case INT:
		    case STRING:
		    case TERM:
		      return value_.toString();
		      

		    case LIST: {
		    	
		    	String res = "[";
		    	Vector<PrologValue> vals =  (Vector<PrologValue>) value_;
		    	
		    	for(int i=0;i<vals.size();i++) {
		    		
		    		res+=vals.get(i).toString();
		    		
		    		if(i<vals.size()-1)
		    			res+=", ";
		    	}
		    	return res + "]";
		    }
		    default:
		      return "";
		  }
		  
	  }
	  
	public Object getValue() {
		return value_;
	}
}


