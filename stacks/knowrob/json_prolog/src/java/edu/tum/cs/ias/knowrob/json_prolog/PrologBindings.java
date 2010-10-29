package edu.tum.cs.ias.knowrob.json_prolog;

import java.util.HashMap;
import java.util.Vector;

import jpl.Term;

import edu.tum.cs.ias.knowrob.json_prolog.JSONQuery.InvalidJSONQuery;

import net.sf.json.JSONArray;
import net.sf.json.JSONObject;
import net.sf.json.util.JSONUtils;


/**
 * Representation of a set of Prolog variable bindings 
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */
public class PrologBindings {

	
	private HashMap<String, PrologValue> bdgs_;
	

	public PrologBindings() {
		bdgs_ = new HashMap<String, PrologValue>();
	}
	
	/**
	 * Convert a JSON-encoded string into a PrologBindings object
	 * 
	 * @param json_bdgs JSON encoded string
	 * @return The corresponding Prolog bindings
	 */
	static public PrologBindings parseJSONBindings(String json_bdgs) {
		
		PrologBindings res = new PrologBindings();		
		if(json_bdgs!=null) {

			JSONObject json_obj = JSONObject.fromObject(json_bdgs);
			for(Object k : json_obj.keySet()) {
				
				Object bdg = json_obj.get(k.toString());
				
				try {

					PrologValue r = new PrologValue();

					if(JSONUtils.isArray(bdg)) {
						
						Term[] objs = JSONQuery.decodeJSONArray(JSONArray.fromObject(bdg));
						
						Vector<PrologValue> values = new Vector<PrologValue>(); 
						for(Term o : objs) {
							values.add(new PrologValue(o.toString()));
						}
						r = new PrologValue(values);

					} else if(JSONUtils.isObject(bdg)) {
						Term obj = JSONQuery.decodeJSONValue(JSONObject.fromObject(bdg));
						r = new PrologValue(obj.toString());
						
					} else if(JSONUtils.isString(bdg)) {
						r = new PrologValue(bdg.toString());
						
					} else if(JSONUtils.isNumber(bdg)) {
						r = new PrologValue(Double.valueOf(bdg.toString()));
						
					}
					res.bdgs_.put(k.toString(), r);	
					
				} catch (InvalidJSONQuery e) {
					e.printStackTrace();
				}
					
			}
			
		}
		return res;
	}


	public HashMap<String, PrologValue> getBdgs_() {
		return bdgs_;
	}


	public void setBdgs_(HashMap<String, PrologValue> bdgs) {
		bdgs_ = bdgs;
	}


	class VariableUnbound extends Exception {

		private static final long serialVersionUID = -5329179598962011281L;

		public VariableUnbound(String var_name)	{
			super(var_name);
		}
	};


	class JSONParseError extends Exception {

		private static final long serialVersionUID = 6704618653787680364L;

		public JSONParseError(String msg) {
			super(msg);
		}
	};

	
}
