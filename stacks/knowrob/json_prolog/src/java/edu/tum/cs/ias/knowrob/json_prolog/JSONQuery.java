/* 
 * Copyright (c) 2010, Lorenz Moesenlechner
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package edu.tum.cs.ias.knowrob.json_prolog;

import net.sf.json.*;
import net.sf.json.util.JSONUtils;
import jpl.JPLException;
import jpl.Query;
import jpl.Term;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class JSONQuery {

  static public class InvalidJSONQuery extends Exception {
    public InvalidJSONQuery() {
    }

    public InvalidJSONQuery(String msg) {
      super(msg);
    }

    private static final long serialVersionUID = 1L;
  }

  static public Query makeQuery(String json) throws InvalidJSONQuery {
    try {
      JSONArray json_query = JSONArray.fromObject(json);

      return new Query(makeCompoundTerm(json_query));
    } catch (Exception e) {
      throw new InvalidJSONQuery("Unable to parse JSON query.");
    }
  }

  
  static public JSONObject encodeResult(Hashtable<String, Term> bindings) {
    JSONObject result = new JSONObject();

    Iterator<Map.Entry<String, Term>> it = bindings.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<String, Term> e = it.next();

      if (e.getValue().isFloat())
        result.put(e.getKey(), e.getValue().floatValue());
      else if (e.getValue().isInteger())
        result.put(e.getKey(), e.getValue().intValue());
      else if (e.getValue().isCompound() && !e.getValue().isAtom()) {
        if (isList(e.getValue()))
          result.put(e.getKey(), encodeList(e.getValue()));
        else
          result.put(e.getKey(), encodeCompoundTerm(e.getValue()));
      }
      else
        result.put(e.getKey(), e.getValue().name());
    }
    return result;
  }
	
  static Term makeCompoundTerm(JSONArray val) throws InvalidJSONQuery {
    if (val.isEmpty())
      throw new InvalidJSONQuery("makeCompoundTerm: empty sequence.");

    String predicate = val.getString(0);
    if (predicate == null)
      throw new InvalidJSONQuery("makeCompoundTerm: no predicate given.");

    String[] split_predicate = predicate.split(":");
    if(split_predicate.length == 1)
      return new jpl.Compound(predicate, decodeJSONArray(val, 1));
    else if(split_predicate.length == 2)
      return new jpl.Compound(":", new Term[] {
          new jpl.Atom(split_predicate[0]), 
          new jpl.Compound(split_predicate[1], decodeJSONArray(val, 1)) });
    else
      throw new InvalidJSONQuery("Predicate encoding wrong. Found more than one ':' in " + predicate);
  }

  static Term[] decodeJSONArray(JSONArray val) throws InvalidJSONQuery {
    return decodeJSONArray(val, 0);
  }

  static private Term[] decodeJSONArray(JSONArray val, int startIndex)
      throws InvalidJSONQuery {
    Term[] terms = new Term[val.size() - startIndex];

    for (int i = startIndex; i < val.size(); i++) {
      if (JSONUtils.isString(val.get(i)))
        terms[i - startIndex] = decodeJSONValue(val.getString(i));
      else if (JSONUtils.isBoolean(val.get(i)))
        terms[i - startIndex] = decodeJSONValue(val.getBoolean(i));
//      else if (JSONUtils.isDouble(val.get(i)))
//        terms[i - startIndex] = decodeJSONValue(val.getDouble(i));
      else if (JSONUtils.isNumber(val.get(i)))
        terms[i - startIndex] = decodeJSONValue(val.getInt(i));
      else if (JSONUtils.isObject(val.get(i)))
        terms[i - startIndex] = decodeJSONValue(val.getJSONObject(i));
      else if (JSONUtils.isArray(val.get(i)))
    	terms[i - startIndex] = jpl.Util.termArrayToList(decodeJSONArray(val.getJSONArray(i)));
      else
        throw new InvalidJSONQuery("decodeJSONArray: unable to parse "
            + val.toString());
    }
    return terms;
  }

  static Term decodeJSONValue(String val) {

	  // wrap string in single quotes if it contains special characters
	  val = val.replace("\\", "");
	  Matcher matcher = Pattern.compile("^[\\w_]*$").matcher(val);
	  if(!matcher.find()) {
		  val="'"+val+"'";
	  }
	 return jpl.Util.textToTerm(val);
  }

  static Term decodeJSONValue(boolean val) {
    // Hack. No idea if we need to serialize bools and how.
    return new jpl.Integer(val ? 1 : 0);
  }

  static Term decodeJSONValue(double val) {
    return new jpl.Float(val);
  }

  static Term decodeJSONValue(int val) {
    return new jpl.Integer(val);
  }

  static Term decodeJSONValue(JSONObject val) throws InvalidJSONQuery {
    if (val.has("variable"))
      return new jpl.Variable(val.getString("variable"));
    else if (val.has("term"))
      return makeCompoundTerm(val.getJSONArray("term"));
    else if (val.has("list")) {
      JSONArray json_elements = val.getJSONArray("list");
      return jpl.Util.termArrayToList(decodeJSONArray(json_elements));
    } else
      throw new InvalidJSONQuery("decodeJSONValue: unable to parse "
          + val.toString());
  }
  
  static private JSONObject encodeCompoundTerm(Term term) {
    JSONObject result = new JSONObject();
    JSONArray term_list = new JSONArray();

    term_list.element(term.name());
    JSONArray args_list = encodeList(term.args());

    for (int i = 0; i < args_list.size(); i++)
      term_list.element(args_list.get(i));

    result.put("term", term_list);
    return result;
  }

  static private JSONArray encodeList(Term term) throws JPLException {
    Term[] term_list = term.toTermArray();
    return encodeList(term_list);
  }

  static private JSONArray encodeList(Term[] term_list) throws JPLException {
    JSONArray result = new JSONArray();
    for (int i = 0; i < term_list.length; i++) {
      if (term_list[i].isFloat())
        result.element(term_list[i].floatValue());
      else if (term_list[i].isInteger())
        result.element(term_list[i].intValue());
      else if (term_list[i].isAtom())
        result.element(term_list[i].name());
      else if (term_list[i].isCompound()) {
        if (isList(term_list[i]))
          result.element(encodeList(term_list[i]));
        else
          result.element(encodeCompoundTerm(term_list[i]));
      } else
        result.element(term_list[i].name());
    }

    return result;
  }

  static private boolean isList(Term term) {
    try {
      term.listLength();
      return true;
    } catch (JPLException e) {
      return false;
    }
  }

}	// 
