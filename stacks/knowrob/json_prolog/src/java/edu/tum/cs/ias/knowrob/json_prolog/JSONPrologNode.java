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

import java.util.*;
import java.io.*;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.json_prolog.srv.PrologFinish;
import ros.pkg.json_prolog.srv.PrologNextSolution;
import ros.pkg.json_prolog.srv.PrologQuery;

public final class JSONPrologNode {
  private Hashtable<String, PrologSolutions> queries;
  private boolean hasIncrementalQuery = false; 
  private String initPackage="";
  

  public JSONPrologNode() {
	  this.initPackage="";
	  queries = new Hashtable<String, PrologSolutions>();
  }

  public JSONPrologNode(String initpkg) {
	  this.initPackage=initpkg;
  }
  
  static private class RospackError extends Exception {
    private static final long serialVersionUID = 1L;
  }

  private class QueryCallback implements
      ServiceServer.Callback<PrologQuery.Request, PrologQuery.Response> {
    @Override
    public PrologQuery.Response call(PrologQuery.Request request) {
      PrologQuery.Response response = new PrologQuery.Response();

      if (hasIncrementalQuery ) {
        response.ok = 0;
        response.message = "Already processing an incremental query.";
      } else if ( queries.get(request.id) != null ) {
        response.ok = 0;
        response.message = "Already processing a query with id " + request.id;
      } else {
        try {
          Ros.getInstance().logDebug("Received query with id " + request.id);
          jpl.Query currentQuery = JSONQuery.makeQuery(request.query);
          String currentQueryId = request.id;
          if(request.mode == PrologQuery.Request.INCREMENTAL) {
            queries.put(currentQueryId, new PrologIncrementalSolutions(currentQuery));
            hasIncrementalQuery = true;
          }
          else {
            queries.put(currentQueryId, new PrologAllSolutions(currentQuery));
          }
          response.ok = 1;
        } catch (JSONQuery.InvalidJSONQuery e) {
          response.ok = 0;
          response.message = e.toString();
        } catch (jpl.JPLException e) {
          response.ok = 0;
          response.message = e.getMessage();
        }
      }
      return response;
    }
  }
  
  private class SimpleQueryCallback implements
      ServiceServer.Callback<PrologQuery.Request, PrologQuery.Response> {
    @Override
    public PrologQuery.Response call(PrologQuery.Request request) {
      PrologQuery.Response response = new PrologQuery.Response();

      if (hasIncrementalQuery ) {
        response.ok = 0;
        response.message = "Already processing an incremental query.";
      } else if ( queries.get(request.id) != null ) {
        response.ok = 0;
        response.message = "Already processing a query with id " + request.id;
      } else {
        try {
          Ros.getInstance().logDebug("Received query with id " + request.id);
          jpl.Query currentQuery = new jpl.Query(request.query);
          String currentQueryId = request.id;
          if(request.mode == PrologQuery.Request.INCREMENTAL) {
            queries.put(currentQueryId, new PrologIncrementalSolutions(currentQuery));
            hasIncrementalQuery = true;
          }
          else {
            queries.put(currentQueryId, new PrologAllSolutions(currentQuery));
          }
          response.ok = 1;
        } catch (jpl.JPLException e) {
          response.ok = 0;
          response.message = e.getMessage();
        }
      }
      return response;
    }
  }

  private class NextSolutionCallback
      implements
      ServiceServer.Callback<PrologNextSolution.Request, PrologNextSolution.Response> {
    @Override
    public PrologNextSolution.Response call(PrologNextSolution.Request request) {
      PrologNextSolution.Response response = new PrologNextSolution.Response();

      try {
        PrologSolutions currentQuery = queries.get(request.id);
        if (currentQuery == null)
          response.status = PrologNextSolution.Response.WRONG_ID;
        else if (!currentQuery.hasMoreSolutions()){
          response.status = PrologNextSolution.Response.NO_SOLUTION;
          removeQuery(request.id);
        } else {
          Hashtable<String, jpl.Term> solution = (Hashtable<String, jpl.Term>) currentQuery.nextSolution();
          response.solution = JSONQuery.encodeResult(solution).toString();
          response.status = PrologNextSolution.Response.OK;
        }
      } catch (jpl.JPLException e) {
        response.solution = e.getMessage();
        response.status = PrologNextSolution.Response.QUERY_FAILED;
        removeQuery(request.id);
      }
      return response;
    }
  }

  private class FinishCallback implements
      ServiceServer.Callback<PrologFinish.Request, PrologFinish.Response> {
    @Override
    public PrologFinish.Response call(PrologFinish.Request request) {
      PrologFinish.Response response = new PrologFinish.Response();
      if (request.id.equals("*")){
        Enumeration<String> e = queries.keys();
        while(e.hasMoreElements())
          removeQuery(e.nextElement());
      }
      else
        removeQuery(request.id);
      return response;
    }
  }


// public class QueryCallback extends ServiceServer<PrologQuery.>
  public void execute() throws InterruptedException, RosException, IOException,
      RospackError {
    initProlog();
    
    if(!this.initPackage.equals("")) {
        new jpl.Query("ensure_loaded('" + findRosPackage(initPackage)
                + "/prolog/init.pl')").oneSolution();
    }

    final Ros ros = Ros.getInstance();
	
	if(!Ros.getInstance().isInitialized()) {
    	ros.init("json_prolog");
	}
    
    NodeHandle n = ros.createNodeHandle("~");

    @SuppressWarnings("unused")
    ServiceServer<PrologQuery.Request, PrologQuery.Response, PrologQuery> query_srv = n
        .advertiseService("query", new PrologQuery(), new QueryCallback());
    @SuppressWarnings("unused")
    ServiceServer<PrologQuery.Request, PrologQuery.Response, PrologQuery> simple_query_srv = n
        .advertiseService("simple_query", new PrologQuery(), new SimpleQueryCallback());
    @SuppressWarnings("unused")
    ServiceServer<PrologNextSolution.Request, PrologNextSolution.Response, PrologNextSolution> next_solution_srv = n
        .advertiseService("next_solution", new PrologNextSolution(),
            new NextSolutionCallback());
    @SuppressWarnings("unused")
    ServiceServer<PrologFinish.Request, PrologFinish.Response, PrologFinish> finish_srv = n
        .advertiseService("finish", new PrologFinish(), new FinishCallback());

    ros.logInfo("json_prolog initialized and waiting for queries.");

    ros.spin();
  }

  public void removeQuery(String id) {
    PrologSolutions query = queries.get(id);
    if(query != null) {
      query.close();
      if(query instanceof PrologIncrementalSolutions)
        hasIncrementalQuery = false;
    }
  }

  private static void initProlog() throws IOException, InterruptedException,
      RospackError {
    Vector<String> pl_args = new Vector<String>(Arrays.asList(jpl.JPL
        .getDefaultInitArgs()));
    pl_args.set(0, "/usr/bin/swipl");
    pl_args.add("-G256M");
    pl_args.add("-nosignals");
    jpl.JPL.setDefaultInitArgs(pl_args.toArray(new String[0]));
    jpl.JPL.init();
    new jpl.Query("ensure_loaded('" + findRosPackage("rosprolog")
        + "/prolog/init.pl')").oneSolution();
  }

  private static String findRosPackage(String name) throws IOException,
      InterruptedException, RospackError {
    Process rospack = Runtime.getRuntime().exec("rospack find " + name);
    if (rospack.waitFor() != 0)
      throw new RospackError();
    return new BufferedReader(new InputStreamReader(rospack.getInputStream()))
        .readLine();
  }

  public static void main(String args[]) {
    try {

    	if(args.length>0) {
    		new JSONPrologNode(args[0]).execute();
    	} else {
    		new JSONPrologNode().execute();	
    	}
    	
      
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
