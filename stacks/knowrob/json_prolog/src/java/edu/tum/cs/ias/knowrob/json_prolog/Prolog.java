package edu.tum.cs.ias.knowrob.json_prolog;

import ros.NodeHandle;
import ros.Ros;
import ros.ServiceClient;

import ros.pkg.json_prolog.srv.PrologQuery;
import ros.pkg.json_prolog.srv.PrologFinish;
import ros.pkg.json_prolog.srv.PrologNextSolution;


/**
 * Client wrapper library for the json_prolog interface
 * 
 * Main class to be used in other applications. For example usage, see {@code JSONPrologTestClient}
 * 
 * The implementation was intentionally kept similar to the C++ client library by Lorenz M\"osenlechner
 * 
 * @author tenorth
 *
 */
public class Prolog {

	private static Ros ros;
	private static NodeHandle n;
	
	protected static void initRos() {

    	ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
	    	ros.init("json_prolog_java_client");
		}
		n = ros.createNodeHandle();

	}
	
	public ServiceClient<PrologQuery.Request, PrologQuery.Response, PrologQuery> prolog_query;
	public ServiceClient<PrologNextSolution.Request, PrologNextSolution.Response, PrologNextSolution> next_solution;
	public ServiceClient<PrologFinish.Request, PrologFinish.Response, PrologFinish> prolog_finish;

	
	
	/**
	 * Constructor. Specify the ROS name space (default: "/json_prolog")
	 * 
	 * @param ns
	 */
	public Prolog(String ns) {
		
		initRos();
		prolog_query = n.serviceClient(ns + "/simple_query", new PrologQuery());
		next_solution = n.serviceClient(ns + "/next_solution", new PrologNextSolution());
		prolog_finish = n.serviceClient(ns + "/finish", new PrologFinish());				
	}
	public Prolog() {
		this("/json_prolog");
	}
	
	/**
	 * Send a query to Prolog
	 * 
	 * @param query_str The query to be sent to Prolog (standard SWI Prolog syntax)
	 * @return An instance of the PrologQueryProxy that can be used to iterate over the results
	 * 
	 */
	public PrologQueryProxy query(String query_str) {
		return new PrologQueryProxy(this, query_str);
	}
	
	/**
	 * Send a query to Prolog and retrieve only the first result. Finishes the
	 * query after retrieving the result.
	 * 
	 * @param query_str The query to be sent to Prolog (standard SWI Prolog syntax)
	 * @return PrologBindings for the first result.
	 * 
	 */
	public PrologBindings once(String query_str) {
		
		PrologQueryProxy query = new PrologQueryProxy(this, query_str);
		PrologBindings result = query.iterator().next();
		query.finish();
		return result;
	}
	 

	
}
