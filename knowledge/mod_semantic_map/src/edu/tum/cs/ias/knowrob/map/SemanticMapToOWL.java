package edu.tum.cs.ias.knowrob.map;

import java.util.ArrayList;
import java.util.HashMap;
import org.semanticweb.owlapi.model.*;

import edu.tum.cs.ias.knowrob.utils.owl.MapObject;
import edu.tum.cs.ias.knowrob.utils.owl.OWLImportExport;
import edu.tum.cs.ias.knowrob.utils.owl.OWLFileUtils;

import ros.*;
import ros.pkg.mod_semantic_map.srv.*;
import ros.pkg.mod_semantic_map.msg.*;


/**
 * ROS service to convert a mod_semantic_map/SemMap message into an OWL description
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 * @author Lars Kunze, kunzel@cs.tum.edu
 *
 */

public class SemanticMapToOWL {

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;

	/**
	 * Constructor. Advertises the needed ROS services.
	 * @param ros reference to rosjava
	 * @param n the node handle
	 * @throws RosException if advertising ROS services failed
	 */
	public SemanticMapToOWL() {

		try {

			initRos();

			n.advertiseService("/knowrob_semantic_map_to_owl/generate_owl_map", 
					new GenerateSemanticMapOWL(), 
					new ConvertToOwlCallback());
			ros.spin();

		} catch (RosException e) {
			e.printStackTrace();	
		}

	}



	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

		ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
			ros.init("knowrob_semantic_map_to_owl");
		}
		n = ros.createNodeHandle();

	}

	/**
	 * 
	 * Callback class for querying the Web for the object type of a barcode
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class ConvertToOwlCallback implements ServiceServer.Callback<GenerateSemanticMapOWL.Request, GenerateSemanticMapOWL.Response> {

		@Override
		public GenerateSemanticMapOWL.Response call(GenerateSemanticMapOWL.Request req) {

			GenerateSemanticMapOWL.Response res = new GenerateSemanticMapOWL.Response();
			res.owlmap="";

			if (req.map != null && req.map.objects.size()>0) {

				OWLImportExport export = new OWLImportExport();

				// Get IRI of target map from the frame_id of header msg
				String map_id = req.map.header.frame_id.toString();
				if (map_id.length() != 0) {
					// use provided map_id
					OWLImportExport.PREFIX_MANAGER.setPrefix("ias_map:", map_id);
				} else {
					//use IAS_MAP as default, PREFIX_MANAGER ist set by default 
					map_id = OWLImportExport.IAS_MAP;
				}
				System.out.println("Using map id: " + map_id);
				
				OWLOntology owlmap = export.createOWLMapDescription(map_id, semMapObj2MapObj(map_id, req.map.objects));
				res.owlmap = OWLFileUtils.saveOntologytoString(owlmap, owlmap.getOWLOntologyManager().getOntologyFormat(owlmap));

			}

			return res;
		}
	}

	
	private ArrayList<MapObject> semMapObj2MapObj(String map_id, ArrayList<SemMapObject> smos) {
		
		HashMap<Integer, MapObject> intIdToID = new HashMap<Integer, MapObject>();
		ArrayList<MapObject> mos = new ArrayList<MapObject>();
		
		for(SemMapObject smo : smos) {
			
			MapObject mo = new MapObject();
			
			mo.id = smo.type + smo.id;
			intIdToID.put(smo.id, mo);
			
			mo.types.add(smo.type);
			
			mo.dimensions.x=smo.width;
			mo.dimensions.y=smo.depth;
			mo.dimensions.z=smo.height;

			for(int i=0;i<4;i++) {
				for(int j=0;j<4;j++) {
					mo.pose_matrix.setElement(i, j, smo.pose[4*i+j]);
				}
			}

			if(intIdToID.get(smo.partOf) != null)
			    intIdToID.get(smo.partOf).physicalParts.add(mo);

			mos.add(mo);
		}
				
		return mos;
	}


	
	public static void main(String[] args) {


//		if (args.length == 0) {
			// run service
			new SemanticMapToOWL();
			
//		} else {
//			System.out.println("usage: rosrun mod_semantic_map SemanticMapToOWL");
//			System.out.println("Commands:");
//			System.out.println("        rosrun mod_semantic_map SemanticMapToOWL       Runs the service");
//			System.out.println();
//		}

	}


}
