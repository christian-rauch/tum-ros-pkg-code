package edu.tum.cs.ias.knowrob.utils.owl;

import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Matrix4d;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.semanticweb.owlapi.reasoner.*;
import org.semanticweb.owlapi.reasoner.structural.*;

import edu.tum.cs.ias.knowrob.utils.ROSUtils;



/**
 * 
 * Utilities for the import and export of OWL files 
 * from/to Java data structures
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 * @author Lars Kunze, kunzel@cs.tum.edu
 *
 */


public class OWLImportExport {


	////////////////////////////////////////////////////////////////////////////////
	// Set IRIs for the ontologies used here
	//

	// Base IRI for KnowRob ontology
	public final static String KNOWROB = "http://ias.cs.tum.edu/kb/knowrob.owl#";

	// Base IRI for OWL ontology
	public final static String OWL = "http://www.w3.org/2002/07/owl#";

	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";

	// Base IRI for semantic map ontology	
	public final static String IAS_MAP = "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#";

	// ROS package name for KnowRob
	public final static String KNOWROB_PKG = "ias_knowledge_base";

	// OWL file of the KnowRob ontology (relative to KNOWROB_PKG)
	public final static String KNOWROB_OWL = "owl/knowrob.owl";

	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(IAS_MAP);
	static {
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("ias_map:", IAS_MAP);
		PREFIX_MANAGER.setPrefix("owl:",    OWL);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
	}

	// mapping ROS-KnowRob identifiers
	protected static final HashMap<String, String> rosToKnowrob = new HashMap<String, String>();

	OWLDataFactory factory;
	OWLOntologyManager manager;
	DefaultPrefixManager pm;

	int inst_counter=0;	// counter to create unique instance identifiers

	public OWLImportExport() {
		//readKnowRobObjectClasses();
	}


	/**
	 * Build a complete semantic map, including name spaces, all contained objects,
	 * and the links between them.
	 * 
	 * @param map
	 * @return
	 */
	public OWLOntology createOWLMapDescription(String map_id, ArrayList<MapObject> map) {

		OWLOntology ontology = null;
		HashMap<String, OWLNamedIndividual> idToInst = new HashMap<String, OWLNamedIndividual>(); 
		
		try {
			
			// Create ontology manager and data factory
			manager = OWLManager.createOWLOntologyManager();
			factory = manager.getOWLDataFactory();

			// Get prefix manager using the base IRI of the JoystickDrive ontology as default namespace
			pm = PREFIX_MANAGER;

			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(map_id));
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(KNOWROB));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);

			
			// create SemanticMap object in the ontology
			idToInst.put(map_id, createSemMapInst(ontology));

			// create time point 
			OWLNamedIndividual time_inst = createTimePointInst(ros.communication.Time.now(), ontology);

			
			// iterate over all objects and create the respective OWL representations
			for(MapObject map_obj : map) {
				idToInst.put(map_obj.id, createSemMapObjectDescription(map_obj, time_inst, ontology));
			}

			
			// link to parent objects (second loop to avoid problems due to wrong ordering)
			for(MapObject map_obj : map) {
				OWLNamedIndividual obj_inst = idToInst.get(map_obj.id);

				// link high-level objects to the map
				if(map_obj.types.contains("Cupboard") ||
						map_obj.types.contains("Drawer") ||
						map_obj.types.contains("Oven") ||
						map_obj.types.contains("Refrigerator") ||
						map_obj.types.contains("Dishwasher") ||
						map_obj.types.contains("Table") ||
						map_obj.types.contains("CounterTop") ||
						map_obj.types.contains("Sink")) {

					// top-level object, link to map
					OWLObjectProperty describedInMap = factory.getOWLObjectProperty("knowrob:describedInMap", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(describedInMap, obj_inst, idToInst.get(map_id)));
					
					
				// link object components based on their types
				} 
				
				for(MapObject p: map_obj.physicalParts) {

					OWLIndividual part = idToInst.get(p.id);
					
					if(p.types.contains("Door")) {

						// doors are part of parent and may be hinged to it
						OWLObjectProperty properPhysicalParts = factory.getOWLObjectProperty("knowrob:properPhysicalParts", pm);
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(properPhysicalParts, obj_inst, part));

						
						// search if map_obj has a rotational joint as part and set the hingedTo property in that case 
						for(MapObject pi : map_obj.physicalParts) {
							
							if(pi.types.contains("HingedJoint")) {
								OWLObjectProperty hingedTo = factory.getOWLObjectProperty("knowrob:hingedTo", pm);
								manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(hingedTo, part, obj_inst));
							}
						}
						
					} else {

						// other objects are only part of their parents
						OWLObjectProperty properPhysicalParts = factory.getOWLObjectProperty("knowrob:properPhysicalParts", pm);
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(properPhysicalParts, obj_inst, part));

					}
				}
			}
			
		} catch (Exception e) {
			ontology = null;
			e.printStackTrace();
		}

		return ontology;
	}


	
	/**
	 * Create the OWL description for an object, including the object instance with its dimension,
	 * a pose instance where the object has been detected, and a SemanticMapPerception instance 
	 * linking object, pose, and detection time
	 * 
	 * @param map_obj   MapObject input data
	 * @param timestamp OWLIndividual for the time when map_obj has been perceived
	 * @param ontology  Ontology to which the axioms are to be added
	 */
	public OWLNamedIndividual createSemMapObjectDescription(MapObject map_obj, OWLNamedIndividual timestamp, OWLOntology ontology) {

		// create object instance
		OWLNamedIndividual obj_inst = createObjectInst(map_obj, ontology);

		// create pose matrix instance
		OWLNamedIndividual pose_inst = createPoseInst(map_obj.pose_matrix, ontology);

		// create perception instance
		createPerceptionInst("knowrob:SemanticMapPerception", obj_inst, pose_inst, timestamp, ontology);

		return obj_inst;
	}


	/**
	 * Create an instance of a knowrob:SemanticEnvironmentMap
	 * 
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a SemanticEnvironmentMap
	 */
	public OWLNamedIndividual createSemMapInst(OWLOntology ontology) {

		OWLClass sem_map_class = factory.getOWLClass("knowrob:SemanticEnvironmentMap", pm);
		OWLNamedIndividual sem_map_inst = factory.getOWLNamedIndividual(
				instForClass("ias_map:SemanticEnvironmentMap"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(sem_map_class, sem_map_inst));

		return sem_map_inst;
	}

	/**
	 * Generate an instance of the object class indicated by map_obj.type and link it to its parent object
	 * 
	 * @param map_obj   MapObject input data
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of the respective object
	 */
	public OWLNamedIndividual createObjectInst(MapObject map_obj, OWLOntology ontology) {


		OWLNamedIndividual obj_inst = factory.getOWLNamedIndividual("knowrob:"+map_obj.id, pm);
		
		for(String t : map_obj.types) {
			OWLClass obj_class = factory.getOWLClass("knowrob:"+t, pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(obj_class, obj_inst));	
		}
		
		// set object dimensions
		OWLDataProperty width  = factory.getOWLDataProperty("knowrob:widthOfObject",  pm);
		OWLDataProperty depth  = factory.getOWLDataProperty("knowrob:depthOfObject",  pm);
		OWLDataProperty height = factory.getOWLDataProperty("knowrob:heightOfObject", pm);

		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(depth,  obj_inst, map_obj.dimensions.x));
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(width,  obj_inst, map_obj.dimensions.y));
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(height, obj_inst, map_obj.dimensions.z));

		return obj_inst;
	}

	/**
	 * Create an instance of a knowrob:TimePoint for time t
	 * 
	 * @param t         Time to be translated into a TimePoint
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a TimePoint
	 */
	public OWLNamedIndividual createTimePointInst(ros.communication.Time stamp, OWLOntology ontology) {

		OWLNamedIndividual time_inst = factory.getOWLNamedIndividual("ias_map:timepoint_"+stamp.secs, pm);
		OWLClass time_class = factory.getOWLClass("knowrob:TimePoint", pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(time_class, time_inst));

		return time_inst;
	}

	
	/**
	 * Create a RotationMatrix3D with all m_ij components set according to the pose of the map_obj
	 * 
	 * @param map_obj   MapObject input data
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a RotationMatrix3D
	 */
	public OWLNamedIndividual createPoseInst(Matrix4d pose, OWLOntology ontology) { 

		// create pose matrix instance
		OWLClass pose_class = factory.getOWLClass("knowrob:RotationMatrix3D", pm);
		OWLNamedIndividual pose_inst = factory.getOWLNamedIndividual(
				instForClass("knowrob:RotationMatrix3D"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(pose_class, pose_inst));

		// set pose properties
		for(int i=0;i<4;i++) {
			for(int j=0;j<4;j++) {
				OWLDataProperty prop = factory.getOWLDataProperty("knowrob:m"+i+j, pm);
				manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(prop,  pose_inst, pose.getElement(i,j)));
			}
		}

		return pose_inst;
	}	


	/**
	 * Create an instance of a SemanticMapPerception linking objects to poses and times
	 * 
	 * @param type      Type of the perception, e.g. "knowrob:SemanticMapPerception"
	 * @param obj_inst  The object that was detected
	 * @param pose_inst Pose where the object was detected
	 * @param timestamp Time when the object was detected
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of SemanticMapPerception
	 */
	public OWLNamedIndividual createPerceptionInst(String type, OWLNamedIndividual obj_inst, OWLNamedIndividual pose_inst, OWLNamedIndividual timestamp, OWLOntology ontology) {

		// create perception instance
		OWLClass perc_class = factory.getOWLClass(type, pm);
		OWLNamedIndividual perc_inst = factory.getOWLNamedIndividual(
				instForClass(type), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(perc_class, perc_inst));

		// link to the object instance and the pose instance
		OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);
		OWLObjectProperty eventOccursAt = factory.getOWLObjectProperty("knowrob:eventOccursAt", pm);

		manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(objectActedOn, perc_inst, obj_inst));
		manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(eventOccursAt, perc_inst, pose_inst));

		// set time stamp
		OWLObjectProperty startTime = factory.getOWLObjectProperty("knowrob:startTime", pm);
		manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(startTime,  perc_inst, timestamp));

		return perc_inst;
	}


	
	/**
	 * Read a semantic map from an OWL file into a HashMap representation, using the object
	 * identifier as key and the object pose description as value.
	 * 
	 * @param filename Name of the OWL file to be imported
	 * @return HashMap with the object identifiers as key and the MapObject data structures as values
	 */
	static public HashMap<String, MapObject> readMapObjectFromOWL(String filename) {
		
		HashMap<String, MapObject> objects = new HashMap<String, MapObject>();
    	OWLOntology ont = null;
    	try {
    		
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();
			DefaultPrefixManager pm = OWLImportExport.PREFIX_MANAGER;
			
			ont = OWLFileUtils.loadOntologyFromFile(filename);
			
			if(ont!=null) {

				// iterate over objects, add to objects hashmap 
				for(OWLNamedIndividual instances : ont.getIndividualsInSignature()) {
										
					Set<OWLClassExpression> types = instances.getTypes(ont);
					OWLClass semanticMapPerception = factory.getOWLClass("knowrob:SemanticMapPerception", pm);
					
					if(types.contains(semanticMapPerception)) {
						
						Map<OWLObjectPropertyExpression, Set<OWLIndividual>> perc_props = 
							instances.getObjectPropertyValues(ont);
						
						OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);
						OWLObjectProperty eventOccursAt = factory.getOWLObjectProperty("knowrob:eventOccursAt", pm);

						Set<OWLIndividual> objs  = perc_props.get(objectActedOn);
						Set<OWLIndividual> poses = perc_props.get(eventOccursAt);

						
						// assuming there is only one object and one pose per perception:
						for(OWLIndividual obj : objs) {
							for(OWLIndividual pose : poses) {
								
								// create map object
								MapObject cur = new MapObject();
								cur.id = obj.toStringID().split("#")[1];
								
								// get types
								for(OWLClassExpression c: obj.getTypes(ont))
									cur.types.add(c.asOWLClass().toStringID().split("#")[1]);

								// get dimensions
								Map<OWLDataPropertyExpression, Set<OWLLiteral>> data_props = 
									obj.getDataPropertyValues(ont);
								
								OWLDataProperty width  = factory.getOWLDataProperty("knowrob:widthOfObject", pm);
								OWLDataProperty depth  = factory.getOWLDataProperty("knowrob:depthOfObject", pm);
								OWLDataProperty height = factory.getOWLDataProperty("knowrob:heightOfObject", pm);


								for(OWLLiteral d : data_props.get(depth))
									cur.dimensions.x = Double.valueOf(d.getLiteral());

								for(OWLLiteral w : data_props.get(width))
									cur.dimensions.y = Double.valueOf(w.getLiteral());
								
								for(OWLLiteral h : data_props.get(height))
									cur.dimensions.z = Double.valueOf(h.getLiteral());

								
								// get pose elements
								Map<OWLDataPropertyExpression, Set<OWLLiteral>> matrix_elems = 
									pose.getDataPropertyValues(ont);
								
								for(int i=0;i<4;i++) {
									for(int j=0;j<4;j++){
										OWLDataProperty m_ij = factory.getOWLDataProperty("knowrob:m"+i+j, pm);
										Set<OWLLiteral> elem = matrix_elems.get(m_ij);
										
										for(OWLLiteral e : elem) {
											cur.pose_matrix.setElement(i, j, Double.valueOf(e.getLiteral()) );	
										}
										
										
									}
								}
								objects.put(cur.id, cur);

							}
						}
					}
				}

				for(OWLNamedIndividual instances : ont.getIndividualsInSignature()) {

					Set<OWLClassExpression> types = instances.getTypes(ont);
					OWLClass semanticMapPerception = factory.getOWLClass("knowrob:SemanticMapPerception", pm);

					if(types.contains(semanticMapPerception)) {

						Map<OWLObjectPropertyExpression, Set<OWLIndividual>> perc_props = 
							instances.getObjectPropertyValues(ont);

						OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);

						Set<OWLIndividual> objs  = perc_props.get(objectActedOn);

						// link objects to their physical parts
						for(OWLIndividual obj : objs) {

							// get proper physical parts
							Map<OWLObjectPropertyExpression, Set<OWLIndividual>> obj_props = 
								obj.getObjectPropertyValues(ont);

							OWLObjectProperty parts = factory.getOWLObjectProperty("knowrob:properPhysicalParts", pm);

							if(obj_props.containsKey(parts)) {
								for(OWLIndividual p : obj_props.get(parts)) {
									MapObject part = objects.get(p.toStringID().split("#")[1]);
									if(part!=null) {
										objects.get(obj.toStringID().split("#")[1]).physicalParts.add(part);
									}
								}
							}
						}

					}
				}
			}
			
		} catch (OWLOntologyCreationException e) {
			e.printStackTrace();
		}
		return objects;
	}

	
	
	/**
	 * Initialization of the mapping between object types that are sent via 
	 * the ROS service and concepts of the KnowRob ontology.
	 */
	protected static void readKnowRobObjectClasses() {

		try {
			
			// Create ontology manager, data factory, and prefix manager
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();
			DefaultPrefixManager pm = PREFIX_MANAGER;

			// Find ros package holding the knowrob ontology
			String knowrob_pkg = ROSUtils.rospackFind(KNOWROB_PKG);
			String knowrob_owl = knowrob_pkg + "/" + KNOWROB_OWL;


			// Load the knowrob ontology  
			OWLOntology ont = manager.loadOntologyFromOntologyDocument(new File(knowrob_owl));

			// Retrieve only subclasses of SpatialThing-Localized
			OWLReasoner reasoner = new StructuralReasoner(ont, new SimpleConfiguration(), BufferingMode.NON_BUFFERING);
			OWLClass spatialThing = factory.getOWLClass("knowrob:SpatialThing-Localized", pm);
			NodeSet<OWLClass> ns = reasoner.getSubClasses(spatialThing, false);

			java.util.Set<Node<OWLClass>>  set = ns.getNodes();       

			// Iterate over all subclasses and put them into the mapping hashmap
			for(Node n : set) {
				OWLClass c = (OWLClass) n.getRepresentativeElement();

				String iri = c.toStringID().replaceAll(KNOWROB, "knowrob:");
				String key = c.toStringID().substring(c.toStringID().lastIndexOf('#') + 1).toLowerCase();

				rosToKnowrob.put(key, iri);
			}
			// to support backward compatibility (should be removed)
			rosToKnowrob.put("hinge", "knowrob:HingedJoint");
			rosToKnowrob.put("knob",  "knowrob:ControlKnob");
			rosToKnowrob.put("horizontal_plane", "knowrob:CounterTop");

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	

	/**
	 * Create a unique instance identifier from a class string
	 * @param cl 	Class string
	 * @return 		Instance identifier (class string plus index)
	 */
	protected String instForClass(String cl) {
		return cl+(inst_counter++);
	}


	/**
	 * Debug method: print all object types imported from KnowRob
	 */
	protected static void printObjectTypes() {

		for(Object o : rosToKnowrob.values()) {
			System.out.println(((String)o).replaceAll("knowrob:",""));
		}
	}


}
