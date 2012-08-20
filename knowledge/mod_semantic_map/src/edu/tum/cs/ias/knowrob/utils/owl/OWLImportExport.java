package edu.tum.cs.ias.knowrob.utils.owl;

import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.semanticweb.owlapi.reasoner.*;
import org.semanticweb.owlapi.reasoner.structural.*;

import edu.tum.cs.ias.knowrob.utils.ros.RosUtilities;



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
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(KNOWROB);
	static {
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
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
			PREFIX_MANAGER.setPrefix("map:", map_id);
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
						map_obj.types.contains("Cabinet-PieceOfFurniture") ||
						map_obj.types.contains("Bed-PieceOfFurniture") ||
						map_obj.types.contains("Sink")) {

					// top-level object, link to map
					OWLObjectProperty describedInMap = factory.getOWLObjectProperty("knowrob:describedInMap", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(describedInMap, obj_inst, idToInst.get(map_id)));
				}
				

				// link proper physical parts of an object				
				for(MapObject p: map_obj.physicalParts) {

					OWLIndividual part = idToInst.get(p.id);
					OWLObjectProperty properPhysicalParts = factory.getOWLObjectProperty("knowrob:properPhysicalParts", pm);
					manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(properPhysicalParts, obj_inst, part));
				}

				
				// link hinges in a special way (set child, parent, hingedTo)
				if(map_obj instanceof MapJoint) {

					OWLIndividual child = idToInst.get(((MapJoint) map_obj).child.id);
					OWLIndividual parent = idToInst.get(((MapJoint) map_obj).parent.id);
				
					// set joint connection between parent and child
					if(map_obj.types.contains("HingedJoint") && child!= null && parent!=null) {
						OWLObjectProperty hingedTo = factory.getOWLObjectProperty("knowrob:hingedTo", pm);
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(hingedTo, parent, child));
						
					} else if(map_obj.types.contains("PrismaticJoint")) {
						OWLObjectProperty prismaticallyConnectedTo = factory.getOWLObjectProperty("knowrob:prismaticallyConnectedTo", pm);
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(prismaticallyConnectedTo, parent, child));
					}
					
					// set rigid connection between joint and parent/child resp.
					OWLObjectProperty connectedTo = factory.getOWLObjectProperty("knowrob:connectedTo-Rigidly", pm);
					if(child!= null) {
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(connectedTo, obj_inst, child));
					}
					
					if(parent!=null) {
						manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(connectedTo, obj_inst, parent));
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
				instForClass("map:SemanticEnvironmentMap"), pm);
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

		
		// create hinge-specific properties
		if(map_obj instanceof MapJoint) {
			
			OWLDataProperty minJointValue = factory.getOWLDataProperty("knowrob:minJointValue", pm);
			manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(minJointValue, obj_inst, ((MapJoint) map_obj).q_min));

			OWLDataProperty maxJointValue = factory.getOWLDataProperty("knowrob:maxJointValue", pm);
			manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(maxJointValue, obj_inst, ((MapJoint) map_obj).q_max));

			OWLDataProperty turnRadius = factory.getOWLDataProperty("knowrob:turnRadius", pm);
			manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(turnRadius, obj_inst, ((MapJoint) map_obj).radius));
			
			// set direction for prismatic joints
			if(map_obj.types.contains("PrismaticJoint")) {
				
				OWLNamedIndividual dir_vec = createDirVector(((MapJoint) map_obj).direction, ontology);
				
				OWLObjectProperty direction = factory.getOWLObjectProperty("knowrob:direction", pm);
				manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(direction, obj_inst, dir_vec));
			}
		}
		
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

		OWLNamedIndividual time_inst = factory.getOWLNamedIndividual("map:timepoint_"+stamp.secs, pm);
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
	 * Create a Vector with all i components set according to the direction of the map_joint
	 * 
	 * @param dir_vec   Input data vector
	 * @param ontology  Ontology to which the axioms are to be added
	 * @return 			Created instance of a Vector
	 */
	public OWLNamedIndividual createDirVector(Vector3d dir_vec, OWLOntology ontology) { 

		// create pose matrix instance
		OWLClass vec_class = factory.getOWLClass("knowrob:Vector", pm);
		OWLNamedIndividual vec_inst = factory.getOWLNamedIndividual(instForClass("knowrob:Vector"), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(vec_class, vec_inst));

		// set vector dimensions
		OWLDataProperty vecX = factory.getOWLDataProperty("knowrob:vectorX", pm);
		OWLDataProperty vecY = factory.getOWLDataProperty("knowrob:vectorY", pm);
		OWLDataProperty vecZ = factory.getOWLDataProperty("knowrob:vectorZ", pm);
		
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(vecX,  vec_inst, dir_vec.x));
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(vecY,  vec_inst, dir_vec.y));
		manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(vecZ,  vec_inst, dir_vec.z));

		return vec_inst;
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

								// get types
								for(OWLClassExpression c: obj.getTypes(ont)) {
									cur.types.add(c.asOWLClass().toStringID().split("#")[1]);
								}
								
								// special treatment for MapJoints
								if(cur.types.contains("HingedJoint") || cur.types.contains("PrismaticJoint")) {
									cur = new MapJoint();
									for(OWLClassExpression c: obj.getTypes(ont)) {
										cur.types.add(c.asOWLClass().toStringID().split("#")[1]);
									}
								}

								cur.id = obj.toStringID().split("#")[1];


								if(cur.types.contains("SemanticEnvironmentMap"))
									continue;

								// get dimensions
								Map<OWLDataPropertyExpression, Set<OWLLiteral>> data_props = 
										obj.getDataPropertyValues(ont);
								Map<OWLObjectPropertyExpression, Set<OWLIndividual>> obj_props = 
										obj.getObjectPropertyValues(ont);

								OWLDataProperty width  = factory.getOWLDataProperty("knowrob:widthOfObject", pm);
								OWLDataProperty depth  = factory.getOWLDataProperty("knowrob:depthOfObject", pm);
								OWLDataProperty height = factory.getOWLDataProperty("knowrob:heightOfObject", pm);

								if(data_props.get(depth) != null) {
									for(OWLLiteral d : data_props.get(depth)) {
										cur.dimensions.x = Double.valueOf(d.getLiteral());
									}
								}
								
								if(data_props.get(width) != null) {
									for(OWLLiteral w : data_props.get(width)) {
										cur.dimensions.y = Double.valueOf(w.getLiteral());
									}
								}
								
								if(data_props.get(height) != null) {
									for(OWLLiteral h : data_props.get(height)) {
										cur.dimensions.z = Double.valueOf(h.getLiteral());
									}
								}

								// read hinge-specific properties
								if(cur.types.contains("HingedJoint") || cur.types.contains("PrismaticJoint")) {

									OWLDataProperty q_min  = factory.getOWLDataProperty("knowrob:minJointValue", pm);
									OWLDataProperty q_max  = factory.getOWLDataProperty("knowrob:maxJointValue", pm);

									if(data_props.containsKey(q_min)) {
										for(OWLLiteral qm : data_props.get(q_min)) {
											((MapJoint) cur).q_min = Double.valueOf(qm.getLiteral());
										}
									}

									if(data_props.containsKey(q_max)) {
										for(OWLLiteral qm : data_props.get(q_max)) {
											((MapJoint) cur).q_max = Double.valueOf(qm.getLiteral());
										}
									}

									if(cur.types.contains("HingedJoint")) {
										OWLDataProperty radius = factory.getOWLDataProperty("knowrob:turnRadius", pm);
										if(data_props.containsKey(radius)) {
											for(OWLLiteral rad : data_props.get(radius))
												((MapJoint) cur).radius = Double.valueOf(rad.getLiteral());
										}
									}

									if(cur.types.contains("PrismaticJoint")) {

										OWLObjectProperty direction = factory.getOWLObjectProperty("knowrob:direction", pm);
										
										if(obj_props.containsKey(direction)) {
											for(OWLIndividual dir : obj_props.get(direction)) {

												Map<OWLDataPropertyExpression, Set<OWLLiteral>> vec_props = 
														dir.getDataPropertyValues(ont);

												OWLDataProperty vectorx  = factory.getOWLDataProperty("knowrob:vectorX", pm);
												OWLDataProperty vectory  = factory.getOWLDataProperty("knowrob:vectorY", pm);
												OWLDataProperty vectorz  = factory.getOWLDataProperty("knowrob:vectorZ", pm);

												if(vec_props.containsKey(vectorx)) {
													for(OWLLiteral x : vec_props.get(vectorx)) {
														((MapJoint) cur).direction.x = Double.valueOf(x.getLiteral());
													}
												}
												if(vec_props.containsKey(vectory)) {
													for(OWLLiteral y : vec_props.get(vectory)) {
														((MapJoint) cur).direction.y = Double.valueOf(y.getLiteral());
													}
												}
												if(vec_props.containsKey(vectorz)) {
													for(OWLLiteral z : vec_props.get(vectorz)) {
														((MapJoint) cur).direction.z = Double.valueOf(z.getLiteral());
													}
												}
											}
										}
									}
								}


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
				
				
				// link objects to their physical parts
				for(OWLNamedIndividual instances : ont.getIndividualsInSignature()) {

					Set<OWLClassExpression> types = instances.getTypes(ont);
					OWLClass semanticMapPerception = factory.getOWLClass("knowrob:SemanticMapPerception", pm);

					if(types.contains(semanticMapPerception)) {

						Map<OWLObjectPropertyExpression, Set<OWLIndividual>> perc_props = 
							instances.getObjectPropertyValues(ont);

						OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);

						Set<OWLIndividual> objs  = perc_props.get(objectActedOn);

						
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
				
				// read parent and child
				for(OWLNamedIndividual instances : ont.getIndividualsInSignature()) {

					Set<OWLClassExpression> types = instances.getTypes(ont);
					OWLClass semanticMapPerception = factory.getOWLClass("knowrob:SemanticMapPerception", pm);

					if(types.contains(semanticMapPerception)) {

						Map<OWLObjectPropertyExpression, Set<OWLIndividual>> perc_props = 
							instances.getObjectPropertyValues(ont);

						OWLObjectProperty objectActedOn = factory.getOWLObjectProperty("knowrob:objectActedOn", pm);

						Set<OWLIndividual> objs  = perc_props.get(objectActedOn);


						
						for(OWLIndividual obj : objs) {
							
							Map<OWLObjectPropertyExpression, Set<OWLIndividual>> obj_props = 
									obj.getObjectPropertyValues(ont);
							OWLObjectProperty connectedTo = factory.getOWLObjectProperty("knowrob:connectedTo-Rigidly", pm);
							
							if(obj_props.containsKey(connectedTo)) {
								
								MapObject cur = objects.get(obj.toStringID().split("#")[1]);
								
								for(OWLIndividual rel : obj_props.get(connectedTo)) {
									
									MapObject connectedObj = objects.get(rel.toStringID().split("#")[1]);
									
									if(connectedObj!=null) {
										
										// connectedObj is a child if it is also contained 
										// in the physicalParts list of the current object
										if(connectedObj.physicalParts.contains(cur)) {
											((MapJoint) cur).parent = connectedObj;
										} else {
											((MapJoint) cur).child = connectedObj;
										}
										
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
			String knowrob_pkg = RosUtilities.rospackFind(KNOWROB_PKG);
			String knowrob_owl = knowrob_pkg + "/" + KNOWROB_OWL;


			// Load the knowrob ontology  
			OWLOntology ont = manager.loadOntologyFromOntologyDocument(new File(knowrob_owl));

			// Retrieve only subclasses of SpatialThing-Localized
			OWLReasoner reasoner = new StructuralReasoner(ont, new SimpleConfiguration(), BufferingMode.NON_BUFFERING);
			OWLClass spatialThing = factory.getOWLClass("knowrob:SpatialThing-Localized", pm);
			NodeSet<OWLClass> ns = reasoner.getSubClasses(spatialThing, false);

			java.util.Set<Node<OWLClass>>  set = ns.getNodes();       

			// Iterate over all subclasses and put them into the mapping hashmap
			for(Node<OWLClass> n : set) {
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
