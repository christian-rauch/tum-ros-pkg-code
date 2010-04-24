# Message to quiey the lo-service, U. Klank klank@in.tum.de
uint64 id		            #id of a frame, there should be unique mapping from a tf-name-string to such an id
string name	           	#optional parameter name representing a tf and knowledgebase correspondence
uint64 parent_id        #id of parent frame
float64[16] pose	      #pose matrix, fully projective 4x4 matrix
float64[36] cov         #covariance for 6 dof (xyz, rpy)
uint16 type             #fixed connection with the parent (1) or free in space (0 = default)
