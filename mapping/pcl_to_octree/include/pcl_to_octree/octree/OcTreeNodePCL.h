#ifndef OCTOMAP_OCTREE_NODE_PCL_H
#define OCTOMAP_OCTREE_NODE_PCL_H

/*
* Based on orignal code writtien by K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @author: Hozefa Indorewala 
*/

#include "octomap/OcTreeNode.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace octomap {


  /**
   *   Node class storing a label as additional information
   */
  class OcTreeNodePCL : public OcTreeNode {

  public:

    int label;
    point3d centroid;
    double resolution;
    std::vector <int> inliers;

  public:

    OcTreeNodePCL();
    virtual ~OcTreeNodePCL();


    // -- children  ----------------------------------

    virtual bool createChild(unsigned int i);
    virtual OcTreeNodePCL* getChild(unsigned int i);
    virtual const OcTreeNodePCL* getChild(unsigned int i) const;
 
    
    // data  -----------------------------------------

    /**
     * set a label
     */
    void setLabel(int l);
    /**
     * @return Label of node
     */
    int getLabel() const;


    /**
     * set a centroid
     */
    void setCentroid(point3d c);
    /**
     * @return centroid of node
     */
    point3d getCentroid() const;

    /**
     * set a resolution
     */
    void setResolution(double res);
    /**
     * @return resolution of node
     */
    double getResolution() const;

    /**
     * set a Leaf Node inliers
     */
    void set3DPointInliers(int inlier_index);
    /**
     * @return Leaf Node inliers
     */
    std::vector<int> get3DPointInliers();
 
     // -- I/O  ---------------------------------------

    /**
     * Read node from binary stream (max-likelihood value), recursively
     * continue with all children.
     *
     * This will set the log_odds_occupancy value of
     * all leaves to either free or occupied.
     *
     * @param s
     * @return
     */
    std::istream& readBinary(std::istream &s);

    /**
     * Write node to binary stream (max-likelihood value),
     * recursively continue with all children.
     *
     * This will discard the log_odds_occupancy value, writing
     * all leaves as either free or occupied.
     *
     * @param s
     * @return
     */
    std::ostream& writeBinary(std::ostream &s) const;

   

  };



} // end namespace



#endif
