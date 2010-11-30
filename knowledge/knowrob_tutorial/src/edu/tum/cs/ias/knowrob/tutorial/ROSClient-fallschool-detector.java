/*
 * Copyright (C) 2010 by Moritz Tenorth
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package edu.tum.cs.ias.knowrob.tutorial;

import ros.*;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Quaternion;
import ros.pkg.mapping_msgs.msg.*;
import ros.pkg.table_objects.srv.*;

public class ROSClient {

        static Boolean rosInitialized = false;
        static Ros ros;
        static NodeHandle n;


        /**
         * Constructor: initializes the ROS environment
         *
         * @param node_name A unique node name
         */
        public ROSClient(String node_name) {
                initRos(node_name);
        }


        /**
         * Initialize the ROS environment if it has not yet been initialized
         *
         * @param node_name A unique node name
         */
        protected static void initRos(String node_name) {

                ros = Ros.getInstance();

                if(!Ros.getInstance().isInitialized()) {
                        ros.init(node_name);
                }
                n = ros.createNodeHandle();

        }


        /**
         * Call the table_objects service and return the result
         * @return An array of CollisionObjects referring to the objects detected on the table
         */
        public CollisionObject[] callTableObjects() {

                CollisionObject[] r=null;
                try {

                        // call the table_objects service
                        ServiceClient<GetCurrentObjects.Request, GetCurrentObjects.Response, GetCurrentObjects> cl =
                                n.serviceClient("/detect_objects_srv/get_table_objects", new GetCurrentObjects());

                        r = cl.call(new GetCurrentObjects.Request()).table_objects;
                        cl.shutdown();

                } catch (RosException e) {
                        ros.logError("ROSClient: Call to service /detect_objects_srv/get_table_objects failed");
                }

                return r;
        }



        /**
         * Read the pose of an object from the DatabaseModelPose (detection result)
         * @param p A detected object as DatabaseModelPose
         * @return Row-based 4x4 pose matrix representation
         */
        public static double[] objectPose(CollisionObject p) {
          return quaternionToMatrix(p.poses[0].position, p.poses[0].orientation);
        }

        /**
         * Convert point/quaternion into a 4x4 pose matrix
         *
         * @param p Point (position)
         * @param q Quaternion (orientation)
         * @return 4x4 pose matrix, row-based
         */
        protected static double[] quaternionToMatrix(Point p, Quaternion q) {

                double[] m = new double[16];

            double xx = q.x * q.x;
            double xy = q.x * q.y;
            double xz = q.x * q.z;
            double xw = q.x * q.w;

            double yy = q.y * q.y;
            double yz = q.y * q.z;
            double yw = q.y * q.w;

            double zz = q.z * q.z;
            double zw = q.z * q.w;

            m[0]  = 1 - 2 * ( yy + zz );
            m[1]  =     2 * ( xy - zw );
            m[2]  =     2 * ( xz + yw );
            m[3]  = p.x;

            m[4]  =     2 * ( xy + zw );
            m[5]  = 1 - 2 * ( xx + zz );
            m[6]  =     2 * ( yz - xw );
            m[7]  = p.y;

            m[8]  =     2 * ( xz - yw );
            m[9]  =     2 * ( yz + xw );
            m[10] = 1 - 2 * ( xx + yy );
            m[11]=p.z;

            m[12]=0;
            m[13]=0;
            m[14]=0;
            m[15]=1;
            return m;
        }



        /*
         * Test method: call the tabletop_object_detector and print results
         */
        public static void main(String[] args) {

                ROSClient r = new ROSClient("knowrob_tabletop_test_123");

                CollisionObject[] res = r.callTableObjects();

                for(int i=0;i<res.length;i++) {
                        System.out.println(res[i].id);
                        System.out.println("Pos: "+res[i].poses[0].position.x+","+res[i].poses[0].position.y+","+res[i].poses[0].position.z);
                }
        }

}
