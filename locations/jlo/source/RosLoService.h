/**********************************************************************************************/
/**
*              Jennifer's Located Object
*              Copyright  (C) 2008, U. Klank
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
 **********************************************************************************************/
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sstream>

#include <vision_msgs/partial_lo.h>
#include <vision_srvs/srvjlo.h>
#include <lo/ServiceInterface.h>

#include "tf/tfMessage.h"
#include "tf/tf.h"

#include <set>
#include <string>

/**
*     class RosLoService
*     Implementation for ros calling the lo-ServiceInterface functionality
*/
class RosLoService : public jlo::ServiceInterface
{
public:
  /**
  *     Start the service listening to listeningTopic, g_stopAll can stop the loop after next read
  */
  RosLoService (const char* nodename, ros::NodeHandle &n, std::string configFile);
  /**
  *   Closing of the ports
  */
  ~RosLoService ();
  /**
  * Parses a service request and reacts on the input, by generating the answer
  * @param botIn incoming request
  * @param bot outgoing answer
  */
  static bool ServiceCallback(vision_srvs::srvjlo::Request& request, vision_srvs::srvjlo::Response&  answer);

private:
  /**
  *
  */
  ros::Subscriber tf_subscription;
  ros::ServiceServer located_object_service;

  std::set<std::string> tf_blacklist;

  /**
  * subscribing to one tf topic to update lowlevel entries
  **/
  void tf_subscription_callback(const tf::tfMessage::ConstPtr &msg_in_);
};
