#ifndef COP_CLIENT_H
#define COP_CLIENT_H


#include <ros/ros.h>
#include <vision_msgs/cop_answer.h>
#include <stdio.h>
#include <tf/tf.h>


class CopClient
{
   public:
      CopClient(ros::NodeHandle &nh, std::string stTopicName = "/tracking/cop_handler", std::string stCopName = "cop");

      unsigned long LOFrameQuery(unsigned long child, unsigned long parent);
      void          LOFreeID(unsigned long id);
      double        LODistanceQuery(unsigned long child, unsigned long parent);
      void          LOPointQuery(unsigned long child, float *vector);
      unsigned long LONameQuery(std::string name);
      tf::Stamped<tf::Pose> LOPoseQuery(unsigned long child);

      unsigned long LOOffset(unsigned long id, unsigned long parent, float x, float y, float z);


      size_t HasResult(long perception_primitive)
      {
        if(m_msg_cluster.find(perception_primitive) != m_msg_cluster.end())
          return m_msg_cluster[perception_primitive].size();
        else
        return 0;
      }
      std::vector<vision_msgs::cop_answer> GetResult(long perception_primitive)
      {
        if(m_msg_cluster.find(perception_primitive) != m_msg_cluster.end())
          return m_msg_cluster[perception_primitive];
        else
          return std::vector<vision_msgs::cop_answer> ();
      }

     long CallCop(std::string object_class, unsigned long position_id, int num_objects = 1, unsigned long alterantive_id = 0, unsigned long action_type = 0);
     long CallCop(std::string object_class, std::vector<unsigned long> position_ids, int num_of_objects, unsigned long alterantive_id, unsigned long action_type);
     void CopFeedBack(long primitive, double  evaluation, unsigned long error_id);
   private:
     ros::Subscriber     m_readAnswer;
     ros::Publisher      m_pubFeedBack;
     ros::ServiceClient  m_client;
     ros::ServiceClient  m_jlo_client;
     std::string         m_answerTopic;
     std::map<unsigned long, std::vector<vision_msgs::cop_answer> > m_msg_cluster;

     void callback(const boost::shared_ptr<const vision_msgs::cop_answer> &msg);

};

#endif /*COP_CLIENT_H*/
