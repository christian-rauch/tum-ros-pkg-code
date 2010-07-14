#include <ros/ros.h>
#include <vision_srvs/srvjlo.h>

using namespace vision_srvs;


#define JLO_IDQUERY "idquery"
#define JLO_FRAMEQUERY "framequery"
#define JLO_DELETE "del"
#define JLO_UPDATE "update"
#define ID_WORLD 1

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "testclient") ;
   srvjlo msg;

  msg.request.command = JLO_FRAMEQUERY;
  if(argc > 2)
    msg.request.query.parent_id = atoi(argv[2]);
  else
    msg.request.query.parent_id = 1;
  if(argc > 1)
    msg.request.query.id = atoi(argv[1]);
  else
    msg.request.query.id = 864;
  if(argc > 15)
    msg.request.query.name = argv[15];
  msg.request.query.type = 0;  
  int width = 4;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
      if(r == c)
        msg.request.query.pose[r * width + c] = 1;
      else
        msg.request.query.pose[r * width + c] = 0;
    }
  }
/*  msg.request.query.pose[3] = 0.0;
  msg.request.query.pose[7] = 0.0;
  msg.request.query.pose[11] = 0.4;*/


/*-0.874767 0.480757 -0.060465 0.050978
-0.296898 -0.433193 0.850997 0.188964
0.382929 0.762375 0.521679 0.777142
0.000000 0.000000 0.000000 1.000000*/
if(argc < 15)
{
  msg.request.query.pose[0] =  0.874767;
  msg.request.query.pose[1] =  0.480757;
  msg.request.query.pose[2] =  0.060465;
  msg.request.query.pose[3] =  0.050978;
  msg.request.query.pose[4] =  0.296898;
  msg.request.query.pose[5] =   -0.433193;
  msg.request.query.pose[6] =  -0.850997;
  msg.request.query.pose[7] =  0.188964; 
  msg.request.query.pose[8] =  -0.382929;
  msg.request.query.pose[9] =  0.762375;
  msg.request.query.pose[10] = -0.521679;
  msg.request.query.pose[11] =  0.777142;
}
else
{
   int i = 0;
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //1
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //2
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++;  //3
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //4  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //5  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //6  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //7  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //8  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //9   
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++; //10  
   msg.request.query.pose[i] = atof(argv[i + 3]);
   i++;  //11
   msg.request.query.pose[i] = atof(argv[i + 3]);
}

  printf("Showing Query with PosId %d with parent %d:\n", (int)msg.request.query.id, (int)msg.request.query.parent_id);
  
    for(int r = 0; r < width; r++)
    {
       for(int c = 0; c < width; c++)
       {
         printf( "%f ", msg.request.query.pose[r * width + c]);
       }
       printf("\n");
    }

  width = 6;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
      if(r == c)
      {
       if(r == 5)
       {
         msg.request.query.cov[r * width + c] = 6.0;
        }
        else
        msg.request.query.cov[r * width + c] = 0.10;
      }
      else
        msg.request.query.cov[r * width + c] = 0;
    }
  }

  /*msg.request.command = JLO_IDQUERY;
  if(argc > 2)
    msg.request.query.parent_id = atoi(argv[2]);
  if(argc > 1)
    msg.request.query.id = atoi(argv[1]);
  else msg.request.query.id = ID_WORLD;*/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<srvjlo>("/located_object", true);
  boost::xtime t0 ,t1;
  boost::xtime_get(&t0, boost::TIME_UTC);
                              int calls = 10000;    
  for(int i =0; i < calls ; i++)
  {
    boost::xtime t2 ,t3;
    boost::xtime_get(&t2, boost::TIME_UTC);
    client.call(msg);
    boost::xtime_get(&t3, boost::TIME_UTC);
    //printf("%dns\n", t3.nsec - t2.nsec);
  }
  boost::xtime_get(&t1, boost::TIME_UTC);   
  printf("%ds %d ms per %d calls\n", t1.sec - t0.sec, (t1.nsec - t0.nsec) / 1000000, calls);  

  int width2 = 4;
  printf("Showing PosId %d with parent %d:\n", (int)msg.response.answer.id, (int)msg.response.answer.parent_id);
  
  for(int r = 0; r < width2; r++)
  {
    for(int c = 0; c < width2; c++)
    {
        printf( "%f ", msg.response.answer.pose[r * width2 + c]);

    }
    printf("\n");
  }
  printf("Type: %d\n", msg.response.answer.type);

  return 0;
}
