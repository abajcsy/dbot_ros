#include <vector>
#include <cmath>

#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/btMatrix3x3.h>

#include <tf/transform_broadcaster.h>

#define OBJ_TOPIC "/rbc_particle_filter_object_tracker/object_model"
#define KINECT_FRAME "/XTION_RGB"
#define POLE_ANGLE_TOPIC "pole_angle"

#define PI 3.14159265359
#define CALIB_EPSILON 0.03

class PoleTrackerRT{

private:
   ros::NodeHandle nh;
   ros::Subscriber object_sub;
   ros::Publisher pole_angle_pub;

   // for keeping basic metrics about angle measurements
   double num_measurements;
   double angle_sum;
   std::vector<btScalar> angle_data;

   ros::Time start;
   ros::Time end;

public:

   PoleTrackerRT::PoleTrackerRT(){

   }

   bool PoleTrackerRT::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n) {
     ...

     realtime_pub = new realtime_tools::RealtimePublisher<mgs_type>(n, "topic", 4);
     return true;
   }


   void PoleTrackerRT::update() {
     if (realtime_pub->trylock()){
       realtime_pub->msg_.a_field = "hallo";
       realtime_pub->msg_.header.stamp = ros::Time::now();
       realtime_pub->unlockAndPublish();
     }
     ...
   }`

};


int main(int argc, char** argv) {

   rosrt::init(argc, argv, "pole_tracker_rt");
   ros::NodeHandle node_handle;

   PoleTrackerRT driver(node_handle);
   driver.run();
}
