#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>

#define PI M_PI
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

class controllerNode{
  ros::NodeHandle nh;
  ros::Timer heartbeat;
  ros::Publisher propeller_speeds = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);  
  ros::Subscriber desired_state_sub;
  ros::Subscriber current_state_sub;
  ros::Publisher cur_pose_viz = nh.advertise<visualization_msgs::Marker>("cur_pose", 10);     
  // Set our initial shape type to be a cube
  
  // Controller parameters
  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame

  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  int queue_size = 10;
  Eigen::Matrix4d A_inv;
  tf::TransformBroadcaster br;

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){
    
      //  PART 2 |  Initialize ROS callback handlers
     
      desired_state_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 
                  queue_size, &controllerNode::onDesiredState, this);

      current_state_sub = nh.subscribe<nav_msgs::Odometry>("/current_state",
                  queue_size, &controllerNode::onCurrentState, this);

      heartbeat = nh.createTimer(ros::Duration(1 / hz), &controllerNode::controlLoop, this);

      //  PART 6 [NOTE: save this for last] |  Tune your gains!

      kx = 3.0;
      kv = 2.0;             
      kr = 2.0;
      komega = 0.5;
  

      // Initialize constants
      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;

      double c45df = cos(M_PI/4) * d * cf;
      Eigen::Matrix4d A;
      A << cf, cf, cf, cf,
            c45df, c45df, -c45df, -c45df,
            -c45df, c45df, c45df, -c45df,
            cd, -cd, cd, -cd;
      A_inv = A.inverse();
      
  }

   void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& des_state){
      ROS_INFO("published");

      // 3.1 Get the desired position, velocity and acceleration from the in-
      //     coming ROS message and fill in the class member variables xd, vd
      //     and ad accordingly. You can ignore the angular acceleration.

      xd << des_state->transforms[0].translation.x, des_state->transforms[0].translation.y, des_state->transforms[0].translation.z;
      vd << des_state->velocities[0].linear.x, des_state->velocities[0].linear.y, des_state->velocities[0].linear.z;
      ad << des_state->accelerations[0].linear.x, des_state->accelerations[0].linear.y, des_state->accelerations[0].linear.z;
      
      // 3.2 Extract the yaw component from the quaternion in the incoming ROS

      tf::Quaternion tf_q;
      tf::quaternionMsgToTF(des_state->transforms[0].rotation, tf_q);
      yawd = tf::getYaw(tf_q);

      tf::Transform AV1World(tf::Transform::getIdentity());
      AV1World.setOrigin(tf::Vector3(xd[0], xd[1], xd[2]));
      AV1World.setRotation(tf::Quaternion(tf_q.x(), tf_q.y(), tf_q.z(), tf_q.w()));
      br.sendTransform(tf::StampedTransform(AV1World, ros::Time::now(), "world", "av1"));
      

  }

  void onCurrentState(const nav_msgs::Odometry::ConstPtr& cur_state){
      //  PART 4 | Objective: fill in x, v, R and omega

      geometry_msgs::Pose cur_pose = cur_state->pose.pose;
      geometry_msgs::Twist cur_twist = cur_state->twist.twist;
      x << cur_pose.position.x, cur_pose.position.y, cur_pose.position.z;
      v << cur_twist.linear.x, cur_twist.linear.y, cur_twist.linear.z;
      
      Eigen::Quaterniond q_cur(cur_pose.orientation.x, cur_pose.orientation.y,
                            cur_pose.orientation.z, cur_pose.orientation.w);
      R = q_cur.normalized().matrix();
      Eigen::Vector3d omega_world(cur_twist.angular.x, cur_twist.angular.y, cur_twist.angular.z);
      omega = R.transpose() * omega_world;

  }

  void controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ex, ev, er, eomega;

    //  PART 5 | Objective: Implement the controller!

    ex = x - xd;
    ROS_INFO("ex[0] %3.2f", ex[0]);
    ev = v - vd;
    ROS_INFO("ev[0] %3.2f", ev[0]);

    // 5.2 Compute the Rd matrix.

    Eigen::Vector3d b3d = - kx * ex - kv * ev + m * g * e3 + m * ad;
    b3d.normalize();
    Eigen::Vector3d b1d(cos(yawd-M_PI/4), sin(yawd-M_PI/4), 0.0);// 坐标系b不同
    Eigen::Vector3d b2d = b3d.cross(b1d).normalized();
    Eigen::Matrix3d Rd;
    Rd << b2d.cross(b3d), b2d, b3d;

    // 5.3 Compute the orientation error (er) and the rotation-rate error (eomega)

    er = Vee(Rd.transpose() * R - R.transpose() * Rd) / 2;
    ROS_INFO("er[0] %3.2f", er[0]);
    // eomega = omega - R.transpose() * Rd * b1d;
    eomega = omega;
    ROS_INFO("eomega[0] %3.2f", eomega[0]);

    // 5.4 Compute the desired wrench (force + torques) to control the UAV.
 
    double f_force = (- kx * ex - kv * ev + m * g * e3 + m * ad).dot(R * e3);
    Eigen::Vector3d M_torques = -kr * er - komega * eomega + omega.cross(J * omega);

    ROS_INFO("kr %3.2f", kr);
    ROS_INFO("M_torques[0] %3.2f", M_torques[0]);
    ROS_INFO("M_torques[1] %3.2f", M_torques[1]);

    // 5.5 Recover the rotor speeds from the wrench computed above
 
    Eigen::Vector4d fM;
    fM << f_force, M_torques;
    ROS_INFO("fM[1] %3.2f", fM[1]);
    ROS_INFO("A_inv[1] %3.2f", A_inv.data()[1]);
    Eigen::Vector4d fi = A_inv * fM;
    ROS_INFO("fi[0] %3.2f", fi[0]);


    // 5.6 Populate and publish the control message

    std::vector<double> vel(4);
    vel[0] = signed_sqrt(fi[0]);
    vel[1] = signed_sqrt(fi[1]);
    vel[2] = signed_sqrt(fi[2]);
    vel[3] = signed_sqrt(fi[3]);
    ROS_INFO("vel[0] %3.2f", vel[0]);
    ROS_INFO("vel[1] %3.2f", vel[1]);
    ROS_INFO("vel[2] %3.2f", vel[2]);
    ROS_INFO("vel[3] %3.2f", vel[3]);
    mav_msgs::Actuators msg;
    msg.angular_velocities = vel;
    propeller_speeds.publish(msg);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  controllerNode n;
  ROS_INFO("ASTTTTTTTTTTTTTTTTTT");
  ros::spin();
}
