//
// Created by ej on 23. 7. 18.
//
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

#include "teb_local_planner/timed_elastic_band.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/recovery_behaviors.h"

#include "global_planning_handler.hpp"

#include "tf2/impl/utils.h"

#include "mbf_msgs/ExePathResult.h"
#include "costmap_2d/costmap_math.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "transform_data.h"

// Params
teb_local_planner::PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
teb_local_planner::ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
teb_local_planner::ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization

boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
teb_local_planner::TebConfig cfg_; //!< Config class that stores and manages all related parameters
teb_local_planner::FailureDetector failure_detector_; //!< Detect if the robot got stucked
tf2_ros::Buffer* tf_;
teb_local_planner::PoseSE2 robot_goal_; //!< Store current robot goal
teb_local_planner::PoseSE2 robot_pose_;
geometry_msgs::PoseWithCovarianceStamped m_robotpose ; // (w.r.t world)

geometry_msgs::Twist robot_vel_;
geometry_msgs::Twist last_cmd_; //!< Store the last control command generated in computeVelocityCommands()
int no_infeasible_plans_; //!< Store how many times in a row the planner failed to find a feasible plan.
std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan
geometry_msgs::TwistStamped cmd_vel;

uint8_t* mp_cost_translation_table;
nav_msgs::OccupancyGrid m_globalcostmap ;
nav_msgs::OccupancyGrid m_localcostmap;
costmap_2d::Costmap2D* mpo_gcostmap;
costmap_2d::Costmap2D* mpo_lcostmap;
//costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)

std::string m_worldFrameId = "map";
std::string m_mapFrameId = "odom";
std::string m_baseFrameId = "base_link";

std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot
double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
double robot_circumscribed_radius; //!< The radius of the circumscribed circle of the robot

std::string global_frame_ = "odom"; //!< The frame in which the controller will run

bool goal_reached_; //!< store whether the goal is reached or not

TfData map_to_odom;
TfData odom_to_baselink;
TfData map_to_baselink;

teb_local_planner::PoseSE2 goal_;

cv::Point2f gridmap2world( cv::Point img_pt_roi  ){
    float fgx =  static_cast<float>(img_pt_roi.x) * m_globalcostmap.info.resolution + m_globalcostmap.info.origin.position.x  ;
    float fgy =  static_cast<float>(img_pt_roi.y) * m_globalcostmap.info.resolution + m_globalcostmap.info.origin.position.y  ;

    return cv::Point2f( fgx, fgy );
}

cv::Point world2gridmap( cv::Point2f grid_pt){
    float fx = (grid_pt.x - m_globalcostmap.info.origin.position.x) / m_globalcostmap.info.resolution ;
    float fy = (grid_pt.y - m_globalcostmap.info.origin.position.y) / m_globalcostmap.info.resolution ;

    return cv::Point( (int)fx, (int)fy );
}

geometry_msgs::PoseStamped StampedPosefromSE2( const float& x, const float& y, const float& yaw_radian )
{
    geometry_msgs::PoseStamped outPose ;
    outPose.pose.position.x = x ;
    outPose.pose.position.y = y ;

    float c[3] = {0,};
    float s[3] = {0,};
    c[0] = cos(yaw_radian/2) ;
    c[1] = cos(0) ;
    c[2] = cos(0) ;
    s[0] = sin(yaw_radian/2) ;
    s[1] = sin(0) ;
    s[2] = sin(0) ;

    float qout[4] = {0,};
    qout[0] = c[0]*c[1]*c[2] + s[0]*s[1]*s[2];
    qout[1] = c[0]*c[1]*s[2] - s[0]*s[1]*c[2];
    qout[2] = c[0]*s[1]*c[2] + s[0]*c[1]*s[2];
    qout[3] = s[0]*c[1]*c[2] - c[0]*s[1]*s[2];

    outPose.pose.orientation.w = qout[0] ;
    outPose.pose.orientation.x = qout[1] ;
    outPose.pose.orientation.y = qout[2] ;
    outPose.pose.orientation.z = qout[3] ;

    outPose.header.frame_id = m_worldFrameId ;
    //outPose.header.stamp = 0 ;

    return outPose;
}

double getYaw_t(const geometry_msgs::Quaternion& ori){
    tf2::Quaternion q = tf2::impl::toQuaternion(ori);
    return tf2::impl::getYaw(q);
}

void padFootprint_t(std::vector<geometry_msgs::Point>& footprint, double padding)
{
    // pad footprint in place
    for (unsigned int i = 0; i < footprint.size(); i++)
    {
        geometry_msgs::Point& pt = footprint[ i ];
        pt.x += sign0(pt.x) * padding;
        pt.y += sign0(pt.y) * padding;

//        std::cout << pt.x << ", " << pt.y << std::endl;
    }
}

void initialize_t(){
    teb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<teb_local_planner::PointRobotFootprint>();
    planner_ = teb_local_planner::PlannerInterfacePtr(new teb_local_planner::TebOptimalPlanner(cfg_, &obstacles_, robot_model, &via_points_, mpo_lcostmap));
    //need to tf initialize
    //    tf_ = tf2_ros::Buffer();
}

void transform_t(geometry_msgs::PoseStamped data_in, geometry_msgs::PoseStamped& data_out, TfData& p_tf, int mode){
    tf2::Vector3 v = tf2::Vector3(data_in.pose.position.x, data_in.pose.position.y, data_in.pose.position.z);
    tf2::Quaternion r = tf2::Quaternion(data_in.pose.orientation.x, data_in.pose.orientation.y, data_in.pose.orientation.z, data_in.pose.orientation.w);

    //need to fill t_in
//        geometry_msgs::Transform t_in;
    tf2::Transform t_out;

//        tf2::Vector3 t_v = tf2::Vector3(t_in.translation.x, t_in.translation.y, t_in.translation.z);
    if(mode == 0){ // original
        tf2::Vector3 t_v = tf2::Vector3(p_tf.get_tX(), p_tf.get_tY(), 0);
        t_out.setOrigin(t_v);

        // w at the end in the constructor
//      tf2::Quaternion t_q = tf2::Quaternion(t_in.rotation.x, t_in.rotation.y, t_in.rotation.z, t_in.rotation.w);
        tf2::Quaternion t_q = tf2::Quaternion(p_tf.get_rX(), p_tf.get_rY(), p_tf.get_rZ(), p_tf.get_rW());
        t_out.setRotation(t_q);
    }
    else{ // reverse
        tf2::Vector3 t_v = tf2::Vector3(p_tf.get_itX(), p_tf.get_itY(), 0);
        t_out.setOrigin(t_v);

        // w at the end in the constructor
//      tf2::Quaternion t_q = tf2::Quaternion(t_in.rotation.x, t_in.rotation.y, t_in.rotation.z, t_in.rotation.w);
        tf2::Quaternion t_q = tf2::Quaternion(p_tf.get_irX(), p_tf.get_irY(), p_tf.get_irZ(), p_tf.get_irW());
        t_out.setRotation(t_q);
    }


    // multifly transform matrix and input
    tf2::Transform v_out = t_out * tf2::Transform(r, v);
    data_out.pose.position.x = v_out.getOrigin().getX();
    data_out.pose.position.y = v_out.getOrigin().getY();
    data_out.pose.position.z = v_out.getOrigin().getZ();

//        toMsg(v_out.getOrigin(), robot.pose.position);
    tf2::Quaternion in_r = v_out.getRotation();
    geometry_msgs::Quaternion out_r;
    out_r.w = in_r.getW();
    out_r.x = in_r.getX();
    out_r.y = in_r.getY();
    out_r.z = in_r.getZ();
    data_out.pose.orientation = out_r;
}

//bool pruneGlobalPlan_t(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
bool pruneGlobalPlan_t(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
    if (global_plan.empty())
        return true;

    try
    {
        // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
        // Ignore this time
        // lookupTransform for setting the transform matrix
        // Then doTransform for transform
//        geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
//        geometry_msgs::PoseStamped robot;
//        tf2::doTransform(global_pose, robot, global_to_plan_transform);


        // first, find transform matrix
        geometry_msgs::PoseStamped robot;
        // global pose -> robot
        //    map     -> base_link
        transform_t(global_pose, robot, map_to_baselink, 0);
/*        tf2::Vector3 v = tf2::Vector3(global_pose.pose.position.x, global_pose.pose.position.y, global_pose.pose.position.z);
        tf2::Quaternion r = tf2::Quaternion(global_pose.pose.orientation.x, global_pose.pose.orientation.y, global_pose.pose.orientation.z, global_pose.pose.orientation.w);

        //need to fill t_in
//        geometry_msgs::Transform t_in;
        tf2::Transform t_out;

//        tf2::Vector3 t_v = tf2::Vector3(t_in.translation.x, t_in.translation.y, t_in.translation.z);
        tf2::Vector3 t_v = tf2::Vector3(m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y, 0);
        t_out.setOrigin(t_v);

        // w at the end in the constructor
//      tf2::Quaternion t_q = tf2::Quaternion(t_in.rotation.x, t_in.rotation.y, t_in.rotation.z, t_in.rotation.w);
        tf2::Quaternion t_q = tf2::Quaternion(0, 0, 0, 1);
        t_out.setRotation(t_q);

        // multifly transform matrix and input
        tf2::Transform v_out = t_out * tf2::Transform(r, v);

//        toMsg(v_out.getOrigin(), robot.pose.position);
        tf2::Quaternion in_r = v_out.getRotation();
        geometry_msgs::Quaternion out_r;
        out_r.w = in_r.getW();
        out_r.x = in_r.getX();
        out_r.y = in_r.getY();
        out_r.z = in_r.getZ();
        robot.pose.orientation = out_r;*/

        //distance thresh = dist_behind_robot^2
        double dist_thresh_sq = dist_behind_robot*dist_behind_robot;

        // iterate plan until a pose close the robot is found
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
        // iterate
        while (it != global_plan.end())
        {
            //check the distance from robot pose to iterate pose(global plan)
            double dx = robot.pose.position.x - it->pose.position.x;
            double dy = robot.pose.position.y - it->pose.position.y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < dist_thresh_sq)
            {
                erase_end = it;
                break;
            }
            ++it;
        }
        if (erase_end == global_plan.end())
            return false;

        if (erase_end != global_plan.begin())
            global_plan.erase(global_plan.begin(), erase_end);
    }
    catch (const tf::TransformException& ex)
    {
        printf("Cannot prune path since no transform is available: %s\n", ex.what());
        return false;
    }
    return true;
}

bool transformGlobalPlan_t(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                             const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                                             std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) {
    // this method is a slightly modified version of base_local_planner/goal_functions.h
    const geometry_msgs::PoseStamped &plan_pose = global_plan[0];

    transformed_plan.clear();

    try {
        if (global_plan.empty()) {
            printf("Received plan with zero length");
            *current_goal_idx = 0;
            return false;
        }

        //err
        // get plan_to_global_transform from plan frame to global_frame
//        geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(),
//                                                                                      plan_pose.header.frame_id,
//                                                                                      plan_pose.header.stamp,
//                                                                                      plan_pose.header.frame_id,
//                                                                                      ros::Duration(
//                                                                                              cfg_.robot.transform_tolerance));

        //let's get the pose of the robot in the frame of the plan
        geometry_msgs::PoseStamped robot_pose;
        transform_t(global_pose, robot_pose, map_to_baselink, 0);
//        tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

        //we'll discard points on the plan that are outside the local costmap
        double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                         costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
        dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
        // located on the border of the local costmap

//        std::cout <<"dist_threshold: "<<dist_threshold << std::endl;
        int i = 0;
        double sq_dist_threshold = dist_threshold * dist_threshold;
        double sq_dist = 1e10;

        //we need to loop to a point on the plan that is within a certain distance of the robot
        bool robot_reached = false;
        for (int j = 0; j < (int) global_plan.size(); ++j) {
            double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
            double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
//            double x_diff = 0.0- global_plan[j].pose.position.x;
//            double y_diff = 0.0 - global_plan[j].pose.position.y;
            double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

            if (robot_reached && new_sq_dist > sq_dist)
                break;

            if (new_sq_dist < sq_dist) // find closest distance
            {
                sq_dist = new_sq_dist;
                i = j;
                if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
                    robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
            }
        }

        geometry_msgs::PoseStamped newer_pose;

        double plan_length = 0; // check cumulative Euclidean distance along the plan

        //now we'll transform until points are outside of our distance threshold
        while (i < (int) global_plan.size() && sq_dist <= sq_dist_threshold &&
               (max_plan_length <= 0 || plan_length <= max_plan_length)) {
            const geometry_msgs::PoseStamped &pose = global_plan[i];
            // global plan pose -> global?
            // tf problem map ->
            std::cout <<"First tf problem pose: "<< pose.header.frame_id << std::endl;
            transform_t(pose, newer_pose, map_to_odom, 0);
//            tf2::doTransform(pose, newer_pose, plan_to_global_transform);

            std::cout << i <<"nd pose: "<< newer_pose <<std::endl;
            transformed_plan.push_back(newer_pose);

            double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
            double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
            sq_dist = x_diff * x_diff + y_diff * y_diff;

            // caclulate distance to previous pose
            if (i > 0 && max_plan_length > 0)
                plan_length += teb_local_planner::distance_points2d(global_plan[i - 1].pose.position,
                                                                    global_plan[i].pose.position);

            ++i;
        }

        // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
        // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
        if (transformed_plan.empty()) {
            //tf problem
            std::cout <<"Second tf global pose"<< global_plan.back().header.frame_id << std::endl;
            transform_t(global_plan.back(), newer_pose, map_to_odom, 0);
//            tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

            transformed_plan.push_back(newer_pose);

            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx) *current_goal_idx = int(global_plan.size()) - 1;
        } else {
            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx)
                *current_goal_idx = i - 1; // subtract 1, since i was increased once before leaving the loop
        }

        // Return the transformation from the global plan to the global planning frame if desired
//        if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
    }
    catch (tf::LookupException &ex) {
        printf("No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch (tf::ConnectivityException &ex) {
        printf("Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch (tf::ExtrapolationException &ex) {
        printf("Extrapolation Error: %s\n", ex.what());
        if (global_plan.size() > 0)
            printf("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int) global_plan.size(),
                   global_plan[0].header.frame_id.c_str());

        return false;
    }

    return true;
}

//update obstacle on the costmap
void updateObstacleContainerWithCostmap_t()
{
    // Add costmap obstacles if desired
    if (cfg_.obstacles.include_costmap_obstacles)
    {
        Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

        for (unsigned int i=0; i<mpo_lcostmap->getSizeInCellsX()-1; ++i)
        {
            for (unsigned int j=0; j<mpo_lcostmap->getSizeInCellsY()-1; ++j)
            {
                //costmap x -> y
                if (mpo_lcostmap->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
                {
                    Eigen::Vector2d obs;
                    mpo_lcostmap->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));

                    // check if obstacle is interesting (e.g. not far behind the robot)
                    //direction from robot to obstacle
                    Eigen::Vector2d obs_dir = obs-robot_pose_.position();
                    if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
                        continue;

                    obstacles_.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(obs)));
                }
            }
        }
    }
}

// saturate velocity
// adjustment? prune?
void saturateVelocity_t(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_trans, double max_vel_theta,
                                          double max_vel_x_backwards)
{
    //ratio of over max value
    double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
    // Limit translational velocity for forward driving
    if (vx > max_vel_x)
        ratio_x = max_vel_x / vx;

    // limit strafing velocity
    if (vy > max_vel_y || vy < -max_vel_y)
        ratio_y = std::abs(max_vel_y / vy);

    // Limit angular velocity
    if (omega > max_vel_theta || omega < -max_vel_theta)
        ratio_omega = std::abs(max_vel_theta / omega);

    // Limit backwards velocity
    if (max_vel_x_backwards<=0)
    {
        printf("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
    }
    else if (vx < -max_vel_x_backwards)
        ratio_x = - max_vel_x_backwards / vx;

    if (cfg_.robot.use_proportional_saturation)
    {
        double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
        vx *= ratio;
        vy *= ratio;
        omega *= ratio;
    }
    else
    {
        vx *= ratio_x;
        vy *= ratio_y;
        omega *= ratio_omega;
    }

    double vel_linear = std::hypot(vx, vy);//(distance 0,0 -> vx,vy)
    if (vel_linear > max_vel_trans)
    {
        double max_vel_trans_ratio = max_vel_trans / vel_linear;
        vx *= max_vel_trans_ratio;
        vy *= max_vel_trans_ratio;
    }
}

double convertTransRotVelToSteeringAngle_t(double v, double omega, double wheelbase, double min_turning_radius)
{
    if (omega==0 || v==0)
        return 0;

    double radius = v/omega;

    if (fabs(radius) < min_turning_radius)
        radius = double(g2o::sign(radius)) * min_turning_radius;

    return std::atan(wheelbase / radius);
}

bool loadLCostMap( const std::string& gridmapfile){
//    std::cout << "start load grid map "<<std::endl;
    autoexplorer::ifstream ifs_map( gridmapfile );
    if(!ifs_map) return false;

    int nheight ;
    int nwidth ;
    int value ;
    float origx ;
    float origy ;
    float resolution ;

    std::string frame;
    std::string child_frame;

    float t_x, t_y;
    float r_x, r_y, r_z, r_w;

    float t_xI, t_yI;
    float r_xI, r_yI, r_zI, r_wI;


    ifs_map >> nwidth >> nheight  >> origx >> origy >> resolution;
    m_localcostmap.info.height = nheight ;
    m_localcostmap.info.width  = nwidth ;
    m_localcostmap.info.origin.position.x = origx ;
    m_localcostmap.info.origin.position.y = origy ;
    m_localcostmap.info.resolution = resolution ;
    std::cout << "width: "<<nwidth<< ", height: " << nheight<< ", origin X: "<< origx<< ", origin Y: "<< origy<< ", res: "<< resolution <<std::endl;

    // Set Goal
    geometry_msgs::Quaternion q;
    ifs_map >> goal_.x() >> goal_.y();
    ifs_map >> q.x >> q.y >> q.z >> q.w;
    goal_.theta() = getYaw_t(q);
    std::cout << "Set goal: "<<goal_<<std::endl;

    // Set TF map -> odom
    ifs_map >> frame >> child_frame;
    ifs_map >> t_x >> t_y;
    ifs_map >> r_x >> r_y >> r_z >> r_w;
    ifs_map >> child_frame >> frame;
    ifs_map >> t_xI >> t_yI;
    ifs_map >> r_xI >> r_yI >> r_zI >> r_wI;
    map_to_odom.initialize(frame, child_frame, t_x, t_y, r_x, r_y, r_z, r_w);
    map_to_odom.setInverse(t_xI, t_yI, r_xI, r_yI, r_zI, r_wI);
    std::cout << "map to odom \n\t(frame: "<<frame<< ", child_frame: " << child_frame <<std::endl
                << "\t translation X: "<< t_x<< ", translation Y: "<< t_y <<std::endl
                << "\t rotation X: "<< r_x << ", rotation Y: "<< r_y<< ", rotation Z: "<< r_z<< ", rotation W: "<< r_w<<")"<<std::endl
                << "\t Inverse" <<std::endl
                << "\t translation X: "<< t_xI<< ", translation Y: "<< t_yI <<std::endl
                << "\t rotation X: "<< r_xI << ", rotation Y: "<< r_yI<< ", rotation Z: "<< r_zI<< ", rotation W: "<< r_wI<<")"<<std::endl;

    // Set TF odom -> base_link
    ifs_map >> frame >> child_frame;
    ifs_map >> t_x >> t_y;
    ifs_map >> r_x >> r_y >> r_z >> r_w;
    ifs_map >> child_frame >> frame;
    ifs_map >> t_xI >> t_yI;
    ifs_map >> r_xI >> r_yI >> r_zI >> r_wI;
    odom_to_baselink.initialize(frame, child_frame, t_x, t_y, r_x, r_y, r_z, r_w);
    odom_to_baselink.setInverse(t_xI, t_yI, r_xI, r_yI, r_zI, r_wI);
    std::cout << "odom to base link \n\t(frame: "<<frame<< ", child_frame: " << child_frame <<std::endl
              << "\t translation X: "<< t_x<< ", translation Y: "<< t_y <<std::endl
              << "\t rotation X: "<< r_x << ", rotation Y: "<< r_y<< ", rotation Z: "<< r_z<< ", rotation W: "<< r_w<<")"<<std::endl
              << "\t Inverse" <<std::endl
              << "\t translation X: "<< t_xI<< ", translation Y: "<< t_yI <<std::endl
              << "\t rotation X: "<< r_xI << ", rotation Y: "<< r_yI<< ", rotation Z: "<< r_zI<< ", rotation W: "<< r_wI<<")"<<std::endl;

    // Set TF map -> base_link
    ifs_map >> frame >> child_frame;
    ifs_map >> t_x >> t_y;
    ifs_map >> r_x >> r_y >> r_z >> r_w;
    ifs_map >> child_frame >> frame;
    ifs_map >> t_xI >> t_yI;
    ifs_map >> r_xI >> r_yI >> r_zI >> r_wI;
    map_to_baselink.initialize(frame, child_frame, t_x, t_y, r_x, r_y, r_z, r_w);
    map_to_baselink.setInverse(t_xI, t_yI, r_xI, r_yI, r_zI, r_wI);
    std::cout << "map to base link \n\t(frame: "<<frame<< ", child_frame: " << child_frame <<std::endl
              << "\t translation X: "<< t_x<< ", translation Y: "<< t_y <<std::endl
              << "\t rotation X: "<< r_x << ", rotation Y: "<< r_y<< ", rotation Z: "<< r_z<< ", rotation W: "<< r_w<<")"<<std::endl
              << "\t Inverse" <<std::endl
              << "\t translation X: "<< t_xI<< ", translation Y: "<< t_yI <<std::endl
              << "\t rotation X: "<< r_xI << ", rotation Y: "<< r_yI<< ", rotation Z: "<< r_zI<< ", rotation W: "<< r_wI<<")"<<std::endl;

//    std::cout << "Gridmap data? ok?" <<std::endl;
    for( int ridx=0; ridx < nheight; ridx++ )
    {
        for( int cidx=0; cidx < nwidth; cidx++ )
        {
            ifs_map >> value ;
            m_localcostmap.data.push_back(value);
//            std::cout<< int(m_gridmap.data[ridx*nwidth+cidx]) <<", ";
        }
//        std::cout <<std::endl;
    }
    ifs_map.close();
    return true;
}

bool loadGCostMap( const std::string& costmapfile)
{
//    std::cout << "start load cost map "<<std::endl;
    autoexplorer::ifstream ifs_map( costmapfile );

    if(!ifs_map) return false;

    int nheight ;
    int nwidth ;
    int value ;
    float origx ;
    float origy ;
    float res ;
    ifs_map >> nwidth >> nheight >> origx >> origy >> res;
    m_globalcostmap.info.height = nheight ;
    m_globalcostmap.info.width  = nwidth ;
    m_globalcostmap.info.origin.position.x = origx ;
    m_globalcostmap.info.origin.position.y = origy ;
    m_globalcostmap.info.resolution = res ;
    std::cout << "height:  "<<m_globalcostmap.info.height << ", width: "<< m_globalcostmap.info.width<< ", origin X: "
    << m_globalcostmap.info.origin.position.x<< ", origin Y: " << m_globalcostmap.info.origin.position.y<< ", res: "<< m_globalcostmap.info.resolution<< std::endl;
//    std::cout << "costmap data? ok?" <<std::endl;
    for( int ridx=0; ridx < nheight; ridx++ )
    {
        for( int cidx=0; cidx < nwidth; cidx++ )
        {
            ifs_map >> value ;
            m_globalcostmap.data.push_back(value);
//            std::cout<< int(m_globalcostmap.data[ridx*nwidth+cidx]) <<", ";
        }
//        std::cout <<std::endl;
    }
    ifs_map.close();
    return true;
}

geometry_msgs::PoseStamped GetCurrPose ( )
{
    geometry_msgs::PoseStamped outPose ;
    outPose.header = m_robotpose.header ;
    outPose.pose.position.x = m_robotpose.pose.pose.position.x ;
    outPose.pose.position.y = m_robotpose.pose.pose.position.y ;
    outPose.pose.position.z = 0.f ;
    outPose.pose.orientation = m_robotpose.pose.pose.orientation ;

    return outPose;
}

void setCostmap(nav_msgs::OccupancyGrid g_gridmap, std::string costmap){
    float cmresolution, cmstartx, cmstarty;
    uint32_t cmwidth, cmheight;
    std::vector<signed char> cmdata;

    //g_gridmap = m_globalcostmap;
    cmresolution=g_gridmap.info.resolution;
    cmstartx=g_gridmap.info.origin.position.x;
    cmstarty=g_gridmap.info.origin.position.y;
    cmwidth =g_gridmap.info.width;
    cmheight=g_gridmap.info.height;
    cmdata  =g_gridmap.data;
    std::cout << "res: "<<cmresolution << ", start X: "<< cmstartx<< ", start Y: " << cmstarty<< ", width: " << cmwidth<< ", height: "<< cmheight<< std::endl;
    std::cout <<"cmdata size: "<< cmdata.size() <<std::endl;

    // robot pose with map -> base_link
    // By map -> odom -> base_link
    m_robotpose.pose.pose.position.x = map_to_baselink.get_tX();
    m_robotpose.pose.pose.position.y = map_to_baselink.get_tY();


//    if( gmheight == 0 || gmwidth == 0
//        || gmwidth  != cmwidth
//        || gmheight != cmheight)
//    {
//        printf("unreliable grid map input h/w (%d, %d) gcostmap h/w (%d, %d) \n",
//               gmheight, gmwidth,
//               cmheight, cmwidth);
//        return;
//    }

    geometry_msgs::PoseStamped start = GetCurrPose( );
    start.header.frame_id = m_worldFrameId;

    // Set mp_cost_translation_table
    if (mp_cost_translation_table == NULL) {
        mp_cost_translation_table = new uint8_t[101];

        // special values:
        mp_cost_translation_table[0] = 0;  // NO obstacle
        mp_cost_translation_table[99] = 253;  // INSCRIBED obstacle
        mp_cost_translation_table[100] = 254;  // LETHAL obstacle
//		mp_cost_translation_table[-1] = 255;  // UNKNOWN

        // regular cost values scale the range 1 to 252 (inclusive) to fit
        // into 1 to 98 (inclusive).
        for (int i = 1; i < 99; i++) {
            mp_cost_translation_table[i] = uint8_t(((i - 1) * 251 - 1) / 97 + 1);
        }
    }

    if(costmap == "global"){
        mpo_gcostmap = new costmap_2d::Costmap2D();

        mpo_gcostmap->resizeMap( cmwidth, cmheight, cmresolution,
                              cmstartx, cmstarty );

        unsigned char* pmap = mpo_gcostmap->getCharMap() ;

        for(uint32_t ridx = 0; ridx < cmheight; ridx++)
        {
            for(uint32_t cidx=0; cidx < cmwidth; cidx++)
            {
                uint32_t idx = ridx * cmwidth + cidx ;
                signed char val = cmdata[idx];
//            std::cout<< int(cmdata[idx]) <<", ";
                pmap[idx] = val < 0 ? 255 : mp_cost_translation_table[val];
//            std::cout<< int(pmap[idx]) <<", ";
            }
//        std::cout<<std::endl;
        }
    }
    else{
        mpo_lcostmap = new costmap_2d::Costmap2D();

        mpo_lcostmap->resizeMap( cmwidth, cmheight, cmresolution,
                                 cmstartx, cmstarty );

        unsigned char* pmap = mpo_lcostmap->getCharMap() ;

        for(uint32_t ridx = 0; ridx < cmheight; ridx++)
        {
            for(uint32_t cidx=0; cidx < cmwidth; cidx++)
            {
                uint32_t idx = ridx * cmwidth + cidx ;
                signed char val = cmdata[idx];
//            std::cout<< int(cmdata[idx]) <<", ";
                pmap[idx] = val < 0 ? 255 : mp_cost_translation_table[val];
//            std::cout<< int(pmap[idx]) <<", ";
            }
//        std::cout<<std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    printf("Start\n");

    std::cout <<"***"<<std::endl;
    int file_num = 5;
    for (int idx = 0; idx < file_num; ++idx) {
        std::string lcmapfile = "/home/ej/Desktop/test_planner/src/maps/map_lc_" + std::to_string(idx) + ".txt";
        std::cout << "Load LOCAL cost map" << lcmapfile << std::endl;
        if(!loadLCostMap(lcmapfile)){
            std::cout<< " LOCAL cost file is not exist" <<std::endl;
            break;
        }

        std::string gcmapfile = "/home/ej/Desktop/test_planner/src/maps/map_gc_" + std::to_string(idx) + ".txt";
        std::cout << "Load GLOBAL cost map" << gcmapfile << std::endl;
        if(!loadGCostMap(gcmapfile)){
            std::cout<< " GLOBAL cost file is not exist" <<std::endl;
            break;
        }

        std::cout << "Set cost map 2d" << std::endl;
        setCostmap(m_globalcostmap, "global");
        setCostmap(m_localcostmap, "local");

        std::cout << "***" << std::endl;

        // Make Global plan before local planner
        std::cout << "new global handler" << std::endl;
        autoexplorer::GlobalPlanningHandler *mpo_gph = new autoexplorer::GlobalPlanningHandler();
        std::cout << "global handler - reinitialize" << std::endl;
        std::cout << mpo_gcostmap <<std::endl;
        mpo_gph->reinitialization(mpo_gcostmap);

        std::cout << "Reinitialize mpo_gcostmap" << std::endl << std::endl;

        std::cout << "***" << std::endl;

        std::cout << "Initialize teb" << std::endl;
        initialize_t();

        //  Set start position
        geometry_msgs::Point p;
        std::cout << "Set start position" << std::endl;
        geometry_msgs::PoseStamped start;
        p.x = m_robotpose.pose.pose.position.x;
        p.y = m_robotpose.pose.pose.position.y;
        p.z = 0.0;
        start = StampedPosefromSE2(p.x, p.y, 0.f);
        start.header.frame_id = m_worldFrameId;

        //  Set goal position
        std::cout << "Set goal position" << std::endl;
        p.x = goal_.x();
        p.y = goal_.y();
        p.z = 0.0;
        geometry_msgs::PoseStamped goal = StampedPosefromSE2(p.x, p.y, 0.f);
        goal.header.frame_id = m_worldFrameId;
        //std::vector<geometry_msgs::PoseStamped> plan;
        std::cout << "***" << std::endl;
        std::cout << "Check goal" << goal.pose.position.x <<", "<<goal.pose.position.y
                  <<  " goal: "<<goal_.x() <<", " << goal_.y()<<std::endl;

        std::cout << "***" << std::endl;

        std::cout << "Make global plan" << std::endl;
        bool bplansuccess = mpo_gph->makePlan(start, goal, global_plan_);

        if (!bplansuccess) {
            std::cout << "global plan failed" << std::endl;
            return 0;
        }

        std::cout << "Global plan size:" << global_plan_.size() << std::endl << std::endl;
        // In teb_local_planner_ros -> compute velocity commands
        // 0. Set robot velocity
        // Get robot velocity
        geometry_msgs::Point vel;
        vel.x = 0.0;
        vel.y = 0.0;
        vel.z = 0.0;
        geometry_msgs::PoseStamped robot_vel_tf = StampedPosefromSE2(vel.x, vel.y, 0.f);
//    start.header.frame_id = m_baseFrameId;
        robot_vel_.linear.x = robot_vel_tf.pose.position.x;
        robot_vel_.linear.y = robot_vel_tf.pose.position.y;
//    tf2::Quaternion q = tf2::impl::toQuaternion(robot_vel_tf.pose.orientation);
//    robot_vel_.angular.z = tf2::impl::getYaw(q);
        robot_vel_.angular.z = getYaw_t(robot_vel_tf.pose.orientation);

        // 1. teb_local_planner_ros -> prune global plan
        // Get robot pose from start pose
        geometry_msgs::PoseStamped robot_pose;
        robot_pose.pose.position.x = m_robotpose.pose.pose.position.x;
        robot_pose.pose.position.y = m_robotpose.pose.pose.position.y;
        robot_pose.pose.position.z = 0.0;
        robot_pose.pose.orientation.z = 1.0;
        robot_pose.header.frame_id = m_worldFrameId;
        robot_pose_ = teb_local_planner::PoseSE2(robot_pose.pose);

//    pruneGlobalPlan_t(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);
        pruneGlobalPlan_t(robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);
//    std::cout <<"Pruned global plan"<<std::endl;
//    for (int i = 0; i < global_plan_.size(); ++i) {
//        std::cout<< global_plan_[i].pose.position.x <<", " <<global_plan_[i].pose.position.y << std::endl;
//    }

        // 2. Transform global plan
        // Transform global plan to the frame of interest (w.r.t. the local costmap)
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        int goal_idx;
        geometry_msgs::TransformStamped tf_plan_to_global;

        if (!transformGlobalPlan_t(*tf_, global_plan_, robot_pose, *mpo_lcostmap, global_frame_,
                                   cfg_.trajectory.max_global_plan_lookahead_dist,
                                   transformed_plan, &goal_idx, &tf_plan_to_global)) {
            printf("Could not transform the global plan to the frame of the controller");
            std::string message = "Could not transform the global plan to the frame of the controller";
            return mbf_msgs::ExePathResult::INTERNAL_ERROR;
        }
        std::cout << "Transformed global plan" << std::endl;
        for (int i = 0; i < transformed_plan.size(); ++i) {
            std::cout << transformed_plan[i].pose.position.x << ", " << transformed_plan[i].pose.position.y
                      << std::endl;
        }

        // 3. Check goal reached
        geometry_msgs::PoseStamped global_goal;
//    tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
        // tf problem
        std::cout <<"First tf problem pose: "<< global_plan_.back().header.frame_id << std::endl;
        transform_t(global_plan_.back(), global_goal, map_to_odom, 0);
        double dx = global_goal.pose.position.x - robot_pose_.x();
        double dy = global_goal.pose.position.y - robot_pose_.y();
//    tf2::Quaternion q_tmp = tf2::impl::toQuaternion(global_goal.pose.orientation);
//    double delta_orient = g2o::normalize_theta( tf2::impl::getYaw(q_tmp) - robot_pose_.theta() );
        double delta_orient = g2o::normalize_theta(getYaw_t(global_goal.pose.orientation) - robot_pose_.theta());

        // check goal reached with goal tolerance
        if (fabs(std::sqrt(dx * dx + dy * dy)) < cfg_.goal_tolerance.xy_goal_tolerance
            && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
            && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
            //    && (base_local_planner::stopped(base_odom, cfg_.goal_tolerance.theta_stopped_vel, cfg_.goal_tolerance.trans_stopped_vel)
            || cfg_.goal_tolerance.free_goal_vel) {
            goal_reached_ = true;
            return mbf_msgs::ExePathResult::SUCCESS;
        }

        // 4. Configuration backup
//    configureBackupModes_t(transformed_plan, goal_idx);

        // Get current goal point (last point of the transformed plan)
        robot_goal_.x() = transformed_plan.back().pose.position.x;
        robot_goal_.y() = transformed_plan.back().pose.position.y;

        //5. Estimate Local goal orientation
        // Overwrite goal orientation if needed
//    if (cfg_.trajectory.global_plan_overwrite_orientation)
//    {
//        robot_goal_.theta() = estimateLocalGoalOrientation_t(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
//        // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
//        tf2::Quaternion q;
//        q.setRPY(0, 0, robot_goal_.theta());
//        tf2::convert(q, transformed_plan.back().pose.orientation);
//    }
//    else
//    {
        robot_goal_.theta() = getYaw_t(transformed_plan.back().pose.orientation);
//    }

        // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
        if (transformed_plan.size() == 1) // plan only contains the goal
        {
            transformed_plan.insert(transformed_plan.begin(),
                                    geometry_msgs::PoseStamped()); // insert start (not yet initialized)
        }
        transformed_plan.front() = robot_pose; // update start

        // clear currently existing obstacles
        obstacles_.clear();

        // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
//  if (costmap_converter_)
//    updateObstacleContainerWithCostmapConverter();
//  else
        updateObstacleContainerWithCostmap_t();
        // Do not allow config changes during the following optimization step
        boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

        std::cout << "***" << std::endl;

        // Now perform the actual planning
//   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
        std::cout << "Plan teb" << std::endl;
        bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
        if (!success) {
            planner_->clearPlanner(); // force reinitialization for next time
            printf("teb_local_planner was not able to obtain a local plan for the current setting.");

            ++no_infeasible_plans_; // increase number of infeasible solutions in a row
//        time_last_infeasible_plan_ = ros::Time::now();
            last_cmd_ = cmd_vel.twist;
            printf("teb_local_planner was not able to obtain a local plan");
            return mbf_msgs::ExePathResult::NO_VALID_CMD;
        }

        // Check for divergence
        bool diverged = planner_->hasDiverged();
        if (diverged) {
            cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

            // Reset everything to start again with the initialization of new trajectories.
            planner_->clearPlanner();
            printf("TebLocalPlannerROS: the trajectory has diverged. Resetting planner...\n");

            ++no_infeasible_plans_; // increase number of infeasible solutions in a row
//        time_last_infeasible_plan_ = ros::Time::now();
            last_cmd_ = cmd_vel.twist;
            return mbf_msgs::ExePathResult::NO_VALID_CMD;
        } else {
            printf("TebLocalPlannerROS: the trajectory has not diverged. Continue planner...\n");
        }

        // Check feasibility (but within the first few states only)
//    if(cfg_.robot.is_footprint_dynamic)
//    {
//        // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
//        footprint_spec_ = costmap_ros_->getRobotFootprint();
//        costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
//    }

        std::vector<geometry_msgs::Point> points;

        // Loop over 16 angles around a circle making a point each time
        int N = 16;
        geometry_msgs::Point pt;
        for (int i = 0; i < N; ++i) {
            double angle = i * 2 * M_PI / N;
            pt.x = cos(angle) * 0.1;
            pt.y = sin(angle) * 0.1;

            points.push_back(pt);
        }
        footprint_spec_ = points;

        padFootprint_t(footprint_spec_, robot_inscribed_radius_);
        costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

        bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_,
                                                       robot_circumscribed_radius,
                                                       cfg_.trajectory.feasibility_check_no_poses);

        if (!feasible) {
            cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

            // now we reset everything to start again with the initialization of new trajectories.
            planner_->clearPlanner();
            printf("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");

            ++no_infeasible_plans_; // increase number of infeasible solutions in a row
//        time_last_infeasible_plan_ = ros::Time::now();
            last_cmd_ = cmd_vel.twist;
            printf("teb_local_planner trajectory is not feasible");
            return mbf_msgs::ExePathResult::NO_VALID_CMD;
        } else {
            std::cout << "Trajectory feasible\n";
        }

        // Get the velocity command for this sampling interval
        if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                                          cfg_.trajectory.control_look_ahead_poses)) {
            planner_->clearPlanner();
            printf("TebLocalPlannerROS: velocity command invalid. Resetting planner...");
            ++no_infeasible_plans_; // increase number of infeasible solutions in a row
//        time_last_infeasible_plan_ = ros::Time::now();
            last_cmd_ = cmd_vel.twist;
            printf("teb_local_planner velocity command invalid");
            return mbf_msgs::ExePathResult::NO_VALID_CMD;
        }

        // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
        saturateVelocity_t(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                           cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_trans,
                           cfg_.robot.max_vel_theta,
                           cfg_.robot.max_vel_x_backwards);

//    std::cout <<"X: "<< cmd_vel.twist.linear.x<<", Y: "<<  cmd_vel.twist.linear.y <<", Z: "<< cmd_vel.twist.angular.z<< std::endl;
        // convert rot-vel to steering angle if desired (carlike robot).
        // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
        // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
        if (cfg_.robot.cmd_angle_instead_rotvel) // cfg_.robot.cmd_angle_instead_rotvel = false
        {
            cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle_t(cmd_vel.twist.linear.x,
                                                                          cmd_vel.twist.angular.z,
                                                                          cfg_.robot.wheelbase,
                                                                          0.95 * cfg_.robot.min_turning_radius);
            if (!std::isfinite(cmd_vel.twist.angular.z)) {
                cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
                last_cmd_ = cmd_vel.twist;
                planner_->clearPlanner();
                printf("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
                ++no_infeasible_plans_; // increase number of infeasible solutions in a row
//            time_last_infeasible_plan_ = ros::Time::now();
                printf("teb_local_planner steering angle is not finite");
                return mbf_msgs::ExePathResult::NO_VALID_CMD;
            }
        }
        std::cout << "show results" << std::endl;
        planner_->showResults();

        // a feasible solution should be found, reset counter
        no_infeasible_plans_ = 0;

        // store last command (for recovery analysis etc.)
        last_cmd_ = cmd_vel.twist;
        std::cout << idx <<" done " <<std::endl;

        planner_->clearPlanner();
    }
    printf("done!\n");

    return 0;
}