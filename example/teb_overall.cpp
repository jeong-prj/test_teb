//
// Created by ej on 23. 7. 18.
//
#include "teb_local_planner/timed_elastic_band.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/recovery_behaviors.h"
#include "global_planning_handler.hpp"
#include "tf2/impl/utils.h"
#include "mbf_msgs/ExePathResult.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Params
teb_local_planner::PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
teb_local_planner::ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
teb_local_planner::ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization

boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
teb_local_planner::TebConfig cfg_; //!< Config class that stores and manages all related parameters
teb_local_planner::FailureDetector failure_detector_; //!< Detect if the robot got stucked
tf2_ros::Buffer* tf_;
teb_local_planner::PoseSE2 robot_pose_;
geometry_msgs::Twist robot_vel_;

std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan
costmap_2d::Costmap2D* mpo_costmap;
//costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)

std::string m_worldFrameId = "map";
//std::string m_mapFrameId ;
std::string m_baseFrameId = "base_link";

std::string global_frame_; //!< The frame in which the controller will run

bool goal_reached_; //!< store whether the goal is reached or not

void initialize_t(){
    teb_local_planner::RobotFootprintModelPtr robot_model = boost::make_shared<teb_local_planner::PointRobotFootprint>();
    planner_ = teb_local_planner::PlannerInterfacePtr(new teb_local_planner::TebOptimalPlanner(cfg_, &obstacles_, robot_model, &via_points_));
}

bool pruneGlobalPlan_t(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
    if (global_plan.empty())
        return true;

    try
    {
        // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
        // Ignore this time
//        geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
//        geometry_msgs::PoseStamped robot;
//        tf2::doTransform(global_pose, robot, global_to_plan_transform);
        geometry_msgs::PoseStamped robot;
        robot = global_pose;

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

        // get plan_to_global_transform from plan frame to global_frame
        geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(),
                                                                                      plan_pose.header.frame_id,
                                                                                      plan_pose.header.stamp,
                                                                                      plan_pose.header.frame_id,
                                                                                      ros::Duration(
                                                                                              cfg_.robot.transform_tolerance));

        //let's get the pose of the robot in the frame of the plan
        geometry_msgs::PoseStamped robot_pose;
        tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

        //we'll discard points on the plan that are outside the local costmap
        double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                         costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
        dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
        // located on the border of the local costmap


        int i = 0;
        double sq_dist_threshold = dist_threshold * dist_threshold;
        double sq_dist = 1e10;

        //we need to loop to a point on the plan that is within a certain distance of the robot
        bool robot_reached = false;
        for (int j = 0; j < (int) global_plan.size(); ++j) {
            double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
            double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
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
            tf2::doTransform(pose, newer_pose, plan_to_global_transform);

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
            tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

            transformed_plan.push_back(newer_pose);

            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx) *current_goal_idx = int(global_plan.size()) - 1;
        } else {
            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx)
                *current_goal_idx = i - 1; // subtract 1, since i was increased once before leaving the loop
        }

        // Return the transformation from the global plan to the global planning frame if desired
        if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
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

int main(int argc, char** argv)
{
    printf("start\n");
    initialize_t();

    // Make Global plan before local planner
    std::cout << "new global handler" <<std::endl;
    autoexplorer::GlobalPlanningHandler *mpo_gph = new autoexplorer::GlobalPlanningHandler();
    std::cout << "global handler- reinitialize" <<std::endl;
    mpo_gph->reinitialization( mpo_costmap ) ;

    //  Set start position
    std::cout << "Set start position" <<std::endl;
    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0 ;
    geometry_msgs::PoseStamped start = StampedPosefromSE2( p.x, p.y, 0.f );
    start.header.frame_id = m_worldFrameId;

    //  Set goal position
    std::cout << "Set goal position" <<std::endl;
    p.x = 1.0;
    p.y = 1.0;
    p.z = 0.0 ;
    geometry_msgs::PoseStamped goal = StampedPosefromSE2( p.x, p.y, 0.f );
    goal.header.frame_id = m_worldFrameId ;
    //std::vector<geometry_msgs::PoseStamped> plan;

    std::cout << "Make global plan" <<std::endl;
    bool bplansuccess = mpo_gph->makePlan(start, goal, global_plan_);

    std::cout << bplansuccess <<std::endl;
    // In teb_local_planner_ros -> compute velocity commands
    // 0. Set robot velocity
    // Get robot velocity
    geometry_msgs::Point vel;
    vel.x = 0.0;
    vel.y = 0.0;
    vel.z = 0.0 ;
    geometry_msgs::PoseStamped robot_vel_tf = StampedPosefromSE2( vel.x, vel.y, 0.f );
    start.header.frame_id = m_worldFrameId;
    robot_vel_.linear.x = robot_vel_tf.pose.position.x;
    robot_vel_.linear.y = robot_vel_tf.pose.position.y;
    tf2::Quaternion q = tf2::impl::toQuaternion(robot_vel_tf.pose.orientation);
    robot_vel_.angular.z = tf2::impl::getYaw(q);

    // 1. teb_local_planner_ros -> prune global plan
    // Get robot pose from start pose
    geometry_msgs::PoseStamped robot_pose;
    robot_pose = start;
    robot_pose_ = teb_local_planner::PoseSE2(robot_pose.pose);

    pruneGlobalPlan_t(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

    // 2. Transform global plan
    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    int goal_idx;
    geometry_msgs::TransformStamped tf_plan_to_global;

//    if (!transformGlobalPlan_t(*tf_, global_plan_, robot_pose, *mpo_costmap, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist,
//                             transformed_plan, &goal_idx, &tf_plan_to_global))
//    {
//        printf("Could not transform the global plan to the frame of the controller");
//        std::string message = "Could not transform the global plan to the frame of the controller";
//        return mbf_msgs::ExePathResult::INTERNAL_ERROR;
//    }

    // 3. Check goal reached
    geometry_msgs::PoseStamped global_goal;
    tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
    double dx = global_goal.pose.position.x - robot_pose_.x();
    double dy = global_goal.pose.position.y - robot_pose_.y();
    tf2::Quaternion q_tmp = tf2::impl::toQuaternion(global_goal.pose.orientation);
    double delta_orient = g2o::normalize_theta( tf2::impl::getYaw(q_tmp) - robot_pose_.theta() );

    // check goal reached with goal tolerance
    if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
       && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
       && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
       //    && (base_local_planner::stopped(base_odom, cfg_.goal_tolerance.theta_stopped_vel, cfg_.goal_tolerance.trans_stopped_vel)
       || cfg_.goal_tolerance.free_goal_vel)
    {
        goal_reached_ = true;
        return mbf_msgs::ExePathResult::SUCCESS;
    }



    /*double dt = 0.1;
    double dt_hysteresis = dt/3.;
    teb_local_planner::TimedElasticBand teb;

    teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
    for (int i = 1; i < 10; ++i) {
        teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
    }
    // add a pose with a large timediff as the last one
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt + 2*dt_hysteresis);
//
//  // auto resize + test of the result
    teb.autoResize(dt, dt_hysteresis, 3, 100, false);
    for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
        if(teb.TimeDiff(i) >= dt + dt_hysteresis + 1e-3)
            std::cout << "dt is greater than allowed: " << i<<std::endl;
        else if(dt - dt_hysteresis - 1e-3 >= teb.TimeDiff(i))
            std::cout << "dt is less than allowed: " << i <<std::endl;
        else
            std::cout << "allowed\n" ;
    }*/
    printf("done!\n");

    return 0;
}