//
// Created by ej on 23. 7. 18.
//
#include "teb_local_planner/timed_elastic_band.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/recovery_behaviors.h"
#include "global_planning_handler.hpp"
#include "tf2/"

// Params
teb_local_planner::PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
teb_local_planner::ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
teb_local_planner::ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization

boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
teb_local_planner::TebConfig cfg_; //!< Config class that stores and manages all related parameters
teb_local_planner::FailureDetector failure_detector_; //!< Detect if the robot got stucked
tf2_ros::Buffer* tf_;
teb_local_planner::PoseSE2 robot_pose_;

std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan
costmap_2d::Costmap2D* mpo_costmap;

std::string m_worldFrameId = "map";
//std::string m_mapFrameId ;
std::string m_baseFrameId = "base_link";

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
        geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
        geometry_msgs::PoseStamped robot;
        tf2::doTransform(global_pose, robot, global_to_plan_transform);

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
    autoexplorer::GlobalPlanningHandler *mpo_gph = new autoexplorer::GlobalPlanningHandler();
    mpo_gph->reinitialization( mpo_costmap ) ;

    //  Set start position
    geometry_msgs::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0 ;
    geometry_msgs::PoseStamped start = StampedPosefromSE2( p.x, p.y, 0.f );
    start.header.frame_id = m_worldFrameId;

    //  Set goal position
    p.x = 1.0;
    p.y = 1.0;
    p.z = 0.0 ;
    geometry_msgs::PoseStamped goal = StampedPosefromSE2( p.x, p.y, 0.f );
    goal.header.frame_id = m_worldFrameId ;
    //std::vector<geometry_msgs::PoseStamped> plan;

    bool bplansuccess = mpo_gph->makePlan(start, goal, global_plan_);

    std::cout << bplansuccess <<std::endl;
    // In teb_local_planner_ros -> compute velocity commands
    // 1. teb_local_planner_ros -> prune global plan
    // Get robot pose from start pose
    geometry_msgs::PoseStamped robot_pose;
    robot_pose = start;
    robot_pose_ = teb_local_planner::PoseSE2(robot_pose.pose);

    pruneGlobalPlan_t(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);


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