//
// Created by ej on 23. 7. 18.
//
#include "teb_local_planner/timed_elastic_band.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/recovery_behaviors.h"

// Params
teb_local_planner::PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
teb_local_planner::ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
teb_local_planner::ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization

boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
teb_local_planner::TebConfig cfg_; //!< Config class that stores and manages all related parameters
teb_local_planner::FailureDetector failure_detector_; //!< Detect if the robot got stucked

std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan

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

int main(int argc, char** argv)
{
    printf("start\n");
    initialize_t();

    // In teb_local_planner_ros -> compute velocity commands
    // 1. teb_local_planner_ros -> prune global plan
    pruneGlobalPlan_t();



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