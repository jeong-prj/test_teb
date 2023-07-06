#include <gtest/gtest.h>

#include "teb_local_planner/timed_elastic_band.h"

int main(int argc, char** argv)
{
    printf("??");

  double dt = 0.1;
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
//  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
//  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
//    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
//    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
//  }

    return 0;
}