//=================================================================================================
// Copyright (c) 2013, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef FLOR_WALK_MONITOR_MONGODB_H__
#define FLOR_WALK_MONITOR_MONGODB_H__

#include <mongo_ros/message_collection.h>
#include <mongo_ros/exceptions.h>
#include <ros/ros.h>

//#include <flor_walk_monitor/mongodb_serialization.h>
#include <flor_walk_monitor/walk_performance.h>


namespace flor_walk_monitor
{
typedef mongo_ros::MessageWithMetadata<flor_walk_monitor::walk_performance> WalkPerformanceWithMetadata;
typedef WalkPerformanceWithMetadata::ConstPtr WalkPerformanceMetaPtr;

// Helper function that creates metadata for a message.
mongo_ros::Metadata makeMetadata(const walk_performance& data)
{
  /// TODO: walltime seems not to work as expected
  return mongo_ros::Metadata("time", (long long int)ros::WallTime().now().toNSec(), "plan_size", (long long int)data.planned_footsteps.size(), "has_fallen", data.has_fallen);
}

class WalkMonitorMongodb
  : public mongo_ros::MessageCollection<walk_performance>
{
public:
  WalkMonitorMongodb(const std::string& db_name, const std::string& db_host, unsigned int db_port = 0, float timeout = 300);
  virtual ~WalkMonitorMongodb();

  bool insert(const walk_performance &data);

  void getAllData(std::list<walk_performance> &data) const;
  bool getLatestEntry(walk_performance &data) const;

  void printPerformanceDataInfo(const flor_walk_monitor::walk_performance &data) const;

  void test();

  // typedefs
  typedef boost::shared_ptr<WalkMonitorMongodb> Ptr;
  typedef boost::shared_ptr<const WalkMonitorMongodb> ConstPtr;
};
}

#endif
