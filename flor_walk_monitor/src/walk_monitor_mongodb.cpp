#include <flor_walk_monitor/walk_monitor_mongodb.h>

namespace flor_walk_monitor
{
WalkMonitorMongodb::WalkMonitorMongodb(const std::string &db_name, const std::string &db_host, unsigned int db_port, float timeout)
  : mongo_ros::MessageCollection<walk_performance>(db_name, "walk_performances", db_host, db_port, timeout)
{
/// TODO: implement indexing
}

WalkMonitorMongodb::~WalkMonitorMongodb()
{
}

bool WalkMonitorMongodb::insert(const walk_performance &data)
{
  try
  {
    mongo_ros::MessageCollection<walk_performance>::insert(data, makeMetadata(data));

    ROS_INFO("------------------");
    ROS_INFO("Inserted data:");
    printPerformanceDataInfo(data);

  }
  catch (mongo::DBException e)
  {
    ROS_ERROR("Can't insert into mongo db: %s", e.what());
    return false;
  }

  return true;
}

void WalkMonitorMongodb::getAllData(std::list<walk_performance> &data) const
{
  data.clear();

  mongo_ros::Query q;
  std::vector<WalkPerformanceMetaPtr> result = mongo_ros::MessageCollection<walk_performance>::pullAllResults(q);

  for (std::vector<WalkPerformanceMetaPtr>::const_iterator itr = result.begin(); itr != result.end(); itr++)
  {
    const WalkPerformanceMetaPtr &entry = *itr;
    data.push_back(*entry);

    ROS_INFO("------------------");
    ROS_INFO("%s", entry->metadata.toString().c_str());
    ROS_INFO("Loaded data:");
    printPerformanceDataInfo(*entry);
  }
}

bool WalkMonitorMongodb::getLatestEntry(walk_performance &data) const
{
//  // Test findOne
//  mongo_ros::Query q4("name", "bar");
//  EXPECT_EQ(p1, *coll.findOne(q4, false));
//  EXPECT_DOUBLE_EQ(24, coll.findOne(q4, true)->lookupDouble("x"));

  mongo_ros::Query q;

  /// TODO: implement here something useful

  ROS_INFO("------------------");
  ROS_INFO("Loaded data:");
  printPerformanceDataInfo(data);
  return false;
}

void WalkMonitorMongodb::printPerformanceDataInfo(const flor_walk_monitor::walk_performance &data) const
{
//    ROS_INFO("Atlas sim interface log: %lu", data.atlas_sim_interface_log.size());
//    ROS_INFO("Controller mode log: %lu", data.controller_mode_log.size());
//    ROS_INFO("Robot stability log: %lu", data.robot_stability_log.size());

//    ROS_INFO("Footstep plan size: %lu", data.planned_footsteps.size());
//    ROS_INFO("BDI plan size: %lu", data.bdi_planned_footsteps.size());
//    ROS_INFO("Real footsteps size: %lu", data.real_footsteps.size());

//    ROS_INFO("Atlas positions size: %lu", data.atlas_position_data.size());
//    ROS_INFO("Robot stability size: %lu", data.robot_stability.size());
//    ROS_INFO("Robot stability worst rating size: %lu", data.stability_worst_rating.size());
}

void WalkMonitorMongodb::test()
{
//  // Symbols used in queries to denote binary predicates for < and >
//  // Note that equality is the default, so we don't need a symbol for it
//  using mongo_ros::LT;
//  using mongo_ros::GT;

//  // Clear existing data if any
//  mongo_ros::dropDatabase("my_db", "localhost", 27019, 60.0);

//  // Set up db
//  mongo_ros::MessageCollection<geometry_msgs::Pose> coll("my_db", "poses", "localhost", 27019, 60.0);

//  // Arrange to index on metadata fields 'x' and 'name'
//  coll.ensureIndex("name");
//  coll.ensureIndex("x");

//  // Add some poses and metadata
//  const geometry_msgs::Pose p1 = makePose(24, 42, 0);
//  const geometry_msgs::Pose p2 = makePose(10, 532, 3);
//  const geometry_msgs::Pose p3 = makePose(53, 22, 5);
//  const geometry_msgs::Pose p4 = makePose(22, -5, 33);
//  coll.insert(p1, makeMetadata(p1, "bar"));
//  coll.insert(p2, makeMetadata(p2, "baz"));
//  coll.insert(p3, makeMetadata(p3, "qux"));
//  coll.insert(p1, makeMetadata(p1, "oof"));
//  coll.insert(p4, makeMetadata(p4, "ooof"));
//  EXPECT_EQ(5u, coll.count());

//  // Simple query: find the pose with name 'qux' and return just its metadata
//  // Since we're doing an equality check, we don't explicitly specify a predicate
//  vector<PoseMetaPtr> res = coll.pullAllResults(mongo_ros::Query("name", "qux"), true);
//  EXPECT_EQ(1u, res.size());
//  EXPECT_EQ("qux", res[0]->lookupString("name"));
//  EXPECT_DOUBLE_EQ(53, res[0]->lookupDouble("x"));

//  // Set up query: position.x < 40 and position.y > 0.  Reverse order
//  // by the "name" metadata field.  Also, here we pull the message itself, not
//  // just the metadata.  Finally, we can't use the simplified construction
//  // syntax here because it's too long
//  mongo_ros::Query q = mongo_ros::Query().append("x", mongo_ros::LT, 40).append("y", mongo_ros::GT, 0);
//  vector<PoseMetaPtr> poses = coll.pullAllResults(q, false, "name", false);

//  // Verify poses.
//  EXPECT_EQ(3u, poses.size());
//  EXPECT_EQ(p1, *poses[0]);
//  EXPECT_EQ(p2, *poses[1]);
//  EXPECT_EQ(p1, *poses[2]);

//  EXPECT_EQ("oof", poses[0]->lookupString("name"));
//  EXPECT_EQ("baz", poses[1]->lookupString("name"));
//  EXPECT_EQ("bar", poses[2]->lookupString("name"));

//  // Set up query to delete some poses.
//  mongo_ros::Query q2 ("y", mongo_ros::LT, 30);

//  EXPECT_EQ(5u, coll.count());
//  EXPECT_EQ(2u, coll.removeMessages(q2));
//  EXPECT_EQ(3u, coll.count());

//  // Test findOne
//  mongo_ros::Query q4("name", "bar");
//  EXPECT_EQ(p1, *coll.findOne(q4, false));
//  EXPECT_DOUBLE_EQ(24, coll.findOne(q4, true)->lookupDouble("x"));

//  mongo_ros::Query q5("name", "barbar");
//  EXPECT_THROW(coll.findOne(q5, true), mongo_ros::NoMatchingeometry_msgsessageException);
//  EXPECT_THROW(coll.findOne(q5, false), mongo_ros::NoMatchingeometry_msgsessageException);

//  // Test update
//  coll.modifyMetadata(q4, mongo_ros::Metadata("name", "barbar"));
//  EXPECT_EQ(3u, coll.count());
//  EXPECT_THROW(coll.findOne(q4, false), mongo_ros::NoMatchingeometry_msgsessageException);
//  ROS_INFO("here");
//  EXPECT_EQ(p1, *coll.findOne(q5, false));

//  // Check stored metadata
//  boost::shared_ptr<mongo::DBClientConnection> conn =
//    mongo_ros::makeDbConnection(ros::NodeHandle());
//  EXPECT_EQ("geometry_msgs/Pose", mongo_ros::messageType(*conn, "my_db", "poses"));
}
}
