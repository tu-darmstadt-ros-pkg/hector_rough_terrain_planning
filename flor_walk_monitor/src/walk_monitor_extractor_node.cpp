#include <flor_walk_monitor/walk_monitor_extractor_node.h>

namespace flor_walk_monitor
{
WalkMonitorExtractorNode::WalkMonitorExtractorNode()
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  nh_.param("log_folder", log_folder, std::string(""));
  if (log_folder == std::string("")) {
    ROS_ERROR("No log_folder specified via parameter, exiting!");
    ros::shutdown();
  }

  ROS_INFO("HIER");

  // load db settings
  std::string db_name;
  std::string db_host;
  unsigned int db_port;

  nh_.param("mongodb/db_name", db_name, std::string(""));
  nh_.param("mongodb/db_host", db_host, std::string(""));
  nh_.param("mongodb/db_port", (int&)db_port, 0);

  // init db connection
  try
  {
    mongodb = new WalkMonitorMongodb(db_name, db_host, db_port);
  }
  catch (mongo::DBException e)
  {
    ROS_ERROR("Can't connect to mongo db: %s", e.what());
    exit(1);
  }
}

WalkMonitorExtractorNode::~WalkMonitorExtractorNode()
{
  if (mongodb)
    delete mongodb;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flor_walk_monitor_extractor");

  flor_walk_monitor::WalkMonitorExtractorNode monitorExtractorNode;

  ros::spin();

  return 0;
}
