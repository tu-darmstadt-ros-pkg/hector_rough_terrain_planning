#include <flor_walk_monitor/mongodb_serialization.h>

namespace flor_mongodb
{

/**
 * Output - write to DB
 */

FlorMongoOutStream &operator<<(FlorMongoOutStream &out, const const_mongo_entry<unsigned long int> &val)
{
  out.append(val.first, (long long)val.second);
  return out;
}


/**
 * Input - read from DB
 */

FlorMongoInStream &operator>>(FlorMongoInStream &in, mongo_entry<unsigned long int> &val)
{
  mongo::BSONElement e = in.next();
  val.first = std::string(e.fieldName());
  val.second = (long unsigned int)e.Long();
  return in;
}
}
