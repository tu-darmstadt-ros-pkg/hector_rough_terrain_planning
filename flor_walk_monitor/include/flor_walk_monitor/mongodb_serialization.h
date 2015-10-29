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

#ifndef FLOR_WALK_MONITOR_MONGODB_SERIALIZATION_H__
#define FLOR_WALK_MONITOR_MONGODB_SERIALIZATION_H__

#include <boost/lexical_cast.hpp>

#include <mongo_ros/message_collection.h>
#include <ros/ros.h>

#include <flor_walk_monitor/walk_performance.h>


namespace flor_mongodb
{
// metadata type
template <typename T>
struct const_mongo_entry
  : public std::pair<const std::string&, const T>
{
  const_mongo_entry() {}
  const_mongo_entry(const std::string &name, const T val)
    : std::pair<const std::string&, const T>(name, val) {}
};

template <typename T>
struct mongo_entry
  : public std::pair<std::string, T>
{
  mongo_entry() {}
  mongo_entry(T val)
    : std::pair<std::string, T>(std::string(), val) {}
  mongo_entry(std::string &name, T val)
    : std::pair<std::string, T>(name, val) {}
};


/**
 * Output - write to DB
 */

struct FlorMongoOutStream
  : public mongo_ros::Metadata
{
  FlorMongoOutStream(const mongo_ros::Metadata &other = mongo_ros::Metadata())
    : mongo_ros::Metadata(other)
  {}
};

// generic case
template <class T> FlorMongoOutStream &operator<<(FlorMongoOutStream &out, const const_mongo_entry<T> &val)
{
  out.append(val.first, val.second);
  return out;
}

// specialization
FlorMongoOutStream &operator<<(FlorMongoOutStream &out, const const_mongo_entry<unsigned long int> &val);

template <class T> FlorMongoOutStream &operator<<(FlorMongoOutStream &out, const const_mongo_entry<std::vector<T>& > &val)
{
  out << const_mongo_entry<unsigned long int>(std::string("size"), val.second.size());

  unsigned long int c = 0;
  for (typename std::vector<T>::const_iterator itr = val.second.begin(); itr != val.second.end(); itr++)
    out << const_mongo_entry<T>(boost::lexical_cast<std::string>(c++), *itr);

  return out;
}


/**
 * Input - read from DB
 */

struct FlorMongoInStream
  : public mongo::BSONObjIterator
{
  FlorMongoInStream(const mongo::BSONObjIterator& other)
    : mongo::BSONObjIterator(other)
  {}
};

// generic case
template <class T> FlorMongoInStream &operator>>(FlorMongoInStream &in, mongo_entry<T> &val)
{
  mongo::BSONElement e = in.next();

  if (e.eoo())
  {
    ROS_WARN("FlorMongoInStream: End of object reached, skipping!");
  }
  else if (e.valuesize() != sizeof(T))
  {
    ROS_WARN("FlorMongoInStream: Size of input data (%i) does not match type size (%lu), skipping!", e.valuesize(), sizeof(T));
  }
  else
  {
    val.first = std::string(e.fieldName());
    val.second = *reinterpret_cast<const T*>(e.value());
  }

  return in;
}

// specialization
FlorMongoInStream &operator>>(FlorMongoInStream &in, mongo_entry<unsigned long int> &val);

template <class T> FlorMongoInStream &operator>>(FlorMongoInStream &in, mongo_entry<std::vector<T>& > &val)
{
  mongo::BSONElement e = in.next();

  if (e.eoo())
  {
    ROS_WARN("FlorMongoInStream: End of object reached, skipping!");
  }
  else
  {
    val.first = std::string(e.fieldName());

    val.second.clear();

    mongo_entry<unsigned int long> size;
    in >> size;

    mongo_entry<T> data;
    for (unsigned long int i = 0; i < (unsigned long int)size.second; i++)
    {
      in >> data;
      val.second.push_back(data.second);
    }
  }

  return in;
}

// wrapper
template <class T> FlorMongoInStream &operator>>(FlorMongoInStream &in, T &val)
{
    mongo_entry<T> data;
    in >> data;
    val = data.second;
    return in;
}
}

#endif
