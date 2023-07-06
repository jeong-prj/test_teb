// Generated by gencpp from file mbf_msgs/MoveBaseResult.msg
// DO NOT EDIT!


#ifndef MBF_MSGS_MESSAGE_MOVEBASERESULT_H
#define MBF_MSGS_MESSAGE_MOVEBASERESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>

namespace mbf_msgs
{
template <class ContainerAllocator>
struct MoveBaseResult_
{
  typedef MoveBaseResult_<ContainerAllocator> Type;

  MoveBaseResult_()
    : outcome(0)
    , message()
    , dist_to_goal(0.0)
    , angle_to_goal(0.0)
    , final_pose()  {
    }
  MoveBaseResult_(const ContainerAllocator& _alloc)
    : outcome(0)
    , message(_alloc)
    , dist_to_goal(0.0)
    , angle_to_goal(0.0)
    , final_pose(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _outcome_type;
  _outcome_type outcome;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;

   typedef float _dist_to_goal_type;
  _dist_to_goal_type dist_to_goal;

   typedef float _angle_to_goal_type;
  _angle_to_goal_type angle_to_goal;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _final_pose_type;
  _final_pose_type final_pose;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(SUCCESS)
  #undef SUCCESS
#endif
#if defined(_WIN32) && defined(FAILURE)
  #undef FAILURE
#endif
#if defined(_WIN32) && defined(CANCELED)
  #undef CANCELED
#endif
#if defined(_WIN32) && defined(COLLISION)
  #undef COLLISION
#endif
#if defined(_WIN32) && defined(OSCILLATION)
  #undef OSCILLATION
#endif
#if defined(_WIN32) && defined(START_BLOCKED)
  #undef START_BLOCKED
#endif
#if defined(_WIN32) && defined(GOAL_BLOCKED)
  #undef GOAL_BLOCKED
#endif
#if defined(_WIN32) && defined(TF_ERROR)
  #undef TF_ERROR
#endif
#if defined(_WIN32) && defined(INTERNAL_ERROR)
  #undef INTERNAL_ERROR
#endif
#if defined(_WIN32) && defined(PLAN_FAILURE)
  #undef PLAN_FAILURE
#endif
#if defined(_WIN32) && defined(CTRL_FAILURE)
  #undef CTRL_FAILURE
#endif

  enum {
    SUCCESS = 0u,
    FAILURE = 10u,
    CANCELED = 11u,
    COLLISION = 12u,
    OSCILLATION = 13u,
    START_BLOCKED = 14u,
    GOAL_BLOCKED = 15u,
    TF_ERROR = 16u,
    INTERNAL_ERROR = 17u,
    PLAN_FAILURE = 50u,
    CTRL_FAILURE = 100u,
  };


  typedef boost::shared_ptr< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> const> ConstPtr;

}; // struct MoveBaseResult_

typedef ::mbf_msgs::MoveBaseResult_<std::allocator<void> > MoveBaseResult;

typedef boost::shared_ptr< ::mbf_msgs::MoveBaseResult > MoveBaseResultPtr;
typedef boost::shared_ptr< ::mbf_msgs::MoveBaseResult const> MoveBaseResultConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mbf_msgs::MoveBaseResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mbf_msgs::MoveBaseResult_<ContainerAllocator1> & lhs, const ::mbf_msgs::MoveBaseResult_<ContainerAllocator2> & rhs)
{
  return lhs.outcome == rhs.outcome &&
    lhs.message == rhs.message &&
    lhs.dist_to_goal == rhs.dist_to_goal &&
    lhs.angle_to_goal == rhs.angle_to_goal &&
    lhs.final_pose == rhs.final_pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mbf_msgs::MoveBaseResult_<ContainerAllocator1> & lhs, const ::mbf_msgs::MoveBaseResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mbf_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c65d301ffa20590244253c6a99c37c5e";
  }

  static const char* value(const ::mbf_msgs::MoveBaseResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc65d301ffa205902ULL;
  static const uint64_t static_value2 = 0x44253c6a99c37c5eULL;
};

template<class ContainerAllocator>
struct DataType< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mbf_msgs/MoveBaseResult";
  }

  static const char* value(const ::mbf_msgs::MoveBaseResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"# Predefined success codes:\n"
"uint8 SUCCESS        = 0\n"
"\n"
"# Predefined general error codes:\n"
"uint8 FAILURE        = 10\n"
"uint8 CANCELED       = 11\n"
"uint8 COLLISION      = 12\n"
"uint8 OSCILLATION    = 13\n"
"uint8 START_BLOCKED  = 14\n"
"uint8 GOAL_BLOCKED   = 15\n"
"uint8 TF_ERROR       = 16\n"
"uint8 INTERNAL_ERROR = 17\n"
"# 21..49 are reserved for future general error codes\n"
"\n"
"# Planning/controlling failures:\n"
"uint8 PLAN_FAILURE   = 50\n"
"# 51..99 are reserved as planner specific errors\n"
"\n"
"uint8 CTRL_FAILURE   = 100\n"
"# 101..149 are reserved as controller specific errors\n"
"\n"
"uint32 outcome\n"
"string message\n"
"\n"
"# Configuration upon action completion\n"
"float32 dist_to_goal\n"
"float32 angle_to_goal\n"
"geometry_msgs/PoseStamped final_pose\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::mbf_msgs::MoveBaseResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.outcome);
      stream.next(m.message);
      stream.next(m.dist_to_goal);
      stream.next(m.angle_to_goal);
      stream.next(m.final_pose);
    }

    // ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveBaseResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mbf_msgs::MoveBaseResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mbf_msgs::MoveBaseResult_<ContainerAllocator>& v)
  {
    s << indent << "outcome: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.outcome);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    s << indent << "dist_to_goal: ";
    Printer<float>::stream(s, indent + "  ", v.dist_to_goal);
    s << indent << "angle_to_goal: ";
    Printer<float>::stream(s, indent + "  ", v.angle_to_goal);
    s << indent << "final_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.final_pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MBF_MSGS_MESSAGE_MOVEBASERESULT_H