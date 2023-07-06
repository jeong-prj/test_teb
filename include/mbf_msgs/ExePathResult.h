// Generated by gencpp from file mbf_msgs/ExePathResult.msg
// DO NOT EDIT!


#ifndef MBF_MSGS_MESSAGE_EXEPATHRESULT_H
#define MBF_MSGS_MESSAGE_EXEPATHRESULT_H


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
struct ExePathResult_
{
  typedef ExePathResult_<ContainerAllocator> Type;

  ExePathResult_()
    : outcome(0)
    , message()
    , final_pose()
    , dist_to_goal(0.0)
    , angle_to_goal(0.0)  {
    }
  ExePathResult_(const ContainerAllocator& _alloc)
    : outcome(0)
    , message(_alloc)
    , final_pose(_alloc)
    , dist_to_goal(0.0)
    , angle_to_goal(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _outcome_type;
  _outcome_type outcome;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _final_pose_type;
  _final_pose_type final_pose;

   typedef float _dist_to_goal_type;
  _dist_to_goal_type dist_to_goal;

   typedef float _angle_to_goal_type;
  _angle_to_goal_type angle_to_goal;



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
#if defined(_WIN32) && defined(NO_VALID_CMD)
  #undef NO_VALID_CMD
#endif
#if defined(_WIN32) && defined(PAT_EXCEEDED)
  #undef PAT_EXCEEDED
#endif
#if defined(_WIN32) && defined(COLLISION)
  #undef COLLISION
#endif
#if defined(_WIN32) && defined(OSCILLATION)
  #undef OSCILLATION
#endif
#if defined(_WIN32) && defined(ROBOT_STUCK)
  #undef ROBOT_STUCK
#endif
#if defined(_WIN32) && defined(MISSED_GOAL)
  #undef MISSED_GOAL
#endif
#if defined(_WIN32) && defined(MISSED_PATH)
  #undef MISSED_PATH
#endif
#if defined(_WIN32) && defined(BLOCKED_PATH)
  #undef BLOCKED_PATH
#endif
#if defined(_WIN32) && defined(INVALID_PATH)
  #undef INVALID_PATH
#endif
#if defined(_WIN32) && defined(TF_ERROR)
  #undef TF_ERROR
#endif
#if defined(_WIN32) && defined(NOT_INITIALIZED)
  #undef NOT_INITIALIZED
#endif
#if defined(_WIN32) && defined(INVALID_PLUGIN)
  #undef INVALID_PLUGIN
#endif
#if defined(_WIN32) && defined(INTERNAL_ERROR)
  #undef INTERNAL_ERROR
#endif
#if defined(_WIN32) && defined(OUT_OF_MAP)
  #undef OUT_OF_MAP
#endif
#if defined(_WIN32) && defined(MAP_ERROR)
  #undef MAP_ERROR
#endif
#if defined(_WIN32) && defined(STOPPED)
  #undef STOPPED
#endif

  enum {
    SUCCESS = 0u,
    FAILURE = 100u,
    CANCELED = 101u,
    NO_VALID_CMD = 102u,
    PAT_EXCEEDED = 103u,
    COLLISION = 104u,
    OSCILLATION = 105u,
    ROBOT_STUCK = 106u,
    MISSED_GOAL = 107u,
    MISSED_PATH = 108u,
    BLOCKED_PATH = 109u,
    INVALID_PATH = 110u,
    TF_ERROR = 111u,
    NOT_INITIALIZED = 112u,
    INVALID_PLUGIN = 113u,
    INTERNAL_ERROR = 114u,
    OUT_OF_MAP = 115u,
    MAP_ERROR = 116u,
    STOPPED = 117u,
  };


  typedef boost::shared_ptr< ::mbf_msgs::ExePathResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mbf_msgs::ExePathResult_<ContainerAllocator> const> ConstPtr;

}; // struct ExePathResult_

typedef ::mbf_msgs::ExePathResult_<std::allocator<void> > ExePathResult;

typedef boost::shared_ptr< ::mbf_msgs::ExePathResult > ExePathResultPtr;
typedef boost::shared_ptr< ::mbf_msgs::ExePathResult const> ExePathResultConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mbf_msgs::ExePathResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mbf_msgs::ExePathResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mbf_msgs::ExePathResult_<ContainerAllocator1> & lhs, const ::mbf_msgs::ExePathResult_<ContainerAllocator2> & rhs)
{
  return lhs.outcome == rhs.outcome &&
    lhs.message == rhs.message &&
    lhs.final_pose == rhs.final_pose &&
    lhs.dist_to_goal == rhs.dist_to_goal &&
    lhs.angle_to_goal == rhs.angle_to_goal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mbf_msgs::ExePathResult_<ContainerAllocator1> & lhs, const ::mbf_msgs::ExePathResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mbf_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::ExePathResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::ExePathResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::ExePathResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::ExePathResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::ExePathResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::ExePathResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mbf_msgs::ExePathResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b22f308686bb4e3a7364ea944ef68fd0";
  }

  static const char* value(const ::mbf_msgs::ExePathResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb22f308686bb4e3aULL;
  static const uint64_t static_value2 = 0x7364ea944ef68fd0ULL;
};

template<class ContainerAllocator>
struct DataType< ::mbf_msgs::ExePathResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mbf_msgs/ExePathResult";
  }

  static const char* value(const ::mbf_msgs::ExePathResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mbf_msgs::ExePathResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"# Predefined success codes:\n"
"uint8 SUCCESS         = 0\n"
"# 1..9 are reserved as plugin specific non-error results\n"
"\n"
"# Predefined error codes:\n"
"uint8 FAILURE         = 100  # Unspecified failure, only used for old, non-mfb_core based plugins\n"
"uint8 CANCELED        = 101\n"
"uint8 NO_VALID_CMD    = 102\n"
"uint8 PAT_EXCEEDED    = 103\n"
"uint8 COLLISION       = 104\n"
"uint8 OSCILLATION     = 105\n"
"uint8 ROBOT_STUCK     = 106\n"
"uint8 MISSED_GOAL     = 107\n"
"uint8 MISSED_PATH     = 108\n"
"uint8 BLOCKED_PATH    = 109\n"
"uint8 INVALID_PATH    = 110\n"
"uint8 TF_ERROR        = 111\n"
"uint8 NOT_INITIALIZED = 112\n"
"uint8 INVALID_PLUGIN  = 113\n"
"uint8 INTERNAL_ERROR  = 114\n"
"uint8 OUT_OF_MAP      = 115  # The start and / or the goal are outside the map\n"
"uint8 MAP_ERROR       = 116  # The map is not running properly\n"
"uint8 STOPPED         = 117  # The controller execution has been stopped rigorously.\n"
"\n"
"# 121..149 are reserved as plugin specific errors\n"
"\n"
"uint32 outcome\n"
"string message\n"
"\n"
"geometry_msgs/PoseStamped  final_pose\n"
"float32 dist_to_goal\n"
"float32 angle_to_goal\n"
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

  static const char* value(const ::mbf_msgs::ExePathResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mbf_msgs::ExePathResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.outcome);
      stream.next(m.message);
      stream.next(m.final_pose);
      stream.next(m.dist_to_goal);
      stream.next(m.angle_to_goal);
    }

    // ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ExePathResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mbf_msgs::ExePathResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mbf_msgs::ExePathResult_<ContainerAllocator>& v)
  {
    s << indent << "outcome: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.outcome);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    s << indent << "final_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.final_pose);
    s << indent << "dist_to_goal: ";
    Printer<float>::stream(s, indent + "  ", v.dist_to_goal);
    s << indent << "angle_to_goal: ";
    Printer<float>::stream(s, indent + "  ", v.angle_to_goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MBF_MSGS_MESSAGE_EXEPATHRESULT_H
