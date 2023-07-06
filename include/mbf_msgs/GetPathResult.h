// Generated by gencpp from file mbf_msgs/GetPathResult.msg
// DO NOT EDIT!


#ifndef MBF_MSGS_MESSAGE_GETPATHRESULT_H
#define MBF_MSGS_MESSAGE_GETPATHRESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <nav_msgs/Path.h>

namespace mbf_msgs
{
template <class ContainerAllocator>
struct GetPathResult_
{
  typedef GetPathResult_<ContainerAllocator> Type;

  GetPathResult_()
    : outcome(0)
    , message()
    , path()
    , cost(0.0)  {
    }
  GetPathResult_(const ContainerAllocator& _alloc)
    : outcome(0)
    , message(_alloc)
    , path(_alloc)
    , cost(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _outcome_type;
  _outcome_type outcome;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;

   typedef  ::nav_msgs::Path_<ContainerAllocator>  _path_type;
  _path_type path;

   typedef double _cost_type;
  _cost_type cost;



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
#if defined(_WIN32) && defined(INVALID_START)
  #undef INVALID_START
#endif
#if defined(_WIN32) && defined(INVALID_GOAL)
  #undef INVALID_GOAL
#endif
#if defined(_WIN32) && defined(NO_PATH_FOUND)
  #undef NO_PATH_FOUND
#endif
#if defined(_WIN32) && defined(PAT_EXCEEDED)
  #undef PAT_EXCEEDED
#endif
#if defined(_WIN32) && defined(EMPTY_PATH)
  #undef EMPTY_PATH
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
    FAILURE = 50u,
    CANCELED = 51u,
    INVALID_START = 52u,
    INVALID_GOAL = 53u,
    NO_PATH_FOUND = 54u,
    PAT_EXCEEDED = 55u,
    EMPTY_PATH = 56u,
    TF_ERROR = 57u,
    NOT_INITIALIZED = 58u,
    INVALID_PLUGIN = 59u,
    INTERNAL_ERROR = 60u,
    OUT_OF_MAP = 61u,
    MAP_ERROR = 62u,
    STOPPED = 63u,
  };


  typedef boost::shared_ptr< ::mbf_msgs::GetPathResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mbf_msgs::GetPathResult_<ContainerAllocator> const> ConstPtr;

}; // struct GetPathResult_

typedef ::mbf_msgs::GetPathResult_<std::allocator<void> > GetPathResult;

typedef boost::shared_ptr< ::mbf_msgs::GetPathResult > GetPathResultPtr;
typedef boost::shared_ptr< ::mbf_msgs::GetPathResult const> GetPathResultConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mbf_msgs::GetPathResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mbf_msgs::GetPathResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mbf_msgs::GetPathResult_<ContainerAllocator1> & lhs, const ::mbf_msgs::GetPathResult_<ContainerAllocator2> & rhs)
{
  return lhs.outcome == rhs.outcome &&
    lhs.message == rhs.message &&
    lhs.path == rhs.path &&
    lhs.cost == rhs.cost;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mbf_msgs::GetPathResult_<ContainerAllocator1> & lhs, const ::mbf_msgs::GetPathResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mbf_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::GetPathResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::GetPathResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::GetPathResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::GetPathResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::GetPathResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::GetPathResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mbf_msgs::GetPathResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b737d51aec954f878a4cd57e83f5767c";
  }

  static const char* value(const ::mbf_msgs::GetPathResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb737d51aec954f87ULL;
  static const uint64_t static_value2 = 0x8a4cd57e83f5767cULL;
};

template<class ContainerAllocator>
struct DataType< ::mbf_msgs::GetPathResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mbf_msgs/GetPathResult";
  }

  static const char* value(const ::mbf_msgs::GetPathResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mbf_msgs::GetPathResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"# Predefined success codes:\n"
"uint8 SUCCESS         = 0\n"
"# 1..9 are reserved as plugin specific non-error results\n"
"\n"
"# Possible error codes:\n"
"uint8 FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins\n"
"uint8 CANCELED        = 51  # The action has been canceled by a action client\n"
"uint8 INVALID_START   = 52  #\n"
"uint8 INVALID_GOAL    = 53\n"
"uint8 NO_PATH_FOUND   = 54\n"
"uint8 PAT_EXCEEDED    = 55\n"
"uint8 EMPTY_PATH      = 56\n"
"uint8 TF_ERROR        = 57\n"
"uint8 NOT_INITIALIZED = 58\n"
"uint8 INVALID_PLUGIN  = 59\n"
"uint8 INTERNAL_ERROR  = 60\n"
"uint8 OUT_OF_MAP      = 61\n"
"uint8 MAP_ERROR       = 62\n"
"uint8 STOPPED         = 63  # The planner execution has been stopped rigorously.\n"
"\n"
"# 71..99 are reserved as plugin specific errors\n"
"\n"
"uint32 outcome\n"
"string message\n"
"\n"
"nav_msgs/Path path\n"
"\n"
"float64 cost\n"
"\n"
"\n"
"================================================================================\n"
"MSG: nav_msgs/Path\n"
"#An array of poses that represents a Path for a robot to follow\n"
"Header header\n"
"geometry_msgs/PoseStamped[] poses\n"
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
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
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

  static const char* value(const ::mbf_msgs::GetPathResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mbf_msgs::GetPathResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.outcome);
      stream.next(m.message);
      stream.next(m.path);
      stream.next(m.cost);
    }

    // ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetPathResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mbf_msgs::GetPathResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mbf_msgs::GetPathResult_<ContainerAllocator>& v)
  {
    s << indent << "outcome: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.outcome);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    s << indent << "path: ";
    s << std::endl;
    Printer< ::nav_msgs::Path_<ContainerAllocator> >::stream(s, indent + "  ", v.path);
    s << indent << "cost: ";
    Printer<double>::stream(s, indent + "  ", v.cost);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MBF_MSGS_MESSAGE_GETPATHRESULT_H
