// Generated by gencpp from file map_msgs/GetPointMapROIRequest.msg
// DO NOT EDIT!


#ifndef MAP_MSGS_MESSAGE_GETPOINTMAPROIREQUEST_H
#define MAP_MSGS_MESSAGE_GETPOINTMAPROIREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace map_msgs
{
template <class ContainerAllocator>
struct GetPointMapROIRequest_
{
  typedef GetPointMapROIRequest_<ContainerAllocator> Type;

  GetPointMapROIRequest_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , r(0.0)
    , l_x(0.0)
    , l_y(0.0)
    , l_z(0.0)  {
    }
  GetPointMapROIRequest_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , r(0.0)
    , l_x(0.0)
    , l_y(0.0)
    , l_z(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _r_type;
  _r_type r;

   typedef double _l_x_type;
  _l_x_type l_x;

   typedef double _l_y_type;
  _l_y_type l_y;

   typedef double _l_z_type;
  _l_z_type l_z;





  typedef boost::shared_ptr< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetPointMapROIRequest_

typedef ::map_msgs::GetPointMapROIRequest_<std::allocator<void> > GetPointMapROIRequest;

typedef boost::shared_ptr< ::map_msgs::GetPointMapROIRequest > GetPointMapROIRequestPtr;
typedef boost::shared_ptr< ::map_msgs::GetPointMapROIRequest const> GetPointMapROIRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::map_msgs::GetPointMapROIRequest_<ContainerAllocator1> & lhs, const ::map_msgs::GetPointMapROIRequest_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.r == rhs.r &&
    lhs.l_x == rhs.l_x &&
    lhs.l_y == rhs.l_y &&
    lhs.l_z == rhs.l_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::map_msgs::GetPointMapROIRequest_<ContainerAllocator1> & lhs, const ::map_msgs::GetPointMapROIRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace map_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "895f7e437a9a6dd225316872b187a303";
  }

  static const char* value(const ::map_msgs::GetPointMapROIRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x895f7e437a9a6dd2ULL;
  static const uint64_t static_value2 = 0x25316872b187a303ULL;
};

template<class ContainerAllocator>
struct DataType< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "map_msgs/GetPointMapROIRequest";
  }

  static const char* value(const ::map_msgs::GetPointMapROIRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 r    # if != 0, circular ROI of radius r\n"
"float64 l_x  # if r == 0, length of AABB on x\n"
"float64 l_y  # if r == 0, length of AABB on y\n"
"float64 l_z  # if r == 0, length of AABB on z\n"
;
  }

  static const char* value(const ::map_msgs::GetPointMapROIRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.r);
      stream.next(m.l_x);
      stream.next(m.l_y);
      stream.next(m.l_z);
    }

    // ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetPointMapROIRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::map_msgs::GetPointMapROIRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::map_msgs::GetPointMapROIRequest_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "r: ";
    Printer<double>::stream(s, indent + "  ", v.r);
    s << indent << "l_x: ";
    Printer<double>::stream(s, indent + "  ", v.l_x);
    s << indent << "l_y: ";
    Printer<double>::stream(s, indent + "  ", v.l_y);
    s << indent << "l_z: ";
    Printer<double>::stream(s, indent + "  ", v.l_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAP_MSGS_MESSAGE_GETPOINTMAPROIREQUEST_H
