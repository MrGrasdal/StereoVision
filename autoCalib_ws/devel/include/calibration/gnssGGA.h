// Generated by gencpp from file calibration/gnssGGA.msg
// DO NOT EDIT!


#ifndef CALIBRATION_MESSAGE_GNSSGGA_H
#define CALIBRATION_MESSAGE_GNSSGGA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <calibration/gnssGGA_status.h>

namespace calibration
{
template <class ContainerAllocator>
struct gnssGGA_
{
  typedef gnssGGA_<ContainerAllocator> Type;

  gnssGGA_()
    : header()
    , sat_time()
    , latitude(0.0)
    , lat_direction()
    , longitude(0.0)
    , lon_direction()
    , altitude(0.0)
    , status()  {
    }
  gnssGGA_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , sat_time()
    , latitude(0.0)
    , lat_direction(_alloc)
    , longitude(0.0)
    , lon_direction(_alloc)
    , altitude(0.0)
    , status(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef ros::Time _sat_time_type;
  _sat_time_type sat_time;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _lat_direction_type;
  _lat_direction_type lat_direction;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _lon_direction_type;
  _lon_direction_type lon_direction;

   typedef double _altitude_type;
  _altitude_type altitude;

   typedef  ::calibration::gnssGGA_status_<ContainerAllocator>  _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::calibration::gnssGGA_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::calibration::gnssGGA_<ContainerAllocator> const> ConstPtr;

}; // struct gnssGGA_

typedef ::calibration::gnssGGA_<std::allocator<void> > gnssGGA;

typedef boost::shared_ptr< ::calibration::gnssGGA > gnssGGAPtr;
typedef boost::shared_ptr< ::calibration::gnssGGA const> gnssGGAConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::calibration::gnssGGA_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::calibration::gnssGGA_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::calibration::gnssGGA_<ContainerAllocator1> & lhs, const ::calibration::gnssGGA_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.sat_time == rhs.sat_time &&
    lhs.latitude == rhs.latitude &&
    lhs.lat_direction == rhs.lat_direction &&
    lhs.longitude == rhs.longitude &&
    lhs.lon_direction == rhs.lon_direction &&
    lhs.altitude == rhs.altitude &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::calibration::gnssGGA_<ContainerAllocator1> & lhs, const ::calibration::gnssGGA_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace calibration

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::calibration::gnssGGA_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::calibration::gnssGGA_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::calibration::gnssGGA_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::calibration::gnssGGA_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::calibration::gnssGGA_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::calibration::gnssGGA_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::calibration::gnssGGA_<ContainerAllocator> >
{
  static const char* value()
  {
    return "76d41a768710775209ac34b58a4ce202";
  }

  static const char* value(const ::calibration::gnssGGA_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x76d41a7687107752ULL;
  static const uint64_t static_value2 = 0x09ac34b58a4ce202ULL;
};

template<class ContainerAllocator>
struct DataType< ::calibration::gnssGGA_<ContainerAllocator> >
{
  static const char* value()
  {
    return "calibration/gnssGGA";
  }

  static const char* value(const ::calibration::gnssGGA_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::calibration::gnssGGA_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Something something\n"
"\n"
"Header header\n"
"\n"
"time sat_time\n"
"\n"
"float64 latitude\n"
"string lat_direction\n"
"float64 longitude\n"
"string lon_direction\n"
"\n"
"float64 altitude\n"
"\n"
"gnssGGA_status status\n"
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
"MSG: calibration/gnssGGA_status\n"
"int64 numSat\n"
"float64 hDOP\n"
"int64 mode\n"
"string modeStatus\n"
;
  }

  static const char* value(const ::calibration::gnssGGA_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::calibration::gnssGGA_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.sat_time);
      stream.next(m.latitude);
      stream.next(m.lat_direction);
      stream.next(m.longitude);
      stream.next(m.lon_direction);
      stream.next(m.altitude);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gnssGGA_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::calibration::gnssGGA_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::calibration::gnssGGA_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "sat_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.sat_time);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "lat_direction: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.lat_direction);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "lon_direction: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.lon_direction);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::calibration::gnssGGA_status_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CALIBRATION_MESSAGE_GNSSGGA_H
