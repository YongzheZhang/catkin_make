// Generated by gencpp from file arbotix_msgs/SetupChannelRequest.msg
// DO NOT EDIT!


#ifndef ARBOTIX_MSGS_MESSAGE_SETUPCHANNELREQUEST_H
#define ARBOTIX_MSGS_MESSAGE_SETUPCHANNELREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace arbotix_msgs
{
template <class ContainerAllocator>
struct SetupChannelRequest_
{
  typedef SetupChannelRequest_<ContainerAllocator> Type;

  SetupChannelRequest_()
    : topic_name()
    , pin(0)
    , value(0)
    , rate(0)  {
    }
  SetupChannelRequest_(const ContainerAllocator& _alloc)
    : topic_name(_alloc)
    , pin(0)
    , value(0)
    , rate(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _topic_name_type;
  _topic_name_type topic_name;

   typedef uint8_t _pin_type;
  _pin_type pin;

   typedef uint8_t _value_type;
  _value_type value;

   typedef uint8_t _rate_type;
  _rate_type rate;





  typedef boost::shared_ptr< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetupChannelRequest_

typedef ::arbotix_msgs::SetupChannelRequest_<std::allocator<void> > SetupChannelRequest;

typedef boost::shared_ptr< ::arbotix_msgs::SetupChannelRequest > SetupChannelRequestPtr;
typedef boost::shared_ptr< ::arbotix_msgs::SetupChannelRequest const> SetupChannelRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace arbotix_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'arbotix_msgs': ['/home/zhe/catkin_ws/src/arbotix_ros-indigo-devel/arbotix_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c65e58d8b3b4d406126f6dc829a6011f";
  }

  static const char* value(const ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc65e58d8b3b4d406ULL;
  static const uint64_t static_value2 = 0x126f6dc829a6011fULL;
};

template<class ContainerAllocator>
struct DataType< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "arbotix_msgs/SetupChannelRequest";
  }

  static const char* value(const ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
string topic_name\n\
uint8 pin\n\
uint8 value\n\
uint8 rate\n\
";
  }

  static const char* value(const ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.topic_name);
      stream.next(m.pin);
      stream.next(m.value);
      stream.next(m.rate);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetupChannelRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::arbotix_msgs::SetupChannelRequest_<ContainerAllocator>& v)
  {
    s << indent << "topic_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.topic_name);
    s << indent << "pin: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pin);
    s << indent << "value: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.value);
    s << indent << "rate: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rate);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARBOTIX_MSGS_MESSAGE_SETUPCHANNELREQUEST_H
