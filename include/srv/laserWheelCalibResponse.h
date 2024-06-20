// Generated by gencpp from file calib_fusion_2d/laserWheelCalibResponse.msg
// DO NOT EDIT!


#ifndef CALIB_FUSION_2D_MESSAGE_LASERWHEELCALIBRESPONSE_H
#define CALIB_FUSION_2D_MESSAGE_LASERWHEELCALIBRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace calib_fusion_2d
{
template <class ContainerAllocator>
struct laserWheelCalibResponse_
{
  typedef laserWheelCalibResponse_<ContainerAllocator> Type;

  laserWheelCalibResponse_()
    : success(false)  {
    }
  laserWheelCalibResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> const> ConstPtr;

}; // struct laserWheelCalibResponse_

typedef ::calib_fusion_2d::laserWheelCalibResponse_<std::allocator<void> > laserWheelCalibResponse;

typedef boost::shared_ptr< ::calib_fusion_2d::laserWheelCalibResponse > laserWheelCalibResponsePtr;
typedef boost::shared_ptr< ::calib_fusion_2d::laserWheelCalibResponse const> laserWheelCalibResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator1> & lhs, const ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator1> & lhs, const ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace calib_fusion_2d

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "calib_fusion_2d/laserWheelCalibResponse";
  }

  static const char* value(const ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
;
  }

  static const char* value(const ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct laserWheelCalibResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::calib_fusion_2d::laserWheelCalibResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CALIB_FUSION_2D_MESSAGE_LASERWHEELCALIBRESPONSE_H