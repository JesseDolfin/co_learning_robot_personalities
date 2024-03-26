// Generated by gencpp from file qb_device_srvs/GetMeasurements.msg
// DO NOT EDIT!


#ifndef QB_DEVICE_SRVS_MESSAGE_GETMEASUREMENTS_H
#define QB_DEVICE_SRVS_MESSAGE_GETMEASUREMENTS_H

#include <ros/service_traits.h>


#include <qb_device_srvs/GetMeasurementsRequest.h>
#include <qb_device_srvs/GetMeasurementsResponse.h>


namespace qb_device_srvs
{

struct GetMeasurements
{

typedef GetMeasurementsRequest Request;
typedef GetMeasurementsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetMeasurements
} // namespace qb_device_srvs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::qb_device_srvs::GetMeasurements > {
  static const char* value()
  {
    return "61d005acb1e04c16b9b33b19436d5ede";
  }

  static const char* value(const ::qb_device_srvs::GetMeasurements&) { return value(); }
};

template<>
struct DataType< ::qb_device_srvs::GetMeasurements > {
  static const char* value()
  {
    return "qb_device_srvs/GetMeasurements";
  }

  static const char* value(const ::qb_device_srvs::GetMeasurements&) { return value(); }
};


// service_traits::MD5Sum< ::qb_device_srvs::GetMeasurementsRequest> should match
// service_traits::MD5Sum< ::qb_device_srvs::GetMeasurements >
template<>
struct MD5Sum< ::qb_device_srvs::GetMeasurementsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::qb_device_srvs::GetMeasurements >::value();
  }
  static const char* value(const ::qb_device_srvs::GetMeasurementsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::qb_device_srvs::GetMeasurementsRequest> should match
// service_traits::DataType< ::qb_device_srvs::GetMeasurements >
template<>
struct DataType< ::qb_device_srvs::GetMeasurementsRequest>
{
  static const char* value()
  {
    return DataType< ::qb_device_srvs::GetMeasurements >::value();
  }
  static const char* value(const ::qb_device_srvs::GetMeasurementsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::qb_device_srvs::GetMeasurementsResponse> should match
// service_traits::MD5Sum< ::qb_device_srvs::GetMeasurements >
template<>
struct MD5Sum< ::qb_device_srvs::GetMeasurementsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::qb_device_srvs::GetMeasurements >::value();
  }
  static const char* value(const ::qb_device_srvs::GetMeasurementsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::qb_device_srvs::GetMeasurementsResponse> should match
// service_traits::DataType< ::qb_device_srvs::GetMeasurements >
template<>
struct DataType< ::qb_device_srvs::GetMeasurementsResponse>
{
  static const char* value()
  {
    return DataType< ::qb_device_srvs::GetMeasurements >::value();
  }
  static const char* value(const ::qb_device_srvs::GetMeasurementsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // QB_DEVICE_SRVS_MESSAGE_GETMEASUREMENTS_H
