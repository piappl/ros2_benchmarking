// generated from rosidl_typesupport_opensplice_c/resource/msg__type_support_c.cpp.template

#include <cassert>
#include <limits>

#include <u_instanceHandle.h>

// Provides the definition of the rosidl_message_type_support_t struct as well
// as the OpenSplice specific macros, e.g. ROSIDL_GET_TYPE_SUPPORT_FUNCTION.
#include <rosidl_generator_c/message_type_support.h>
// Ensure the correct version of the above header was included.
static_assert(USING_ROSIDL_TYPESUPPORT_OPENSPLICE_C, "expected OpenSplice C message type support");
// Provides the rosidl_typesupport_opensplice_c__identifier symbol declaration.
#include <rosidl_typesupport_opensplice_c/identifier.h>
#include "robot_information_msgs/msg/rosidl_generator_c__visibility_control.h"
// Provides the definition of the message_type_support_callbacks_t struct.
#include <rosidl_typesupport_opensplice_cpp/message_type_support.h>

#include "robot_information_msgs/msg/robot_status__struct.h"
#include "robot_information_msgs/msg/robot_status__functions.h"
#include "robot_information_msgs/msg/dds_opensplice/ccpp_RobotStatus_.h"

// includes and forward declarations of message dependencies and their conversion functions
#if defined(__cplusplus)
extern "C"
{
#endif

// Forward declare the get type support function for this type.
ROSIDL_GENERATOR_C_EXPORT_robot_information_msgs
const rosidl_message_type_support_t *
ROSIDL_GET_TYPE_SUPPORT_FUNCTION(robot_information_msgs, msg, RobotStatus)();


using __dds_msg_type = robot_information_msgs::msg::dds_::RobotStatus_;
using __ros_msg_type = robot_information_msgs__msg__RobotStatus;

static const char *
register_type(void * untyped_participant, const char * type_name)
{
  if (!untyped_participant) {
    return "untyped participant handle is null";
  }
  if (!type_name) {
    return "type name handle is null";
  }
  using DDS::DomainParticipant;
  DomainParticipant * participant = static_cast<DomainParticipant *>(untyped_participant);

  robot_information_msgs::msg::dds_::RobotStatus_TypeSupport type_support;
  DDS::ReturnCode_t status = type_support.register_type(participant, type_name);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "robot_information_msgs::msg::dds_::RobotStatus_TypeSupport.register_type: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "robot_information_msgs::msg::dds_::RobotStatus_TypeSupport.register_type: "
             "bad domain participant or type name parameter";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "robot_information_msgs::msg::dds_::RobotStatus_TypeSupport.register_type: "
             "out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "robot_information_msgs::msg::dds_::RobotStatus_TypeSupport.register_type: "
             "already registered with a different TypeSupport class";
    case DDS::RETCODE_OK:
      return nullptr;
    default:
      return "robot_information_msgs::msg::dds_::RobotStatus_TypeSupport.register_type: unknown return code";
  }
}

static const char *
convert_ros_to_dds(const void * untyped_ros_message, void * untyped_dds_message)
{
  if (!untyped_ros_message) {
    return "ros message handle is null";
  }
  if (!untyped_dds_message) {
    return "dds message handle is null";
  }
  const __ros_msg_type * ros_message = static_cast<const __ros_msg_type *>(untyped_ros_message);
  __dds_msg_type * dds_message = static_cast<__dds_msg_type *>(untyped_dds_message);
  // Field name: battery
  {
    dds_message->battery_ = ros_message->battery;
  }

  // Field name: turtle_factor
  {
    dds_message->turtle_factor_ = ros_message->turtle_factor;
  }

  // Field name: battery_charging
  {
    dds_message->battery_charging_ = ros_message->battery_charging;
  }

  // Field name: drive_reversed
  {
    dds_message->drive_reversed_ = ros_message->drive_reversed;
  }

  // Field name: emergency_active
  {
    dds_message->emergency_active_ = ros_message->emergency_active;
  }

  // Field name: brake_active
  {
    dds_message->brake_active_ = ros_message->brake_active;
  }

  return 0;
}

static const char *
publish(void * dds_data_writer, const void * ros_message)
{
  if (!dds_data_writer) {
    return "data writer handle is null";
  }
  if (!ros_message) {
    return "ros message handle is null";
  }

  DDS::DataWriter * topic_writer = static_cast<DDS::DataWriter *>(dds_data_writer);

  __dds_msg_type dds_message;
  const char * err_msg = convert_ros_to_dds(ros_message, &dds_message);
  if (err_msg != 0) {
    return err_msg;
  }

  robot_information_msgs::msg::dds_::RobotStatus_DataWriter * data_writer =
    robot_information_msgs::msg::dds_::RobotStatus_DataWriter::_narrow(topic_writer);
  DDS::ReturnCode_t status = data_writer->write(dds_message, DDS::HANDLE_NIL);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataWriter.write: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataWriter.write: "
             "bad handle or instance_data parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataWriter.write: "
             "this robot_information_msgs::msg::dds_::RobotStatus_DataWriter has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataWriter.write: "
             "out of resources";
    case DDS::RETCODE_NOT_ENABLED:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataWriter.write: "
             "this robot_information_msgs::msg::dds_::RobotStatus_DataWriter is not enabled";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataWriter.write: "
             "the handle has not been registered with this robot_information_msgs::msg::dds_::RobotStatus_DataWriter";
    case DDS::RETCODE_TIMEOUT:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataWriter.write: "
             "writing resulted in blocking and then exceeded the timeout set by the "
             "max_blocking_time of the ReliabilityQosPolicy";
    case DDS::RETCODE_OK:
      return nullptr;
    default:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataWriter.write: unknown return code";
  }
}

static const char *
convert_dds_to_ros(const void * untyped_dds_message, void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    return "ros message handle is null";
  }
  if (!untyped_dds_message) {
    return "dds message handle is null";
  }
  const __dds_msg_type * dds_message = static_cast<const __dds_msg_type *>(untyped_dds_message);
  __ros_msg_type * ros_message = static_cast<__ros_msg_type *>(untyped_ros_message);
  // Field name: battery
  {
    ros_message->battery = dds_message->battery_;
  }

  // Field name: turtle_factor
  {
    ros_message->turtle_factor = dds_message->turtle_factor_;
  }

  // Field name: battery_charging
  {
    ros_message->battery_charging = dds_message->battery_charging_ == TRUE;
  }

  // Field name: drive_reversed
  {
    ros_message->drive_reversed = dds_message->drive_reversed_ == TRUE;
  }

  // Field name: emergency_active
  {
    ros_message->emergency_active = dds_message->emergency_active_ == TRUE;
  }

  // Field name: brake_active
  {
    ros_message->brake_active = dds_message->brake_active_ == TRUE;
  }

  return 0;
}

static const char *
take(
  void * dds_data_reader,
  bool ignore_local_publications,
  void * untyped_ros_message,
  bool * taken,
  void * sending_publication_handle)
{
  if (untyped_ros_message == 0) {
    return "invalid ros message pointer";
  };

  DDS::DataReader * topic_reader = static_cast<DDS::DataReader*>(dds_data_reader);

  robot_information_msgs::msg::dds_::RobotStatus_DataReader * data_reader =
    robot_information_msgs::msg::dds_::RobotStatus_DataReader::_narrow(topic_reader);

  robot_information_msgs::msg::dds_::RobotStatus_Seq dds_messages;
  DDS::SampleInfoSeq sample_infos;
  DDS::ReturnCode_t status = data_reader->take(
    dds_messages,
    sample_infos,
    1,
    DDS::ANY_SAMPLE_STATE,
    DDS::ANY_VIEW_STATE,
    DDS::ANY_INSTANCE_STATE);

  const char * errs = nullptr;
  bool ignore_sample = false;

  switch (status) {
    case DDS::RETCODE_ERROR:
      errs = "robot_information_msgs::msg::dds_::RobotStatus_DataReader.take: "
             "an internal error has occurred";
      goto finally;
    case DDS::RETCODE_ALREADY_DELETED:
      errs = "robot_information_msgs::msg::dds_::RobotStatus_DataReader.take: "
             "this robot_information_msgs::msg::dds_::RobotStatus_DataReader has already been deleted";
      goto finally;
    case DDS::RETCODE_OUT_OF_RESOURCES:
      errs = "robot_information_msgs::msg::dds_::RobotStatus_DataReader.take: "
             "out of resources";
      goto finally;
    case DDS::RETCODE_NOT_ENABLED:
      errs = "robot_information_msgs::msg::dds_::RobotStatus_DataReader.take: "
             "this robot_information_msgs::msg::dds_::RobotStatus_DataReader is not enabled";
      goto finally;
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      errs = "robot_information_msgs::msg::dds_::RobotStatus_DataReader.take: "
             "a precondition is not met, one of: "
             "max_samples > maximum and max_samples != LENGTH_UNLIMITED, or "
             "the two sequences do not have matching parameters (length, maximum, release), or "
             "maximum > 0 and release is false.";
      goto finally;
    case DDS::RETCODE_NO_DATA:
      *taken = false;
      errs = nullptr;
      goto finally;
    case DDS::RETCODE_OK:
      break;
    default:
      errs = "robot_information_msgs::msg::dds_::RobotStatus_DataReader.take: unknown return code";
      goto finally;
  }

  {
    DDS::SampleInfo & sample_info = sample_infos[0];
    if (!sample_info.valid_data) {
      // skip sample without data
      ignore_sample = true;
    } else {
      DDS::InstanceHandle_t sender_handle = sample_info.publication_handle;
      auto sender_gid = u_instanceHandleToGID(sender_handle);
      if (ignore_local_publications) {
        // compare the system id from the sender and this receiver
        // if they are equal the sample has been sent from this process and should be ignored
        DDS::InstanceHandle_t receiver_handle = topic_reader->get_instance_handle();
        auto receiver_gid = u_instanceHandleToGID(receiver_handle);
        ignore_sample = sender_gid.systemId == receiver_gid.systemId;
      }
      // This is nullptr when being used with plain rmw_take, so check first.
      if (sending_publication_handle) {
        *static_cast<DDS::InstanceHandle_t *>(sending_publication_handle) = sender_handle;
      }
    }
  }

  if (!ignore_sample) {
    errs = convert_dds_to_ros(&dds_messages[0], untyped_ros_message);
    if (errs != 0) {
      goto finally;
    }
    *taken = true;
  } else {
    *taken = false;
  }

finally:
  // Ensure the loan is returned.
  status = data_reader->return_loan(dds_messages, sample_infos);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataReader.return_loan: "
             "an internal error has occurred";
    case DDS::RETCODE_ALREADY_DELETED:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataReader.return_loan: "
             "this robot_information_msgs::msg::dds_::RobotStatus_DataReader has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataReader.return_loan: "
             "out of resources";
    case DDS::RETCODE_NOT_ENABLED:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataReader.return_loan: "
             "this robot_information_msgs::msg::dds_::RobotStatus_DataReader is not enabled";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataReader.return_loan: "
             "a precondition is not met, one of: "
             "the data_values and info_seq do not belong to a single related pair, or "
             "the data_values and info_seq were not obtained from this "
             "robot_information_msgs::msg::dds_::RobotStatus_DataReader";
    case DDS::RETCODE_OK:
      return nullptr;
    default:
      return "robot_information_msgs::msg::dds_::RobotStatus_DataReader.return_loan failed with "
             "unknown return code";
  }

  return errs;
}

static message_type_support_callbacks_t __callbacks = {
  "robot_information_msgs",  // package_name
  "RobotStatus",  // message_name
  register_type,  // register_type
  publish,  // publish
  take,  // take
  convert_ros_to_dds,  // convert_ros_to_dds
  convert_dds_to_ros,  // convert_dds_to_ros
};

static rosidl_message_type_support_t __type_support = {
  // Cannot be set since it is not a constexpr, it is set in the get type support function below.
  0,  // typesupport_identifier
  &__callbacks,  // data
};

ROSIDL_GENERATOR_C_EXPORT_robot_information_msgs
const rosidl_message_type_support_t *
ROSIDL_GET_TYPE_SUPPORT_FUNCTION(robot_information_msgs, msg, RobotStatus)()
{
  if (!__type_support.typesupport_identifier) {
    __type_support.typesupport_identifier = rosidl_typesupport_opensplice_c__identifier;
  }
  return &__type_support;
}

#if defined(__cplusplus)
}
#endif
