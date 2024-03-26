// Auto-generated. Do not edit!

// (in-package qb_device_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Info = require('./Info.js');
let State = require('./State.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class StateStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.device_info = null;
      this.device_data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('device_info')) {
        this.device_info = initObj.device_info
      }
      else {
        this.device_info = new Info();
      }
      if (initObj.hasOwnProperty('device_data')) {
        this.device_data = initObj.device_data
      }
      else {
        this.device_data = new State();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StateStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [device_info]
    bufferOffset = Info.serialize(obj.device_info, buffer, bufferOffset);
    // Serialize message field [device_data]
    bufferOffset = State.serialize(obj.device_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StateStamped
    let len;
    let data = new StateStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [device_info]
    data.device_info = Info.deserialize(buffer, bufferOffset);
    // Deserialize message field [device_data]
    data.device_data = State.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += Info.getMessageSize(object.device_info);
    length += State.getMessageSize(object.device_data);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qb_device_msgs/StateStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ad56bd88424f6cfda763bbf1b38cce8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # State message with stamped info valid for either qbhand or qbmove
    
    std_msgs/Header header
    
    qb_device_msgs/Info device_info
    
    qb_device_msgs/State device_data
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: qb_device_msgs/Info
    # Standard device-independent info message
    
    int32 id
    string serial_port
    int32 max_repeats
    bool get_positions
    bool get_currents
    bool get_distinct_packages
    bool set_commands
    bool set_commands_async
    int32[] position_limits
    uint8[] encoder_resolutions
    ================================================================================
    MSG: qb_device_msgs/State
    # State message valid for either qbhand or qbmove
    
    # either qbhand or qbmove:
    #  - motors: position, command in [ticks], velocity in [ticks/s], effort in [mA]
    qb_device_msgs/ResourceData[] actuators
    
    # qbhand:
    #  - closure: position, command in [0,1], velocity in [percent/s],  effort in [A].
    # qbmove:
    #  - shaft: position, command in [radians], velocity in [radians/s], effort in [A];
    #  - preset: position, command in [0,1], velocity in [percent/s], effort is not used.
    qb_device_msgs/ResourceData[] joints
    
    # Reliability of the retrieved measurements
    bool is_reliable
    int32 consecutive_failures
    ================================================================================
    MSG: qb_device_msgs/ResourceData
    # Device-independent resource data message
    
    string name
    float64 position
    float64 velocity
    float64 effort
    float64 command
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StateStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.device_info !== undefined) {
      resolved.device_info = Info.Resolve(msg.device_info)
    }
    else {
      resolved.device_info = new Info()
    }

    if (msg.device_data !== undefined) {
      resolved.device_data = State.Resolve(msg.device_data)
    }
    else {
      resolved.device_data = new State()
    }

    return resolved;
    }
};

module.exports = StateStamped;
