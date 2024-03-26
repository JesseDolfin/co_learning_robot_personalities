// Auto-generated. Do not edit!

// (in-package qb_device_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ResourceData = require('./ResourceData.js');

//-----------------------------------------------------------

class State {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.actuators = null;
      this.joints = null;
      this.is_reliable = null;
      this.consecutive_failures = null;
    }
    else {
      if (initObj.hasOwnProperty('actuators')) {
        this.actuators = initObj.actuators
      }
      else {
        this.actuators = [];
      }
      if (initObj.hasOwnProperty('joints')) {
        this.joints = initObj.joints
      }
      else {
        this.joints = [];
      }
      if (initObj.hasOwnProperty('is_reliable')) {
        this.is_reliable = initObj.is_reliable
      }
      else {
        this.is_reliable = false;
      }
      if (initObj.hasOwnProperty('consecutive_failures')) {
        this.consecutive_failures = initObj.consecutive_failures
      }
      else {
        this.consecutive_failures = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type State
    // Serialize message field [actuators]
    // Serialize the length for message field [actuators]
    bufferOffset = _serializer.uint32(obj.actuators.length, buffer, bufferOffset);
    obj.actuators.forEach((val) => {
      bufferOffset = ResourceData.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [joints]
    // Serialize the length for message field [joints]
    bufferOffset = _serializer.uint32(obj.joints.length, buffer, bufferOffset);
    obj.joints.forEach((val) => {
      bufferOffset = ResourceData.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [is_reliable]
    bufferOffset = _serializer.bool(obj.is_reliable, buffer, bufferOffset);
    // Serialize message field [consecutive_failures]
    bufferOffset = _serializer.int32(obj.consecutive_failures, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type State
    let len;
    let data = new State(null);
    // Deserialize message field [actuators]
    // Deserialize array length for message field [actuators]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.actuators = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.actuators[i] = ResourceData.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [joints]
    // Deserialize array length for message field [joints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.joints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.joints[i] = ResourceData.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [is_reliable]
    data.is_reliable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [consecutive_failures]
    data.consecutive_failures = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.actuators.forEach((val) => {
      length += ResourceData.getMessageSize(val);
    });
    object.joints.forEach((val) => {
      length += ResourceData.getMessageSize(val);
    });
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qb_device_msgs/State';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '035992012f0af1c782c17a0f8f6e544c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new State(null);
    if (msg.actuators !== undefined) {
      resolved.actuators = new Array(msg.actuators.length);
      for (let i = 0; i < resolved.actuators.length; ++i) {
        resolved.actuators[i] = ResourceData.Resolve(msg.actuators[i]);
      }
    }
    else {
      resolved.actuators = []
    }

    if (msg.joints !== undefined) {
      resolved.joints = new Array(msg.joints.length);
      for (let i = 0; i < resolved.joints.length; ++i) {
        resolved.joints[i] = ResourceData.Resolve(msg.joints[i]);
      }
    }
    else {
      resolved.joints = []
    }

    if (msg.is_reliable !== undefined) {
      resolved.is_reliable = msg.is_reliable;
    }
    else {
      resolved.is_reliable = false
    }

    if (msg.consecutive_failures !== undefined) {
      resolved.consecutive_failures = msg.consecutive_failures;
    }
    else {
      resolved.consecutive_failures = 0
    }

    return resolved;
    }
};

module.exports = State;
