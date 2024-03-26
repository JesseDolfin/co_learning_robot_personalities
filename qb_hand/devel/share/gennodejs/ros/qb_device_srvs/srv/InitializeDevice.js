// Auto-generated. Do not edit!

// (in-package qb_device_srvs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let qb_device_msgs = _finder('qb_device_msgs');

//-----------------------------------------------------------

class InitializeDeviceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.activate = null;
      this.rescan = null;
      this.max_repeats = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('activate')) {
        this.activate = initObj.activate
      }
      else {
        this.activate = false;
      }
      if (initObj.hasOwnProperty('rescan')) {
        this.rescan = initObj.rescan
      }
      else {
        this.rescan = false;
      }
      if (initObj.hasOwnProperty('max_repeats')) {
        this.max_repeats = initObj.max_repeats
      }
      else {
        this.max_repeats = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InitializeDeviceRequest
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [activate]
    bufferOffset = _serializer.bool(obj.activate, buffer, bufferOffset);
    // Serialize message field [rescan]
    bufferOffset = _serializer.bool(obj.rescan, buffer, bufferOffset);
    // Serialize message field [max_repeats]
    bufferOffset = _serializer.int32(obj.max_repeats, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitializeDeviceRequest
    let len;
    let data = new InitializeDeviceRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [activate]
    data.activate = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [rescan]
    data.rescan = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [max_repeats]
    data.max_repeats = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qb_device_srvs/InitializeDeviceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd69fcec3a38c9f637f3b8a74cff24b49';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request
    int32 id
    bool activate
    bool rescan
    int32 max_repeats
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InitializeDeviceRequest(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.activate !== undefined) {
      resolved.activate = msg.activate;
    }
    else {
      resolved.activate = false
    }

    if (msg.rescan !== undefined) {
      resolved.rescan = msg.rescan;
    }
    else {
      resolved.rescan = false
    }

    if (msg.max_repeats !== undefined) {
      resolved.max_repeats = msg.max_repeats;
    }
    else {
      resolved.max_repeats = 0
    }

    return resolved;
    }
};

class InitializeDeviceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.failures = null;
      this.message = null;
      this.info = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('failures')) {
        this.failures = initObj.failures
      }
      else {
        this.failures = 0;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('info')) {
        this.info = initObj.info
      }
      else {
        this.info = new qb_device_msgs.msg.Info();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InitializeDeviceResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [failures]
    bufferOffset = _serializer.int32(obj.failures, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [info]
    bufferOffset = qb_device_msgs.msg.Info.serialize(obj.info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InitializeDeviceResponse
    let len;
    let data = new InitializeDeviceResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [failures]
    data.failures = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [info]
    data.info = qb_device_msgs.msg.Info.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    length += qb_device_msgs.msg.Info.getMessageSize(object.info);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qb_device_srvs/InitializeDeviceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bd0e0da354b1afd61de86913895db9e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # response
    bool success
    int32 failures
    string message
    qb_device_msgs/Info info
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InitializeDeviceResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.failures !== undefined) {
      resolved.failures = msg.failures;
    }
    else {
      resolved.failures = 0
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.info !== undefined) {
      resolved.info = qb_device_msgs.msg.Info.Resolve(msg.info)
    }
    else {
      resolved.info = new qb_device_msgs.msg.Info()
    }

    return resolved;
    }
};

module.exports = {
  Request: InitializeDeviceRequest,
  Response: InitializeDeviceResponse,
  md5sum() { return 'c23409117bb3e9fa24111fdf884ca1f3'; },
  datatype() { return 'qb_device_srvs/InitializeDevice'; }
};
