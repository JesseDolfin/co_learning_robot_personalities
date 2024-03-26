// Auto-generated. Do not edit!

// (in-package qb_device_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DeviceConnectionInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.is_active = null;
      this.port = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('is_active')) {
        this.is_active = initObj.is_active
      }
      else {
        this.is_active = false;
      }
      if (initObj.hasOwnProperty('port')) {
        this.port = initObj.port
      }
      else {
        this.port = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeviceConnectionInfo
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [is_active]
    bufferOffset = _serializer.bool(obj.is_active, buffer, bufferOffset);
    // Serialize message field [port]
    bufferOffset = _serializer.string(obj.port, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceConnectionInfo
    let len;
    let data = new DeviceConnectionInfo(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [is_active]
    data.is_active = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [port]
    data.port = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.port);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qb_device_msgs/DeviceConnectionInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1a1c593244281b064cf77ab64e673fe3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Device-independent message that constains: 
    
    int32 id           # device id;
    bool is_active     # motor activation status;
    string port        # serial port
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DeviceConnectionInfo(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.is_active !== undefined) {
      resolved.is_active = msg.is_active;
    }
    else {
      resolved.is_active = false
    }

    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = ''
    }

    return resolved;
    }
};

module.exports = DeviceConnectionInfo;
