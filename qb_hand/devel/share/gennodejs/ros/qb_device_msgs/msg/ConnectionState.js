// Auto-generated. Do not edit!

// (in-package qb_device_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DeviceConnectionInfo = require('./DeviceConnectionInfo.js');

//-----------------------------------------------------------

class ConnectionState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.devices = null;
    }
    else {
      if (initObj.hasOwnProperty('devices')) {
        this.devices = initObj.devices
      }
      else {
        this.devices = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConnectionState
    // Serialize message field [devices]
    // Serialize the length for message field [devices]
    bufferOffset = _serializer.uint32(obj.devices.length, buffer, bufferOffset);
    obj.devices.forEach((val) => {
      bufferOffset = DeviceConnectionInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConnectionState
    let len;
    let data = new ConnectionState(null);
    // Deserialize message field [devices]
    // Deserialize array length for message field [devices]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.devices = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.devices[i] = DeviceConnectionInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.devices.forEach((val) => {
      length += DeviceConnectionInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qb_device_msgs/ConnectionState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3fb8aec8a16b727f8528545cb7e04895';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Device-independent connection state message
    
    qb_device_msgs/DeviceConnectionInfo[] devices
    ================================================================================
    MSG: qb_device_msgs/DeviceConnectionInfo
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
    const resolved = new ConnectionState(null);
    if (msg.devices !== undefined) {
      resolved.devices = new Array(msg.devices.length);
      for (let i = 0; i < resolved.devices.length; ++i) {
        resolved.devices[i] = DeviceConnectionInfo.Resolve(msg.devices[i]);
      }
    }
    else {
      resolved.devices = []
    }

    return resolved;
    }
};

module.exports = ConnectionState;
