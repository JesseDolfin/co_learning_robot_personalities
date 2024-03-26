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


//-----------------------------------------------------------

class SetControlModeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.max_repeats = null;
      this.control = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('max_repeats')) {
        this.max_repeats = initObj.max_repeats
      }
      else {
        this.max_repeats = 0;
      }
      if (initObj.hasOwnProperty('control')) {
        this.control = initObj.control
      }
      else {
        this.control = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetControlModeRequest
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [max_repeats]
    bufferOffset = _serializer.int32(obj.max_repeats, buffer, bufferOffset);
    // Serialize message field [control]
    bufferOffset = _serializer.string(obj.control, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControlModeRequest
    let len;
    let data = new SetControlModeRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_repeats]
    data.max_repeats = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [control]
    data.control = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.control);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qb_device_srvs/SetControlModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5ca3acba2aaf7cc4f3f0f974eabc3f70';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request
    int32 id
    int32 max_repeats
    string control
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetControlModeRequest(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.max_repeats !== undefined) {
      resolved.max_repeats = msg.max_repeats;
    }
    else {
      resolved.max_repeats = 0
    }

    if (msg.control !== undefined) {
      resolved.control = msg.control;
    }
    else {
      resolved.control = ''
    }

    return resolved;
    }
};

class SetControlModeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.failures = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetControlModeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [failures]
    bufferOffset = _serializer.int32(obj.failures, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControlModeResponse
    let len;
    let data = new SetControlModeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [failures]
    data.failures = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qb_device_srvs/SetControlModeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '45434ccc588901681d58dcbb05939d8a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # response
    bool success
    int32 failures
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetControlModeResponse(null);
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

    return resolved;
    }
};

module.exports = {
  Request: SetControlModeRequest,
  Response: SetControlModeResponse,
  md5sum() { return 'f6dd4c28fb851ba56deb138366053217'; },
  datatype() { return 'qb_device_srvs/SetControlMode'; }
};
