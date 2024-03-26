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

class SetCommandsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.max_repeats = null;
      this.set_commands = null;
      this.set_commands_async = null;
      this.commands = null;
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
      if (initObj.hasOwnProperty('set_commands')) {
        this.set_commands = initObj.set_commands
      }
      else {
        this.set_commands = false;
      }
      if (initObj.hasOwnProperty('set_commands_async')) {
        this.set_commands_async = initObj.set_commands_async
      }
      else {
        this.set_commands_async = false;
      }
      if (initObj.hasOwnProperty('commands')) {
        this.commands = initObj.commands
      }
      else {
        this.commands = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCommandsRequest
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [max_repeats]
    bufferOffset = _serializer.int32(obj.max_repeats, buffer, bufferOffset);
    // Serialize message field [set_commands]
    bufferOffset = _serializer.bool(obj.set_commands, buffer, bufferOffset);
    // Serialize message field [set_commands_async]
    bufferOffset = _serializer.bool(obj.set_commands_async, buffer, bufferOffset);
    // Serialize message field [commands]
    bufferOffset = _arraySerializer.int16(obj.commands, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCommandsRequest
    let len;
    let data = new SetCommandsRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_repeats]
    data.max_repeats = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [set_commands]
    data.set_commands = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [set_commands_async]
    data.set_commands_async = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [commands]
    data.commands = _arrayDeserializer.int16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.commands.length;
    return length + 14;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qb_device_srvs/SetCommandsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ed52285e2ef154a04295dd634f128ae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request
    int32 id
    int32 max_repeats
    bool set_commands
    bool set_commands_async
    int16[] commands
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCommandsRequest(null);
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

    if (msg.set_commands !== undefined) {
      resolved.set_commands = msg.set_commands;
    }
    else {
      resolved.set_commands = false
    }

    if (msg.set_commands_async !== undefined) {
      resolved.set_commands_async = msg.set_commands_async;
    }
    else {
      resolved.set_commands_async = false
    }

    if (msg.commands !== undefined) {
      resolved.commands = msg.commands;
    }
    else {
      resolved.commands = []
    }

    return resolved;
    }
};

class SetCommandsResponse {
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
    // Serializes a message object of type SetCommandsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [failures]
    bufferOffset = _serializer.int32(obj.failures, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCommandsResponse
    let len;
    let data = new SetCommandsResponse(null);
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
    return 'qb_device_srvs/SetCommandsResponse';
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
    const resolved = new SetCommandsResponse(null);
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
  Request: SetCommandsRequest,
  Response: SetCommandsResponse,
  md5sum() { return '2a24c554c16e33a4da324c504c12f0f4'; },
  datatype() { return 'qb_device_srvs/SetCommands'; }
};
