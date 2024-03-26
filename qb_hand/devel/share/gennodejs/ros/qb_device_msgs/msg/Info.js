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

class Info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.serial_port = null;
      this.max_repeats = null;
      this.get_positions = null;
      this.get_currents = null;
      this.get_distinct_packages = null;
      this.set_commands = null;
      this.set_commands_async = null;
      this.position_limits = null;
      this.encoder_resolutions = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('serial_port')) {
        this.serial_port = initObj.serial_port
      }
      else {
        this.serial_port = '';
      }
      if (initObj.hasOwnProperty('max_repeats')) {
        this.max_repeats = initObj.max_repeats
      }
      else {
        this.max_repeats = 0;
      }
      if (initObj.hasOwnProperty('get_positions')) {
        this.get_positions = initObj.get_positions
      }
      else {
        this.get_positions = false;
      }
      if (initObj.hasOwnProperty('get_currents')) {
        this.get_currents = initObj.get_currents
      }
      else {
        this.get_currents = false;
      }
      if (initObj.hasOwnProperty('get_distinct_packages')) {
        this.get_distinct_packages = initObj.get_distinct_packages
      }
      else {
        this.get_distinct_packages = false;
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
      if (initObj.hasOwnProperty('position_limits')) {
        this.position_limits = initObj.position_limits
      }
      else {
        this.position_limits = [];
      }
      if (initObj.hasOwnProperty('encoder_resolutions')) {
        this.encoder_resolutions = initObj.encoder_resolutions
      }
      else {
        this.encoder_resolutions = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Info
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [serial_port]
    bufferOffset = _serializer.string(obj.serial_port, buffer, bufferOffset);
    // Serialize message field [max_repeats]
    bufferOffset = _serializer.int32(obj.max_repeats, buffer, bufferOffset);
    // Serialize message field [get_positions]
    bufferOffset = _serializer.bool(obj.get_positions, buffer, bufferOffset);
    // Serialize message field [get_currents]
    bufferOffset = _serializer.bool(obj.get_currents, buffer, bufferOffset);
    // Serialize message field [get_distinct_packages]
    bufferOffset = _serializer.bool(obj.get_distinct_packages, buffer, bufferOffset);
    // Serialize message field [set_commands]
    bufferOffset = _serializer.bool(obj.set_commands, buffer, bufferOffset);
    // Serialize message field [set_commands_async]
    bufferOffset = _serializer.bool(obj.set_commands_async, buffer, bufferOffset);
    // Serialize message field [position_limits]
    bufferOffset = _arraySerializer.int32(obj.position_limits, buffer, bufferOffset, null);
    // Serialize message field [encoder_resolutions]
    bufferOffset = _arraySerializer.uint8(obj.encoder_resolutions, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Info
    let len;
    let data = new Info(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [serial_port]
    data.serial_port = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [max_repeats]
    data.max_repeats = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [get_positions]
    data.get_positions = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [get_currents]
    data.get_currents = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [get_distinct_packages]
    data.get_distinct_packages = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [set_commands]
    data.set_commands = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [set_commands_async]
    data.set_commands_async = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [position_limits]
    data.position_limits = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [encoder_resolutions]
    data.encoder_resolutions = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.serial_port);
    length += 4 * object.position_limits.length;
    length += object.encoder_resolutions.length;
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qb_device_msgs/Info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ee93ec9c1a360ac561916c3deecf8486';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Info(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.serial_port !== undefined) {
      resolved.serial_port = msg.serial_port;
    }
    else {
      resolved.serial_port = ''
    }

    if (msg.max_repeats !== undefined) {
      resolved.max_repeats = msg.max_repeats;
    }
    else {
      resolved.max_repeats = 0
    }

    if (msg.get_positions !== undefined) {
      resolved.get_positions = msg.get_positions;
    }
    else {
      resolved.get_positions = false
    }

    if (msg.get_currents !== undefined) {
      resolved.get_currents = msg.get_currents;
    }
    else {
      resolved.get_currents = false
    }

    if (msg.get_distinct_packages !== undefined) {
      resolved.get_distinct_packages = msg.get_distinct_packages;
    }
    else {
      resolved.get_distinct_packages = false
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

    if (msg.position_limits !== undefined) {
      resolved.position_limits = msg.position_limits;
    }
    else {
      resolved.position_limits = []
    }

    if (msg.encoder_resolutions !== undefined) {
      resolved.encoder_resolutions = msg.encoder_resolutions;
    }
    else {
      resolved.encoder_resolutions = []
    }

    return resolved;
    }
};

module.exports = Info;
