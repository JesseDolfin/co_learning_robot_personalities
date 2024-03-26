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

class GetMeasurementsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.max_repeats = null;
      this.get_positions = null;
      this.get_currents = null;
      this.get_distinct_packages = null;
      this.get_commands = null;
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
      if (initObj.hasOwnProperty('get_commands')) {
        this.get_commands = initObj.get_commands
      }
      else {
        this.get_commands = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetMeasurementsRequest
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [max_repeats]
    bufferOffset = _serializer.int32(obj.max_repeats, buffer, bufferOffset);
    // Serialize message field [get_positions]
    bufferOffset = _serializer.bool(obj.get_positions, buffer, bufferOffset);
    // Serialize message field [get_currents]
    bufferOffset = _serializer.bool(obj.get_currents, buffer, bufferOffset);
    // Serialize message field [get_distinct_packages]
    bufferOffset = _serializer.bool(obj.get_distinct_packages, buffer, bufferOffset);
    // Serialize message field [get_commands]
    bufferOffset = _serializer.bool(obj.get_commands, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetMeasurementsRequest
    let len;
    let data = new GetMeasurementsRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_repeats]
    data.max_repeats = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [get_positions]
    data.get_positions = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [get_currents]
    data.get_currents = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [get_distinct_packages]
    data.get_distinct_packages = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [get_commands]
    data.get_commands = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qb_device_srvs/GetMeasurementsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '07fa6d68716a20141030e8bcbf164485';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request
    int32 id
    int32 max_repeats
    bool get_positions
    bool get_currents
    bool get_distinct_packages
    bool get_commands
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetMeasurementsRequest(null);
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

    if (msg.get_commands !== undefined) {
      resolved.get_commands = msg.get_commands;
    }
    else {
      resolved.get_commands = false
    }

    return resolved;
    }
};

class GetMeasurementsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.failures = null;
      this.positions = null;
      this.currents = null;
      this.commands = null;
      this.stamp = null;
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
      if (initObj.hasOwnProperty('positions')) {
        this.positions = initObj.positions
      }
      else {
        this.positions = [];
      }
      if (initObj.hasOwnProperty('currents')) {
        this.currents = initObj.currents
      }
      else {
        this.currents = [];
      }
      if (initObj.hasOwnProperty('commands')) {
        this.commands = initObj.commands
      }
      else {
        this.commands = [];
      }
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetMeasurementsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [failures]
    bufferOffset = _serializer.int32(obj.failures, buffer, bufferOffset);
    // Serialize message field [positions]
    bufferOffset = _arraySerializer.int16(obj.positions, buffer, bufferOffset, null);
    // Serialize message field [currents]
    bufferOffset = _arraySerializer.int16(obj.currents, buffer, bufferOffset, null);
    // Serialize message field [commands]
    bufferOffset = _arraySerializer.int16(obj.commands, buffer, bufferOffset, null);
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetMeasurementsResponse
    let len;
    let data = new GetMeasurementsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [failures]
    data.failures = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [positions]
    data.positions = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [currents]
    data.currents = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [commands]
    data.commands = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.positions.length;
    length += 2 * object.currents.length;
    length += 2 * object.commands.length;
    return length + 25;
  }

  static datatype() {
    // Returns string type for a service object
    return 'qb_device_srvs/GetMeasurementsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd03a565b7bb32cc57f36f5fb5a9f047a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # response
    bool success
    int32 failures
    int16[] positions
    int16[] currents
    int16[] commands
    time stamp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetMeasurementsResponse(null);
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

    if (msg.positions !== undefined) {
      resolved.positions = msg.positions;
    }
    else {
      resolved.positions = []
    }

    if (msg.currents !== undefined) {
      resolved.currents = msg.currents;
    }
    else {
      resolved.currents = []
    }

    if (msg.commands !== undefined) {
      resolved.commands = msg.commands;
    }
    else {
      resolved.commands = []
    }

    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = {
  Request: GetMeasurementsRequest,
  Response: GetMeasurementsResponse,
  md5sum() { return '61d005acb1e04c16b9b33b19436d5ede'; },
  datatype() { return 'qb_device_srvs/GetMeasurements'; }
};
