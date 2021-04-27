// Auto-generated. Do not edit!

// (in-package calibration.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class gnssGGA_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.numSat = null;
      this.hDOP = null;
      this.mode = null;
      this.modeStatus = null;
    }
    else {
      if (initObj.hasOwnProperty('numSat')) {
        this.numSat = initObj.numSat
      }
      else {
        this.numSat = 0;
      }
      if (initObj.hasOwnProperty('hDOP')) {
        this.hDOP = initObj.hDOP
      }
      else {
        this.hDOP = 0.0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('modeStatus')) {
        this.modeStatus = initObj.modeStatus
      }
      else {
        this.modeStatus = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gnssGGA_status
    // Serialize message field [numSat]
    bufferOffset = _serializer.int64(obj.numSat, buffer, bufferOffset);
    // Serialize message field [hDOP]
    bufferOffset = _serializer.float64(obj.hDOP, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int64(obj.mode, buffer, bufferOffset);
    // Serialize message field [modeStatus]
    bufferOffset = _serializer.string(obj.modeStatus, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gnssGGA_status
    let len;
    let data = new gnssGGA_status(null);
    // Deserialize message field [numSat]
    data.numSat = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [hDOP]
    data.hDOP = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [modeStatus]
    data.modeStatus = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.modeStatus);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'calibration/gnssGGA_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0e8745cbb7cfe1a00d99b4fdc87eeb1f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 numSat
    float64 hDOP
    int64 mode
    string modeStatus
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gnssGGA_status(null);
    if (msg.numSat !== undefined) {
      resolved.numSat = msg.numSat;
    }
    else {
      resolved.numSat = 0
    }

    if (msg.hDOP !== undefined) {
      resolved.hDOP = msg.hDOP;
    }
    else {
      resolved.hDOP = 0.0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.modeStatus !== undefined) {
      resolved.modeStatus = msg.modeStatus;
    }
    else {
      resolved.modeStatus = ''
    }

    return resolved;
    }
};

module.exports = gnssGGA_status;
