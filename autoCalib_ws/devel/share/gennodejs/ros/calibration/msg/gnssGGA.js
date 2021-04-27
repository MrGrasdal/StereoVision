// Auto-generated. Do not edit!

// (in-package calibration.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let gnssGGA_status = require('./gnssGGA_status.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class gnssGGA {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.sat_time = null;
      this.latitude = null;
      this.lat_direction = null;
      this.longitude = null;
      this.lon_direction = null;
      this.altitude = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('sat_time')) {
        this.sat_time = initObj.sat_time
      }
      else {
        this.sat_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('lat_direction')) {
        this.lat_direction = initObj.lat_direction
      }
      else {
        this.lat_direction = '';
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('lon_direction')) {
        this.lon_direction = initObj.lon_direction
      }
      else {
        this.lon_direction = '';
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new gnssGGA_status();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gnssGGA
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [sat_time]
    bufferOffset = _serializer.time(obj.sat_time, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [lat_direction]
    bufferOffset = _serializer.string(obj.lat_direction, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [lon_direction]
    bufferOffset = _serializer.string(obj.lon_direction, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float64(obj.altitude, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = gnssGGA_status.serialize(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gnssGGA
    let len;
    let data = new gnssGGA(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [sat_time]
    data.sat_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lat_direction]
    data.lat_direction = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lon_direction]
    data.lon_direction = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = gnssGGA_status.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.lat_direction);
    length += _getByteLength(object.lon_direction);
    length += gnssGGA_status.getMessageSize(object.status);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'calibration/gnssGGA';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76d41a768710775209ac34b58a4ce202';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Something something
    
    Header header
    
    time sat_time
    
    float64 latitude
    string lat_direction
    float64 longitude
    string lon_direction
    
    float64 altitude
    
    gnssGGA_status status
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
    MSG: calibration/gnssGGA_status
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
    const resolved = new gnssGGA(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.sat_time !== undefined) {
      resolved.sat_time = msg.sat_time;
    }
    else {
      resolved.sat_time = {secs: 0, nsecs: 0}
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.lat_direction !== undefined) {
      resolved.lat_direction = msg.lat_direction;
    }
    else {
      resolved.lat_direction = ''
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.lon_direction !== undefined) {
      resolved.lon_direction = msg.lon_direction;
    }
    else {
      resolved.lon_direction = ''
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = gnssGGA_status.Resolve(msg.status)
    }
    else {
      resolved.status = new gnssGGA_status()
    }

    return resolved;
    }
};

module.exports = gnssGGA;
