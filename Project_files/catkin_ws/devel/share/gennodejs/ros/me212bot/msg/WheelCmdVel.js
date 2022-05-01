// Auto-generated. Do not edit!

// (in-package me212bot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class WheelCmdVel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.desiredWV_R = null;
      this.desiredWV_L = null;
    }
    else {
      if (initObj.hasOwnProperty('desiredWV_R')) {
        this.desiredWV_R = initObj.desiredWV_R
      }
      else {
        this.desiredWV_R = 0.0;
      }
      if (initObj.hasOwnProperty('desiredWV_L')) {
        this.desiredWV_L = initObj.desiredWV_L
      }
      else {
        this.desiredWV_L = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WheelCmdVel
    // Serialize message field [desiredWV_R]
    bufferOffset = _serializer.float32(obj.desiredWV_R, buffer, bufferOffset);
    // Serialize message field [desiredWV_L]
    bufferOffset = _serializer.float32(obj.desiredWV_L, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WheelCmdVel
    let len;
    let data = new WheelCmdVel(null);
    // Deserialize message field [desiredWV_R]
    data.desiredWV_R = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [desiredWV_L]
    data.desiredWV_L = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'me212bot/WheelCmdVel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '428fbbfd1f38717ca7baa73045b4efaa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 desiredWV_R
    float32 desiredWV_L
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WheelCmdVel(null);
    if (msg.desiredWV_R !== undefined) {
      resolved.desiredWV_R = msg.desiredWV_R;
    }
    else {
      resolved.desiredWV_R = 0.0
    }

    if (msg.desiredWV_L !== undefined) {
      resolved.desiredWV_L = msg.desiredWV_L;
    }
    else {
      resolved.desiredWV_L = 0.0
    }

    return resolved;
    }
};

module.exports = WheelCmdVel;
