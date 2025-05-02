// Auto-generated. Do not edit!

// (in-package human_clustering_pipeline.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class OptimalPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.orientation = null;
      this.priority_score = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = 0.0;
      }
      if (initObj.hasOwnProperty('priority_score')) {
        this.priority_score = initObj.priority_score
      }
      else {
        this.priority_score = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OptimalPosition
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = _serializer.float64(obj.orientation, buffer, bufferOffset);
    // Serialize message field [priority_score]
    bufferOffset = _serializer.float64(obj.priority_score, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OptimalPosition
    let len;
    let data = new OptimalPosition(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [priority_score]
    data.priority_score = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'human_clustering_pipeline/OptimalPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8dfd8a1c68c8f6ee8f557969f62c7f98';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x
    float64 y
    float64 orientation
    float64 priority_score
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OptimalPosition(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = msg.orientation;
    }
    else {
      resolved.orientation = 0.0
    }

    if (msg.priority_score !== undefined) {
      resolved.priority_score = msg.priority_score;
    }
    else {
      resolved.priority_score = 0.0
    }

    return resolved;
    }
};

module.exports = OptimalPosition;
