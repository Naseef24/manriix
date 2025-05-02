// Auto-generated. Do not edit!

// (in-package human_detection_pipeline.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let HumanTrackingData = require('./HumanTrackingData.js');

//-----------------------------------------------------------

class HumansList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.humans = null;
    }
    else {
      if (initObj.hasOwnProperty('humans')) {
        this.humans = initObj.humans
      }
      else {
        this.humans = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HumansList
    // Serialize message field [humans]
    // Serialize the length for message field [humans]
    bufferOffset = _serializer.uint32(obj.humans.length, buffer, bufferOffset);
    obj.humans.forEach((val) => {
      bufferOffset = HumanTrackingData.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HumansList
    let len;
    let data = new HumansList(null);
    // Deserialize message field [humans]
    // Deserialize array length for message field [humans]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.humans = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.humans[i] = HumanTrackingData.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 36 * object.humans.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'human_detection_pipeline/HumansList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c7644a1a2834d6a42a244e016b847ece';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # List of human tracking data
    HumanTrackingData[] humans
    
    ================================================================================
    MSG: human_detection_pipeline/HumanTrackingData
    # Define a message for storing the position (x, y, z)
    geometry_msgs/Point position   # A standard ROS message type for a 3D point (x, y, z)
    
    # Tracking information for one human
    int32 tracking_id               # Unique identifier for the human
    float32 distance                # Distance to the human
    float32 confidence              # Confidence of the tracking data
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HumansList(null);
    if (msg.humans !== undefined) {
      resolved.humans = new Array(msg.humans.length);
      for (let i = 0; i < resolved.humans.length; ++i) {
        resolved.humans[i] = HumanTrackingData.Resolve(msg.humans[i]);
      }
    }
    else {
      resolved.humans = []
    }

    return resolved;
    }
};

module.exports = HumansList;
