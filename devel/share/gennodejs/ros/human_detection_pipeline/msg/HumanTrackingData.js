// Auto-generated. Do not edit!

// (in-package human_detection_pipeline.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class HumanTrackingData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.tracking_id = null;
      this.distance = null;
      this.confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('tracking_id')) {
        this.tracking_id = initObj.tracking_id
      }
      else {
        this.tracking_id = 0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HumanTrackingData
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [tracking_id]
    bufferOffset = _serializer.int32(obj.tracking_id, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HumanTrackingData
    let len;
    let data = new HumanTrackingData(null);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [tracking_id]
    data.tracking_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'human_detection_pipeline/HumanTrackingData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc13d57981a0f76b7a28dbe20a53f4bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new HumanTrackingData(null);
    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.tracking_id !== undefined) {
      resolved.tracking_id = msg.tracking_id;
    }
    else {
      resolved.tracking_id = 0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    return resolved;
    }
};

module.exports = HumanTrackingData;
