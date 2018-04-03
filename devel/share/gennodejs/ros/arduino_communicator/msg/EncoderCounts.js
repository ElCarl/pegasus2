// Auto-generated. Do not edit!

// (in-package arduino_communicator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EncoderCounts {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left_wheel_counts = null;
      this.right_wheel_counts = null;
      this.base_rotation_counts = null;
      this.wrist_rotation_counts = null;
      this.gripper_counts = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('left_wheel_counts')) {
        this.left_wheel_counts = initObj.left_wheel_counts
      }
      else {
        this.left_wheel_counts = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('right_wheel_counts')) {
        this.right_wheel_counts = initObj.right_wheel_counts
      }
      else {
        this.right_wheel_counts = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('base_rotation_counts')) {
        this.base_rotation_counts = initObj.base_rotation_counts
      }
      else {
        this.base_rotation_counts = 0;
      }
      if (initObj.hasOwnProperty('wrist_rotation_counts')) {
        this.wrist_rotation_counts = initObj.wrist_rotation_counts
      }
      else {
        this.wrist_rotation_counts = 0;
      }
      if (initObj.hasOwnProperty('gripper_counts')) {
        this.gripper_counts = initObj.gripper_counts
      }
      else {
        this.gripper_counts = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EncoderCounts
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [left_wheel_counts] has the right length
    if (obj.left_wheel_counts.length !== 3) {
      throw new Error('Unable to serialize array field left_wheel_counts - length must be 3')
    }
    // Serialize message field [left_wheel_counts]
    bufferOffset = _arraySerializer.int32(obj.left_wheel_counts, buffer, bufferOffset, 3);
    // Check that the constant length array field [right_wheel_counts] has the right length
    if (obj.right_wheel_counts.length !== 3) {
      throw new Error('Unable to serialize array field right_wheel_counts - length must be 3')
    }
    // Serialize message field [right_wheel_counts]
    bufferOffset = _arraySerializer.int32(obj.right_wheel_counts, buffer, bufferOffset, 3);
    // Serialize message field [base_rotation_counts]
    bufferOffset = _serializer.int32(obj.base_rotation_counts, buffer, bufferOffset);
    // Serialize message field [wrist_rotation_counts]
    bufferOffset = _serializer.int32(obj.wrist_rotation_counts, buffer, bufferOffset);
    // Serialize message field [gripper_counts]
    bufferOffset = _serializer.int32(obj.gripper_counts, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EncoderCounts
    let len;
    let data = new EncoderCounts(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_wheel_counts]
    data.left_wheel_counts = _arrayDeserializer.int32(buffer, bufferOffset, 3)
    // Deserialize message field [right_wheel_counts]
    data.right_wheel_counts = _arrayDeserializer.int32(buffer, bufferOffset, 3)
    // Deserialize message field [base_rotation_counts]
    data.base_rotation_counts = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [wrist_rotation_counts]
    data.wrist_rotation_counts = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [gripper_counts]
    data.gripper_counts = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arduino_communicator/EncoderCounts';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '890e34750b24fc194afb98e978102a4f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32[3] left_wheel_counts   # Front to back
    int32[3] right_wheel_counts  # Front to back
    int32 base_rotation_counts
    int32 wrist_rotation_counts
    int32 gripper_counts
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EncoderCounts(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.left_wheel_counts !== undefined) {
      resolved.left_wheel_counts = msg.left_wheel_counts;
    }
    else {
      resolved.left_wheel_counts = new Array(3).fill(0)
    }

    if (msg.right_wheel_counts !== undefined) {
      resolved.right_wheel_counts = msg.right_wheel_counts;
    }
    else {
      resolved.right_wheel_counts = new Array(3).fill(0)
    }

    if (msg.base_rotation_counts !== undefined) {
      resolved.base_rotation_counts = msg.base_rotation_counts;
    }
    else {
      resolved.base_rotation_counts = 0
    }

    if (msg.wrist_rotation_counts !== undefined) {
      resolved.wrist_rotation_counts = msg.wrist_rotation_counts;
    }
    else {
      resolved.wrist_rotation_counts = 0
    }

    if (msg.gripper_counts !== undefined) {
      resolved.gripper_counts = msg.gripper_counts;
    }
    else {
      resolved.gripper_counts = 0
    }

    return resolved;
    }
};

module.exports = EncoderCounts;
