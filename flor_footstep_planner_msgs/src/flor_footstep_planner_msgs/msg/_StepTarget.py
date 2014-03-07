"""autogenerated by genpy from flor_footstep_planner_msgs/StepTarget.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class StepTarget(genpy.Message):
  _md5sum = "d8bcddd34acd7018686c496a5ef2ea49"
  _type = "flor_footstep_planner_msgs/StepTarget"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# Target for a single stepping motion of a humanoid's leg
std_msgs/Header header
int32 step_index                            # Step index
uint8 foot_index                            # which leg to use (LEFT_FOOT/RIGHT_FOOT)
#flor_atlas_msgs/AtlasBehaviorFootData foot  # Foothold to step to
float32 step_duration                       # duration of the single support phase
float32 sway_duration                       # duration of the double support phase (optional)
float32 swing_height                        # step apex above the lift height while swinging the leg
float32 lift_height                         # height to lift vertically before moving the foot (optional)
int32 toe_off                               # specify whether toe-off is allowed or not during swaying
float32 knee_nominal                        # Nominal knee angle during the step. (optional)
float32 max_body_accel                      # Maximum body acceleration to determine minimum sway duration (optional)
float32 max_foot_vel                        # Maximum foot velocity to determine minimum step duration (optional)
float32 sway_end_dist                       # Distance short of the foot to aim for at the end of sway (in meters) (optional)
float32 step_end_dist                       # Distance to lean into the step before the foot comes down (in meters) (optional)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','step_index','foot_index','step_duration','sway_duration','swing_height','lift_height','toe_off','knee_nominal','max_body_accel','max_foot_vel','sway_end_dist','step_end_dist']
  _slot_types = ['std_msgs/Header','int32','uint8','float32','float32','float32','float32','int32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,step_index,foot_index,step_duration,sway_duration,swing_height,lift_height,toe_off,knee_nominal,max_body_accel,max_foot_vel,sway_end_dist,step_end_dist

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(StepTarget, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.step_index is None:
        self.step_index = 0
      if self.foot_index is None:
        self.foot_index = 0
      if self.step_duration is None:
        self.step_duration = 0.
      if self.sway_duration is None:
        self.sway_duration = 0.
      if self.swing_height is None:
        self.swing_height = 0.
      if self.lift_height is None:
        self.lift_height = 0.
      if self.toe_off is None:
        self.toe_off = 0
      if self.knee_nominal is None:
        self.knee_nominal = 0.
      if self.max_body_accel is None:
        self.max_body_accel = 0.
      if self.max_foot_vel is None:
        self.max_foot_vel = 0.
      if self.sway_end_dist is None:
        self.sway_end_dist = 0.
      if self.step_end_dist is None:
        self.step_end_dist = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.step_index = 0
      self.foot_index = 0
      self.step_duration = 0.
      self.sway_duration = 0.
      self.swing_height = 0.
      self.lift_height = 0.
      self.toe_off = 0
      self.knee_nominal = 0.
      self.max_body_accel = 0.
      self.max_foot_vel = 0.
      self.sway_end_dist = 0.
      self.step_end_dist = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_iB4fi5f.pack(_x.step_index, _x.foot_index, _x.step_duration, _x.sway_duration, _x.swing_height, _x.lift_height, _x.toe_off, _x.knee_nominal, _x.max_body_accel, _x.max_foot_vel, _x.sway_end_dist, _x.step_end_dist))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 45
      (_x.step_index, _x.foot_index, _x.step_duration, _x.sway_duration, _x.swing_height, _x.lift_height, _x.toe_off, _x.knee_nominal, _x.max_body_accel, _x.max_foot_vel, _x.sway_end_dist, _x.step_end_dist,) = _struct_iB4fi5f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_iB4fi5f.pack(_x.step_index, _x.foot_index, _x.step_duration, _x.sway_duration, _x.swing_height, _x.lift_height, _x.toe_off, _x.knee_nominal, _x.max_body_accel, _x.max_foot_vel, _x.sway_end_dist, _x.step_end_dist))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 45
      (_x.step_index, _x.foot_index, _x.step_duration, _x.sway_duration, _x.swing_height, _x.lift_height, _x.toe_off, _x.knee_nominal, _x.max_body_accel, _x.max_foot_vel, _x.sway_end_dist, _x.step_end_dist,) = _struct_iB4fi5f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_iB4fi5f = struct.Struct("<iB4fi5f")
