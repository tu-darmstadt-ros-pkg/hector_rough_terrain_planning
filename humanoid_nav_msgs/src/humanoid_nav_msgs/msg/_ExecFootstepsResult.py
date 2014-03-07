"""autogenerated by genpy from humanoid_nav_msgs/ExecFootstepsResult.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import humanoid_nav_msgs.msg

class ExecFootstepsResult(genpy.Message):
  _md5sum = "5dfde2cb244d6c76567d3c52c40a988c"
  _type = "humanoid_nav_msgs/ExecFootstepsResult"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Define the result
humanoid_nav_msgs/StepTarget[] executed_footsteps

================================================================================
MSG: humanoid_nav_msgs/StepTarget
# Target for a single stepping motion of a humanoid's leg

geometry_msgs/Pose2D pose   # step pose as relative offset to last leg
uint8 leg                   # which leg to use (left/right, see below)

uint8 right=0               # right leg constant
uint8 left=1                # left leg constant

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
"""
  __slots__ = ['executed_footsteps']
  _slot_types = ['humanoid_nav_msgs/StepTarget[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       executed_footsteps

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ExecFootstepsResult, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.executed_footsteps is None:
        self.executed_footsteps = []
    else:
      self.executed_footsteps = []

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
      length = len(self.executed_footsteps)
      buff.write(_struct_I.pack(length))
      for val1 in self.executed_footsteps:
        _v1 = val1.pose
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.theta))
        buff.write(_struct_B.pack(val1.leg))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.executed_footsteps is None:
        self.executed_footsteps = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.executed_footsteps = []
      for i in range(0, length):
        val1 = humanoid_nav_msgs.msg.StepTarget()
        _v2 = val1.pose
        _x = _v2
        start = end
        end += 24
        (_x.x, _x.y, _x.theta,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 1
        (val1.leg,) = _struct_B.unpack(str[start:end])
        self.executed_footsteps.append(val1)
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
      length = len(self.executed_footsteps)
      buff.write(_struct_I.pack(length))
      for val1 in self.executed_footsteps:
        _v3 = val1.pose
        _x = _v3
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.theta))
        buff.write(_struct_B.pack(val1.leg))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.executed_footsteps is None:
        self.executed_footsteps = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.executed_footsteps = []
      for i in range(0, length):
        val1 = humanoid_nav_msgs.msg.StepTarget()
        _v4 = val1.pose
        _x = _v4
        start = end
        end += 24
        (_x.x, _x.y, _x.theta,) = _struct_3d.unpack(str[start:end])
        start = end
        end += 1
        (val1.leg,) = _struct_B.unpack(str[start:end])
        self.executed_footsteps.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
_struct_3d = struct.Struct("<3d")
