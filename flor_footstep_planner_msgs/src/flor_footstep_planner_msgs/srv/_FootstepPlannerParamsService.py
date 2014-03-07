"""autogenerated by genpy from flor_footstep_planner_msgs/FootstepPlannerParamsServiceRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import flor_footstep_planner_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class FootstepPlannerParamsServiceRequest(genpy.Message):
  _md5sum = "1e58a94a3c082139562bf1707fd26cb9"
  _type = "flor_footstep_planner_msgs/FootstepPlannerParamsServiceRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """flor_footstep_planner_msgs/FootstepPlannerParams params

================================================================================
MSG: flor_footstep_planner_msgs/FootstepPlannerParams
uint32 change_mask
# values for change mask
uint32 STEP_COST_ESTIMATOR   =   1
uint32 FOOTSTEP_SET          =   2
uint32 LOAD_MAP_STEP_COST    =   4
uint32 LOAD_GPR_STEP_COST    =   8
uint32 COLLISION_CHECK_TYPE  =  16
uint32 FOOT_SIZE             =  32
uint32 UPPER_BODY_SIZE       =  64
uint32 STANDARD_STEP_PARAMS  = 128
uint32 TERRAIN_MODEL         = 256

### STEP_COST_ESTIMATOR ########
# switch between EUCLIDEAN or GPR
uint8 step_cost_type
# values for step_cost_type
uint8 EUCLIDEAN_STEP_COST_ESTIMATOR = 0  # discrete footstep planning
uint8 GPR_STEP_COST_ESTIMATOR       = 1  # continous GPR-based planning
uint8 MAP_STEP_COST_ESTIMATOR       = 2  # continous map-based planning
uint8 BOUNDARY_STEP_COST_ESTIMATOR  = 3  # continous boundary-based planning
uint8 DYNAMICS_STEP_COST_ESTIMATOR  = 4  # continous boundary-based planning with dynamic restrictions

### FOOTSTEP_SET ########
# parameters for discret footstep planning mode
#flor_atlas_msgs/AtlasBehaviorStepData[] footstep_set            # set of footsteps (displacement vectors (in meter / rad))
float32[] footstep_cost                                         # cost for each footstep given in footstep set

### LOAD_GPR_STEP_COST ########
# map step cost file
std_msgs/String map_step_cost_file

### LOAD_MAP_STEP_COST ########
# destination of gpr file
std_msgs/String gpr_step_cost_file

### COLLISION_CHECK_TYPE ########
# collision check
uint8 collision_check_type

# values for collision_check_type (may composed via bitwise selection, e.g. foot and upper body => 3)
uint8 FEET_COLLISION        = 1
uint8 UPPER_BODY_COLLISION  = 2
uint8 FOOT_CONTACT_SUPPORT  = 4

### FOOT_SIZE ########
# setting foot size and/or upper body
geometry_msgs/Vector3 foot_size
geometry_msgs/Vector3 foot_origin_shift
float32 foot_seperation

### UPPER_BODY_SIZE ########
geometry_msgs/Vector3 upper_body_size
geometry_msgs/Vector3 upper_body_origin_shift

### TERRAIN_MODEL ########
bool use_terrain_model
uint32 min_sampling_steps_x
uint32 min_sampling_steps_y
uint32 max_sampling_steps_x
uint32 max_sampling_steps_y
float32  max_intrusion_z    
float32 max_ground_clearance 
float32 minimal_support    

### STANDARD_STEP_PARAMS ########
# standard params for steps
float32 step_duration
float32 sway_duration   # Ignored if it is less than 0.2
float32 swing_height    # Ignored if it is less than 0.1
float32 lift_height     # Ignored if it is less than 0.1

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
"""
  __slots__ = ['params']
  _slot_types = ['flor_footstep_planner_msgs/FootstepPlannerParams']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       params

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FootstepPlannerParamsServiceRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.params is None:
        self.params = flor_footstep_planner_msgs.msg.FootstepPlannerParams()
    else:
      self.params = flor_footstep_planner_msgs.msg.FootstepPlannerParams()

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
      buff.write(_struct_IB.pack(_x.params.change_mask, _x.params.step_cost_type))
      length = len(self.params.footstep_cost)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.params.footstep_cost))
      _x = self.params.map_step_cost_file.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.params.gpr_step_cost_file.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B6df6dB4I7f.pack(_x.params.collision_check_type, _x.params.foot_size.x, _x.params.foot_size.y, _x.params.foot_size.z, _x.params.foot_origin_shift.x, _x.params.foot_origin_shift.y, _x.params.foot_origin_shift.z, _x.params.foot_seperation, _x.params.upper_body_size.x, _x.params.upper_body_size.y, _x.params.upper_body_size.z, _x.params.upper_body_origin_shift.x, _x.params.upper_body_origin_shift.y, _x.params.upper_body_origin_shift.z, _x.params.use_terrain_model, _x.params.min_sampling_steps_x, _x.params.min_sampling_steps_y, _x.params.max_sampling_steps_x, _x.params.max_sampling_steps_y, _x.params.max_intrusion_z, _x.params.max_ground_clearance, _x.params.minimal_support, _x.params.step_duration, _x.params.sway_duration, _x.params.swing_height, _x.params.lift_height))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.params is None:
        self.params = flor_footstep_planner_msgs.msg.FootstepPlannerParams()
      end = 0
      _x = self
      start = end
      end += 5
      (_x.params.change_mask, _x.params.step_cost_type,) = _struct_IB.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.params.footstep_cost = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.params.map_step_cost_file.data = str[start:end].decode('utf-8')
      else:
        self.params.map_step_cost_file.data = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.params.gpr_step_cost_file.data = str[start:end].decode('utf-8')
      else:
        self.params.gpr_step_cost_file.data = str[start:end]
      _x = self
      start = end
      end += 146
      (_x.params.collision_check_type, _x.params.foot_size.x, _x.params.foot_size.y, _x.params.foot_size.z, _x.params.foot_origin_shift.x, _x.params.foot_origin_shift.y, _x.params.foot_origin_shift.z, _x.params.foot_seperation, _x.params.upper_body_size.x, _x.params.upper_body_size.y, _x.params.upper_body_size.z, _x.params.upper_body_origin_shift.x, _x.params.upper_body_origin_shift.y, _x.params.upper_body_origin_shift.z, _x.params.use_terrain_model, _x.params.min_sampling_steps_x, _x.params.min_sampling_steps_y, _x.params.max_sampling_steps_x, _x.params.max_sampling_steps_y, _x.params.max_intrusion_z, _x.params.max_ground_clearance, _x.params.minimal_support, _x.params.step_duration, _x.params.sway_duration, _x.params.swing_height, _x.params.lift_height,) = _struct_B6df6dB4I7f.unpack(str[start:end])
      self.params.use_terrain_model = bool(self.params.use_terrain_model)
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
      buff.write(_struct_IB.pack(_x.params.change_mask, _x.params.step_cost_type))
      length = len(self.params.footstep_cost)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.params.footstep_cost.tostring())
      _x = self.params.map_step_cost_file.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.params.gpr_step_cost_file.data
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B6df6dB4I7f.pack(_x.params.collision_check_type, _x.params.foot_size.x, _x.params.foot_size.y, _x.params.foot_size.z, _x.params.foot_origin_shift.x, _x.params.foot_origin_shift.y, _x.params.foot_origin_shift.z, _x.params.foot_seperation, _x.params.upper_body_size.x, _x.params.upper_body_size.y, _x.params.upper_body_size.z, _x.params.upper_body_origin_shift.x, _x.params.upper_body_origin_shift.y, _x.params.upper_body_origin_shift.z, _x.params.use_terrain_model, _x.params.min_sampling_steps_x, _x.params.min_sampling_steps_y, _x.params.max_sampling_steps_x, _x.params.max_sampling_steps_y, _x.params.max_intrusion_z, _x.params.max_ground_clearance, _x.params.minimal_support, _x.params.step_duration, _x.params.sway_duration, _x.params.swing_height, _x.params.lift_height))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.params is None:
        self.params = flor_footstep_planner_msgs.msg.FootstepPlannerParams()
      end = 0
      _x = self
      start = end
      end += 5
      (_x.params.change_mask, _x.params.step_cost_type,) = _struct_IB.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.params.footstep_cost = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.params.map_step_cost_file.data = str[start:end].decode('utf-8')
      else:
        self.params.map_step_cost_file.data = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.params.gpr_step_cost_file.data = str[start:end].decode('utf-8')
      else:
        self.params.gpr_step_cost_file.data = str[start:end]
      _x = self
      start = end
      end += 146
      (_x.params.collision_check_type, _x.params.foot_size.x, _x.params.foot_size.y, _x.params.foot_size.z, _x.params.foot_origin_shift.x, _x.params.foot_origin_shift.y, _x.params.foot_origin_shift.z, _x.params.foot_seperation, _x.params.upper_body_size.x, _x.params.upper_body_size.y, _x.params.upper_body_size.z, _x.params.upper_body_origin_shift.x, _x.params.upper_body_origin_shift.y, _x.params.upper_body_origin_shift.z, _x.params.use_terrain_model, _x.params.min_sampling_steps_x, _x.params.min_sampling_steps_y, _x.params.max_sampling_steps_x, _x.params.max_sampling_steps_y, _x.params.max_intrusion_z, _x.params.max_ground_clearance, _x.params.minimal_support, _x.params.step_duration, _x.params.sway_duration, _x.params.swing_height, _x.params.lift_height,) = _struct_B6df6dB4I7f.unpack(str[start:end])
      self.params.use_terrain_model = bool(self.params.use_terrain_model)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_IB = struct.Struct("<IB")
_struct_B6df6dB4I7f = struct.Struct("<B6df6dB4I7f")
"""autogenerated by genpy from flor_footstep_planner_msgs/FootstepPlannerParamsServiceResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class FootstepPlannerParamsServiceResponse(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "flor_footstep_planner_msgs/FootstepPlannerParamsServiceResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """

"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FootstepPlannerParamsServiceResponse, self).__init__(*args, **kwds)

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
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
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
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
class FootstepPlannerParamsService(object):
  _type          = 'flor_footstep_planner_msgs/FootstepPlannerParamsService'
  _md5sum = '1e58a94a3c082139562bf1707fd26cb9'
  _request_class  = FootstepPlannerParamsServiceRequest
  _response_class = FootstepPlannerParamsServiceResponse
