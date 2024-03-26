# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from qb_device_srvs/SetPIDRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetPIDRequest(genpy.Message):
  _md5sum = "dc65c5e2e1370541c789de54fc9b025c"
  _type = "qb_device_srvs/SetPIDRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# request
int32 id
int32 max_repeats
float32 p
float32 i
float32 d
"""
  __slots__ = ['id','max_repeats','p','i','d']
  _slot_types = ['int32','int32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id,max_repeats,p,i,d

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetPIDRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = 0
      if self.max_repeats is None:
        self.max_repeats = 0
      if self.p is None:
        self.p = 0.
      if self.i is None:
        self.i = 0.
      if self.d is None:
        self.d = 0.
    else:
      self.id = 0
      self.max_repeats = 0
      self.p = 0.
      self.i = 0.
      self.d = 0.

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
      buff.write(_get_struct_2i3f().pack(_x.id, _x.max_repeats, _x.p, _x.i, _x.d))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 20
      (_x.id, _x.max_repeats, _x.p, _x.i, _x.d,) = _get_struct_2i3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2i3f().pack(_x.id, _x.max_repeats, _x.p, _x.i, _x.d))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 20
      (_x.id, _x.max_repeats, _x.p, _x.i, _x.d,) = _get_struct_2i3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2i3f = None
def _get_struct_2i3f():
    global _struct_2i3f
    if _struct_2i3f is None:
        _struct_2i3f = struct.Struct("<2i3f")
    return _struct_2i3f
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from qb_device_srvs/SetPIDResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetPIDResponse(genpy.Message):
  _md5sum = "45434ccc588901681d58dcbb05939d8a"
  _type = "qb_device_srvs/SetPIDResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# response
bool success
int32 failures
"""
  __slots__ = ['success','failures']
  _slot_types = ['bool','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       success,failures

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetPIDResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.success is None:
        self.success = False
      if self.failures is None:
        self.failures = 0
    else:
      self.success = False
      self.failures = 0

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
      buff.write(_get_struct_Bi().pack(_x.success, _x.failures))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.success, _x.failures,) = _get_struct_Bi().unpack(str[start:end])
      self.success = bool(self.success)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_Bi().pack(_x.success, _x.failures))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 5
      (_x.success, _x.failures,) = _get_struct_Bi().unpack(str[start:end])
      self.success = bool(self.success)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_Bi = None
def _get_struct_Bi():
    global _struct_Bi
    if _struct_Bi is None:
        _struct_Bi = struct.Struct("<Bi")
    return _struct_Bi
class SetPID(object):
  _type          = 'qb_device_srvs/SetPID'
  _md5sum = '924f2bcd17c4f9d95c42467a96352ac1'
  _request_class  = SetPIDRequest
  _response_class = SetPIDResponse
