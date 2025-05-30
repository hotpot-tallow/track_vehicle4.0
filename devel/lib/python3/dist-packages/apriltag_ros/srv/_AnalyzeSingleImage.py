# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from apriltag_ros/AnalyzeSingleImageRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import sensor_msgs.msg
import std_msgs.msg

class AnalyzeSingleImageRequest(genpy.Message):
  _md5sum = "ce260db7e8fcb58cbea397e93c5438a4"
  _type = "apriltag_ros/AnalyzeSingleImageRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Service which takes in:
#
#   full_path_to_image : full path to a .jpg image
#
# and returns:
#
#                 pose : the pose of the tag in the camera frame
#  tag_detection_image : an image with the detected tag's border highlighted and payload value printed

string full_path_where_to_get_image
string full_path_where_to_save_image
sensor_msgs/CameraInfo camera_info

================================================================================
MSG: sensor_msgs/CameraInfo
# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
Header header    # Header timestamp should be acquisition time of image
                 # Header frame_id should be optical frame of camera
                 # origin of frame should be optical center of camera
                 # +x should point to the right in the image
                 # +y should point down in the image
                 # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may "recalibrate" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated. Normally
# this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficient.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] D

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  K # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  R # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] P # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi

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
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the "raw"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
"""
  __slots__ = ['full_path_where_to_get_image','full_path_where_to_save_image','camera_info']
  _slot_types = ['string','string','sensor_msgs/CameraInfo']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       full_path_where_to_get_image,full_path_where_to_save_image,camera_info

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AnalyzeSingleImageRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.full_path_where_to_get_image is None:
        self.full_path_where_to_get_image = ''
      if self.full_path_where_to_save_image is None:
        self.full_path_where_to_save_image = ''
      if self.camera_info is None:
        self.camera_info = sensor_msgs.msg.CameraInfo()
    else:
      self.full_path_where_to_get_image = ''
      self.full_path_where_to_save_image = ''
      self.camera_info = sensor_msgs.msg.CameraInfo()

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
      _x = self.full_path_where_to_get_image
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.full_path_where_to_save_image
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.camera_info.header.seq, _x.camera_info.header.stamp.secs, _x.camera_info.header.stamp.nsecs))
      _x = self.camera_info.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.camera_info.height, _x.camera_info.width))
      _x = self.camera_info.distortion_model
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.camera_info.D)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.camera_info.D))
      buff.write(_get_struct_9d().pack(*self.camera_info.K))
      buff.write(_get_struct_9d().pack(*self.camera_info.R))
      buff.write(_get_struct_12d().pack(*self.camera_info.P))
      _x = self
      buff.write(_get_struct_6IB().pack(_x.camera_info.binning_x, _x.camera_info.binning_y, _x.camera_info.roi.x_offset, _x.camera_info.roi.y_offset, _x.camera_info.roi.height, _x.camera_info.roi.width, _x.camera_info.roi.do_rectify))
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
      if self.camera_info is None:
        self.camera_info = sensor_msgs.msg.CameraInfo()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.full_path_where_to_get_image = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.full_path_where_to_get_image = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.full_path_where_to_save_image = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.full_path_where_to_save_image = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.camera_info.header.seq, _x.camera_info.header.stamp.secs, _x.camera_info.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.camera_info.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.camera_info.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.camera_info.height, _x.camera_info.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.camera_info.distortion_model = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.camera_info.distortion_model = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.camera_info.D = s.unpack(str[start:end])
      start = end
      end += 72
      self.camera_info.K = _get_struct_9d().unpack(str[start:end])
      start = end
      end += 72
      self.camera_info.R = _get_struct_9d().unpack(str[start:end])
      start = end
      end += 96
      self.camera_info.P = _get_struct_12d().unpack(str[start:end])
      _x = self
      start = end
      end += 25
      (_x.camera_info.binning_x, _x.camera_info.binning_y, _x.camera_info.roi.x_offset, _x.camera_info.roi.y_offset, _x.camera_info.roi.height, _x.camera_info.roi.width, _x.camera_info.roi.do_rectify,) = _get_struct_6IB().unpack(str[start:end])
      self.camera_info.roi.do_rectify = bool(self.camera_info.roi.do_rectify)
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
      _x = self.full_path_where_to_get_image
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.full_path_where_to_save_image
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.camera_info.header.seq, _x.camera_info.header.stamp.secs, _x.camera_info.header.stamp.nsecs))
      _x = self.camera_info.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2I().pack(_x.camera_info.height, _x.camera_info.width))
      _x = self.camera_info.distortion_model
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.camera_info.D)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.camera_info.D.tostring())
      buff.write(self.camera_info.K.tostring())
      buff.write(self.camera_info.R.tostring())
      buff.write(self.camera_info.P.tostring())
      _x = self
      buff.write(_get_struct_6IB().pack(_x.camera_info.binning_x, _x.camera_info.binning_y, _x.camera_info.roi.x_offset, _x.camera_info.roi.y_offset, _x.camera_info.roi.height, _x.camera_info.roi.width, _x.camera_info.roi.do_rectify))
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
      if self.camera_info is None:
        self.camera_info = sensor_msgs.msg.CameraInfo()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.full_path_where_to_get_image = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.full_path_where_to_get_image = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.full_path_where_to_save_image = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.full_path_where_to_save_image = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.camera_info.header.seq, _x.camera_info.header.stamp.secs, _x.camera_info.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.camera_info.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.camera_info.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.camera_info.height, _x.camera_info.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.camera_info.distortion_model = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.camera_info.distortion_model = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.camera_info.D = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 72
      self.camera_info.K = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=9)
      start = end
      end += 72
      self.camera_info.R = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=9)
      start = end
      end += 96
      self.camera_info.P = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=12)
      _x = self
      start = end
      end += 25
      (_x.camera_info.binning_x, _x.camera_info.binning_y, _x.camera_info.roi.x_offset, _x.camera_info.roi.y_offset, _x.camera_info.roi.height, _x.camera_info.roi.width, _x.camera_info.roi.do_rectify,) = _get_struct_6IB().unpack(str[start:end])
      self.camera_info.roi.do_rectify = bool(self.camera_info.roi.do_rectify)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_12d = None
def _get_struct_12d():
    global _struct_12d
    if _struct_12d is None:
        _struct_12d = struct.Struct("<12d")
    return _struct_12d
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_6IB = None
def _get_struct_6IB():
    global _struct_6IB
    if _struct_6IB is None:
        _struct_6IB = struct.Struct("<6IB")
    return _struct_6IB
_struct_9d = None
def _get_struct_9d():
    global _struct_9d
    if _struct_9d is None:
        _struct_9d = struct.Struct("<9d")
    return _struct_9d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from apriltag_ros/AnalyzeSingleImageResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import apriltag_ros.msg
import geometry_msgs.msg
import std_msgs.msg

class AnalyzeSingleImageResponse(genpy.Message):
  _md5sum = "252b618af4df2baf843a5edd035f3c2c"
  _type = "apriltag_ros/AnalyzeSingleImageResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """apriltag_ros/AprilTagDetectionArray tag_detections

================================================================================
MSG: apriltag_ros/AprilTagDetectionArray
std_msgs/Header header
AprilTagDetection[] detections

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
MSG: apriltag_ros/AprilTagDetection
# Tag ID(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
# this is a vector containing the IDs of each tag in the bundle.
int32[] id

# Tag size(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
# this is a vector containing the sizes of each tag in the bundle, in the same
# order as the IDs above.
float64[] size

# Pose in the camera frame, obtained from homography transform. If a standalone
# tag, the homography is from the four tag corners. If a tag bundle, the
# homography is from at least the four corners of one member tag and at most the
# four corners of all member tags.
geometry_msgs/PoseWithCovarianceStamped pose

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['tag_detections']
  _slot_types = ['apriltag_ros/AprilTagDetectionArray']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       tag_detections

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AnalyzeSingleImageResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.tag_detections is None:
        self.tag_detections = apriltag_ros.msg.AprilTagDetectionArray()
    else:
      self.tag_detections = apriltag_ros.msg.AprilTagDetectionArray()

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
      buff.write(_get_struct_3I().pack(_x.tag_detections.header.seq, _x.tag_detections.header.stamp.secs, _x.tag_detections.header.stamp.nsecs))
      _x = self.tag_detections.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.tag_detections.detections)
      buff.write(_struct_I.pack(length))
      for val1 in self.tag_detections.detections:
        length = len(val1.id)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(struct.Struct(pattern).pack(*val1.id))
        length = len(val1.size)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(struct.Struct(pattern).pack(*val1.size))
        _v1 = val1.pose
        _v2 = _v1.header
        _x = _v2.seq
        buff.write(_get_struct_I().pack(_x))
        _v3 = _v2.stamp
        _x = _v3
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v2.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v4 = _v1.pose
        _v5 = _v4.pose
        _v6 = _v5.position
        _x = _v6
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v7 = _v5.orientation
        _x = _v7
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_get_struct_36d().pack(*_v4.covariance))
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
      if self.tag_detections is None:
        self.tag_detections = apriltag_ros.msg.AprilTagDetectionArray()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.tag_detections.header.seq, _x.tag_detections.header.stamp.secs, _x.tag_detections.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tag_detections.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tag_detections.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.tag_detections.detections = []
      for i in range(0, length):
        val1 = apriltag_ros.msg.AprilTagDetection()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.id = s.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.size = s.unpack(str[start:end])
        _v8 = val1.pose
        _v9 = _v8.header
        start = end
        end += 4
        (_v9.seq,) = _get_struct_I().unpack(str[start:end])
        _v10 = _v9.stamp
        _x = _v10
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v9.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v9.frame_id = str[start:end]
        _v11 = _v8.pose
        _v12 = _v11.pose
        _v13 = _v12.position
        _x = _v13
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v14 = _v12.orientation
        _x = _v14
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        start = end
        end += 288
        _v11.covariance = _get_struct_36d().unpack(str[start:end])
        self.tag_detections.detections.append(val1)
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
      buff.write(_get_struct_3I().pack(_x.tag_detections.header.seq, _x.tag_detections.header.stamp.secs, _x.tag_detections.header.stamp.nsecs))
      _x = self.tag_detections.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.tag_detections.detections)
      buff.write(_struct_I.pack(length))
      for val1 in self.tag_detections.detections:
        length = len(val1.id)
        buff.write(_struct_I.pack(length))
        pattern = '<%si'%length
        buff.write(val1.id.tostring())
        length = len(val1.size)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(val1.size.tostring())
        _v15 = val1.pose
        _v16 = _v15.header
        _x = _v16.seq
        buff.write(_get_struct_I().pack(_x))
        _v17 = _v16.stamp
        _x = _v17
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v16.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v18 = _v15.pose
        _v19 = _v18.pose
        _v20 = _v19.position
        _x = _v20
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v21 = _v19.orientation
        _x = _v21
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        buff.write(_v18.covariance.tostring())
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
      if self.tag_detections is None:
        self.tag_detections = apriltag_ros.msg.AprilTagDetectionArray()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.tag_detections.header.seq, _x.tag_detections.header.stamp.secs, _x.tag_detections.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tag_detections.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tag_detections.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.tag_detections.detections = []
      for i in range(0, length):
        val1 = apriltag_ros.msg.AprilTagDetection()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%si'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.id = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.size = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
        _v22 = val1.pose
        _v23 = _v22.header
        start = end
        end += 4
        (_v23.seq,) = _get_struct_I().unpack(str[start:end])
        _v24 = _v23.stamp
        _x = _v24
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v23.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v23.frame_id = str[start:end]
        _v25 = _v22.pose
        _v26 = _v25.pose
        _v27 = _v26.position
        _x = _v27
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v28 = _v26.orientation
        _x = _v28
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        start = end
        end += 288
        _v25.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
        self.tag_detections.detections.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_36d = None
def _get_struct_36d():
    global _struct_36d
    if _struct_36d is None:
        _struct_36d = struct.Struct("<36d")
    return _struct_36d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
class AnalyzeSingleImage(object):
  _type          = 'apriltag_ros/AnalyzeSingleImage'
  _md5sum = 'd60d994450f73cbdba772751d78c9952'
  _request_class  = AnalyzeSingleImageRequest
  _response_class = AnalyzeSingleImageResponse
