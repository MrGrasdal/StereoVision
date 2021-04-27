; Auto-generated. Do not edit!


(cl:in-package calibration-msg)


;//! \htmlinclude gnssGGA_status.msg.html

(cl:defclass <gnssGGA_status> (roslisp-msg-protocol:ros-message)
  ((numSat
    :reader numSat
    :initarg :numSat
    :type cl:integer
    :initform 0)
   (hDOP
    :reader hDOP
    :initarg :hDOP
    :type cl:float
    :initform 0.0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (modeStatus
    :reader modeStatus
    :initarg :modeStatus
    :type cl:string
    :initform ""))
)

(cl:defclass gnssGGA_status (<gnssGGA_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gnssGGA_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gnssGGA_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name calibration-msg:<gnssGGA_status> is deprecated: use calibration-msg:gnssGGA_status instead.")))

(cl:ensure-generic-function 'numSat-val :lambda-list '(m))
(cl:defmethod numSat-val ((m <gnssGGA_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calibration-msg:numSat-val is deprecated.  Use calibration-msg:numSat instead.")
  (numSat m))

(cl:ensure-generic-function 'hDOP-val :lambda-list '(m))
(cl:defmethod hDOP-val ((m <gnssGGA_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calibration-msg:hDOP-val is deprecated.  Use calibration-msg:hDOP instead.")
  (hDOP m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <gnssGGA_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calibration-msg:mode-val is deprecated.  Use calibration-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'modeStatus-val :lambda-list '(m))
(cl:defmethod modeStatus-val ((m <gnssGGA_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader calibration-msg:modeStatus-val is deprecated.  Use calibration-msg:modeStatus instead.")
  (modeStatus m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gnssGGA_status>) ostream)
  "Serializes a message object of type '<gnssGGA_status>"
  (cl:let* ((signed (cl:slot-value msg 'numSat)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'hDOP))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'modeStatus))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'modeStatus))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gnssGGA_status>) istream)
  "Deserializes a message object of type '<gnssGGA_status>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'numSat) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hDOP) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'modeStatus) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'modeStatus) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gnssGGA_status>)))
  "Returns string type for a message object of type '<gnssGGA_status>"
  "calibration/gnssGGA_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gnssGGA_status)))
  "Returns string type for a message object of type 'gnssGGA_status"
  "calibration/gnssGGA_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gnssGGA_status>)))
  "Returns md5sum for a message object of type '<gnssGGA_status>"
  "0e8745cbb7cfe1a00d99b4fdc87eeb1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gnssGGA_status)))
  "Returns md5sum for a message object of type 'gnssGGA_status"
  "0e8745cbb7cfe1a00d99b4fdc87eeb1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gnssGGA_status>)))
  "Returns full string definition for message of type '<gnssGGA_status>"
  (cl:format cl:nil "int64 numSat~%float64 hDOP~%int64 mode~%string modeStatus~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gnssGGA_status)))
  "Returns full string definition for message of type 'gnssGGA_status"
  (cl:format cl:nil "int64 numSat~%float64 hDOP~%int64 mode~%string modeStatus~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gnssGGA_status>))
  (cl:+ 0
     8
     8
     8
     4 (cl:length (cl:slot-value msg 'modeStatus))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gnssGGA_status>))
  "Converts a ROS message object to a list"
  (cl:list 'gnssGGA_status
    (cl:cons ':numSat (numSat msg))
    (cl:cons ':hDOP (hDOP msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':modeStatus (modeStatus msg))
))
