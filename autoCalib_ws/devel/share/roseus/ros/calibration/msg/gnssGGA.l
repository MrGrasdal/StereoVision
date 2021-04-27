;; Auto-generated. Do not edit!


(when (boundp 'calibration::gnssGGA)
  (if (not (find-package "CALIBRATION"))
    (make-package "CALIBRATION"))
  (shadow 'gnssGGA (find-package "CALIBRATION")))
(unless (find-package "CALIBRATION::GNSSGGA")
  (make-package "CALIBRATION::GNSSGGA"))

(in-package "ROS")
;;//! \htmlinclude gnssGGA.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass calibration::gnssGGA
  :super ros::object
  :slots (_header _sat_time _latitude _lat_direction _longitude _lon_direction _altitude _status ))

(defmethod calibration::gnssGGA
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sat_time __sat_time) (instance ros::time :init))
    ((:latitude __latitude) 0.0)
    ((:lat_direction __lat_direction) "")
    ((:longitude __longitude) 0.0)
    ((:lon_direction __lon_direction) "")
    ((:altitude __altitude) 0.0)
    ((:status __status) (instance calibration::gnssGGA_status :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _sat_time __sat_time)
   (setq _latitude (float __latitude))
   (setq _lat_direction (string __lat_direction))
   (setq _longitude (float __longitude))
   (setq _lon_direction (string __lon_direction))
   (setq _altitude (float __altitude))
   (setq _status __status)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sat_time
   (&optional __sat_time)
   (if __sat_time (setq _sat_time __sat_time)) _sat_time)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:lat_direction
   (&optional __lat_direction)
   (if __lat_direction (setq _lat_direction __lat_direction)) _lat_direction)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:lon_direction
   (&optional __lon_direction)
   (if __lon_direction (setq _lon_direction __lon_direction)) _lon_direction)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _sat_time
    8
    ;; float64 _latitude
    8
    ;; string _lat_direction
    4 (length _lat_direction)
    ;; float64 _longitude
    8
    ;; string _lon_direction
    4 (length _lon_direction)
    ;; float64 _altitude
    8
    ;; calibration/gnssGGA_status _status
    (send _status :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; time _sat_time
       (write-long (send _sat_time :sec) s) (write-long (send _sat_time :nsec) s)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _lat_direction
       (write-long (length _lat_direction) s) (princ _lat_direction s)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _lon_direction
       (write-long (length _lon_direction) s) (princ _lon_direction s)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; calibration/gnssGGA_status _status
       (send _status :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; time _sat_time
     (send _sat_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _sat_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _lat_direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lat_direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _lon_direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lon_direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; calibration/gnssGGA_status _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;;
   self)
  )

(setf (get calibration::gnssGGA :md5sum-) "76d41a768710775209ac34b58a4ce202")
(setf (get calibration::gnssGGA :datatype-) "calibration/gnssGGA")
(setf (get calibration::gnssGGA :definition-)
      "#Something something

Header header

time sat_time

float64 latitude
string lat_direction
float64 longitude
string lon_direction

float64 altitude

gnssGGA_status status
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
MSG: calibration/gnssGGA_status
int64 numSat
float64 hDOP
int64 mode
string modeStatus
")



(provide :calibration/gnssGGA "76d41a768710775209ac34b58a4ce202")


