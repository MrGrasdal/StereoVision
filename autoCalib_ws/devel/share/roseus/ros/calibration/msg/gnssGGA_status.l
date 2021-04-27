;; Auto-generated. Do not edit!


(when (boundp 'calibration::gnssGGA_status)
  (if (not (find-package "CALIBRATION"))
    (make-package "CALIBRATION"))
  (shadow 'gnssGGA_status (find-package "CALIBRATION")))
(unless (find-package "CALIBRATION::GNSSGGA_STATUS")
  (make-package "CALIBRATION::GNSSGGA_STATUS"))

(in-package "ROS")
;;//! \htmlinclude gnssGGA_status.msg.html


(defclass calibration::gnssGGA_status
  :super ros::object
  :slots (_numSat _hDOP _mode _modeStatus ))

(defmethod calibration::gnssGGA_status
  (:init
   (&key
    ((:numSat __numSat) 0)
    ((:hDOP __hDOP) 0.0)
    ((:mode __mode) 0)
    ((:modeStatus __modeStatus) "")
    )
   (send-super :init)
   (setq _numSat (round __numSat))
   (setq _hDOP (float __hDOP))
   (setq _mode (round __mode))
   (setq _modeStatus (string __modeStatus))
   self)
  (:numSat
   (&optional __numSat)
   (if __numSat (setq _numSat __numSat)) _numSat)
  (:hDOP
   (&optional __hDOP)
   (if __hDOP (setq _hDOP __hDOP)) _hDOP)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:modeStatus
   (&optional __modeStatus)
   (if __modeStatus (setq _modeStatus __modeStatus)) _modeStatus)
  (:serialization-length
   ()
   (+
    ;; int64 _numSat
    8
    ;; float64 _hDOP
    8
    ;; int64 _mode
    8
    ;; string _modeStatus
    4 (length _modeStatus)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _numSat
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _numSat (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _numSat) (= (length (_numSat . bv)) 2)) ;; bignum
              (write-long (ash (elt (_numSat . bv) 0) 0) s)
              (write-long (ash (elt (_numSat . bv) 1) -1) s))
             ((and (class _numSat) (= (length (_numSat . bv)) 1)) ;; big1
              (write-long (elt (_numSat . bv) 0) s)
              (write-long (if (>= _numSat 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _numSat s)(write-long (if (>= _numSat 0) 0 #xffffffff) s)))
     ;; float64 _hDOP
       (sys::poke _hDOP (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _mode
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _mode (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _mode) (= (length (_mode . bv)) 2)) ;; bignum
              (write-long (ash (elt (_mode . bv) 0) 0) s)
              (write-long (ash (elt (_mode . bv) 1) -1) s))
             ((and (class _mode) (= (length (_mode . bv)) 1)) ;; big1
              (write-long (elt (_mode . bv) 0) s)
              (write-long (if (>= _mode 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _mode s)(write-long (if (>= _mode 0) 0 #xffffffff) s)))
     ;; string _modeStatus
       (write-long (length _modeStatus) s) (princ _modeStatus s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _numSat
#+(or :alpha :irix6 :x86_64)
      (setf _numSat (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _numSat (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _hDOP
     (setq _hDOP (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _mode
#+(or :alpha :irix6 :x86_64)
      (setf _mode (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _mode (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _modeStatus
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _modeStatus (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get calibration::gnssGGA_status :md5sum-) "0e8745cbb7cfe1a00d99b4fdc87eeb1f")
(setf (get calibration::gnssGGA_status :datatype-) "calibration/gnssGGA_status")
(setf (get calibration::gnssGGA_status :definition-)
      "int64 numSat
float64 hDOP
int64 mode
string modeStatus
")



(provide :calibration/gnssGGA_status "0e8745cbb7cfe1a00d99b4fdc87eeb1f")


