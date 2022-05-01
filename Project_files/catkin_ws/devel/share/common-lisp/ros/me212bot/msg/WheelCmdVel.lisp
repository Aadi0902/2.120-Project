; Auto-generated. Do not edit!


(cl:in-package me212bot-msg)


;//! \htmlinclude WheelCmdVel.msg.html

(cl:defclass <WheelCmdVel> (roslisp-msg-protocol:ros-message)
  ((desiredWV_R
    :reader desiredWV_R
    :initarg :desiredWV_R
    :type cl:float
    :initform 0.0)
   (desiredWV_L
    :reader desiredWV_L
    :initarg :desiredWV_L
    :type cl:float
    :initform 0.0))
)

(cl:defclass WheelCmdVel (<WheelCmdVel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelCmdVel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelCmdVel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name me212bot-msg:<WheelCmdVel> is deprecated: use me212bot-msg:WheelCmdVel instead.")))

(cl:ensure-generic-function 'desiredWV_R-val :lambda-list '(m))
(cl:defmethod desiredWV_R-val ((m <WheelCmdVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader me212bot-msg:desiredWV_R-val is deprecated.  Use me212bot-msg:desiredWV_R instead.")
  (desiredWV_R m))

(cl:ensure-generic-function 'desiredWV_L-val :lambda-list '(m))
(cl:defmethod desiredWV_L-val ((m <WheelCmdVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader me212bot-msg:desiredWV_L-val is deprecated.  Use me212bot-msg:desiredWV_L instead.")
  (desiredWV_L m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelCmdVel>) ostream)
  "Serializes a message object of type '<WheelCmdVel>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'desiredWV_R))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'desiredWV_L))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelCmdVel>) istream)
  "Deserializes a message object of type '<WheelCmdVel>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desiredWV_R) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desiredWV_L) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelCmdVel>)))
  "Returns string type for a message object of type '<WheelCmdVel>"
  "me212bot/WheelCmdVel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelCmdVel)))
  "Returns string type for a message object of type 'WheelCmdVel"
  "me212bot/WheelCmdVel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelCmdVel>)))
  "Returns md5sum for a message object of type '<WheelCmdVel>"
  "428fbbfd1f38717ca7baa73045b4efaa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelCmdVel)))
  "Returns md5sum for a message object of type 'WheelCmdVel"
  "428fbbfd1f38717ca7baa73045b4efaa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelCmdVel>)))
  "Returns full string definition for message of type '<WheelCmdVel>"
  (cl:format cl:nil "float32 desiredWV_R~%float32 desiredWV_L~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelCmdVel)))
  "Returns full string definition for message of type 'WheelCmdVel"
  (cl:format cl:nil "float32 desiredWV_R~%float32 desiredWV_L~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelCmdVel>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelCmdVel>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelCmdVel
    (cl:cons ':desiredWV_R (desiredWV_R msg))
    (cl:cons ':desiredWV_L (desiredWV_L msg))
))
