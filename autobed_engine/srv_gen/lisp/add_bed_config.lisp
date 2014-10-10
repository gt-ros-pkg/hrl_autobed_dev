; Auto-generated. Do not edit!


(cl:in-package hrl_autobed_dev-srv)


;//! \htmlinclude add_bed_config-request.msg.html

(cl:defclass <add_bed_config-request> (roslisp-msg-protocol:ros-message)
  ((config
    :reader config
    :initarg :config
    :type cl:string
    :initform ""))
)

(cl:defclass add_bed_config-request (<add_bed_config-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <add_bed_config-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'add_bed_config-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hrl_autobed_dev-srv:<add_bed_config-request> is deprecated: use hrl_autobed_dev-srv:add_bed_config-request instead.")))

(cl:ensure-generic-function 'config-val :lambda-list '(m))
(cl:defmethod config-val ((m <add_bed_config-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrl_autobed_dev-srv:config-val is deprecated.  Use hrl_autobed_dev-srv:config instead.")
  (config m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <add_bed_config-request>) ostream)
  "Serializes a message object of type '<add_bed_config-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'config))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'config))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <add_bed_config-request>) istream)
  "Deserializes a message object of type '<add_bed_config-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'config) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'config) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<add_bed_config-request>)))
  "Returns string type for a service object of type '<add_bed_config-request>"
  "hrl_autobed_dev/add_bed_configRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'add_bed_config-request)))
  "Returns string type for a service object of type 'add_bed_config-request"
  "hrl_autobed_dev/add_bed_configRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<add_bed_config-request>)))
  "Returns md5sum for a message object of type '<add_bed_config-request>"
  "b43b6e2f4ff2ffe6e9e8f6e13cce1eb5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'add_bed_config-request)))
  "Returns md5sum for a message object of type 'add_bed_config-request"
  "b43b6e2f4ff2ffe6e9e8f6e13cce1eb5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<add_bed_config-request>)))
  "Returns full string definition for message of type '<add_bed_config-request>"
  (cl:format cl:nil "string config~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'add_bed_config-request)))
  "Returns full string definition for message of type 'add_bed_config-request"
  (cl:format cl:nil "string config~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <add_bed_config-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'config))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <add_bed_config-request>))
  "Converts a ROS message object to a list"
  (cl:list 'add_bed_config-request
    (cl:cons ':config (config msg))
))
;//! \htmlinclude add_bed_config-response.msg.html

(cl:defclass <add_bed_config-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass add_bed_config-response (<add_bed_config-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <add_bed_config-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'add_bed_config-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hrl_autobed_dev-srv:<add_bed_config-response> is deprecated: use hrl_autobed_dev-srv:add_bed_config-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <add_bed_config-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrl_autobed_dev-srv:success-val is deprecated.  Use hrl_autobed_dev-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <add_bed_config-response>) ostream)
  "Serializes a message object of type '<add_bed_config-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <add_bed_config-response>) istream)
  "Deserializes a message object of type '<add_bed_config-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<add_bed_config-response>)))
  "Returns string type for a service object of type '<add_bed_config-response>"
  "hrl_autobed_dev/add_bed_configResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'add_bed_config-response)))
  "Returns string type for a service object of type 'add_bed_config-response"
  "hrl_autobed_dev/add_bed_configResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<add_bed_config-response>)))
  "Returns md5sum for a message object of type '<add_bed_config-response>"
  "b43b6e2f4ff2ffe6e9e8f6e13cce1eb5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'add_bed_config-response)))
  "Returns md5sum for a message object of type 'add_bed_config-response"
  "b43b6e2f4ff2ffe6e9e8f6e13cce1eb5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<add_bed_config-response>)))
  "Returns full string definition for message of type '<add_bed_config-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'add_bed_config-response)))
  "Returns full string definition for message of type 'add_bed_config-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <add_bed_config-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <add_bed_config-response>))
  "Converts a ROS message object to a list"
  (cl:list 'add_bed_config-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'add_bed_config)))
  'add_bed_config-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'add_bed_config)))
  'add_bed_config-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'add_bed_config)))
  "Returns string type for a service object of type '<add_bed_config>"
  "hrl_autobed_dev/add_bed_config")