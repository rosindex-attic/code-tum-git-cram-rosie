;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :rex-reasoning)

(defvar *cop-feedback-pub* nil)

(defun init-cop-feedback ()
  (setf *cop-feedback-pub*
        (advertise
         "/cop/feedback"
         "vision_msgs/cop_feedback")))
(register-ros-init-function init-cop-feedback)

;;; publish a cop_feedback message on the corresponding cop topic.
;;; The perception_primitive is received in the cop_feedback message
;;; and should be in the designator passed to the callbacks.
(defun cop-successful-pick-up (op &key ?obj ?side)
  (declare (ignore ?side))
  (when (eq op :assert)
    (let ((perceived-object (reference (newest-valid-designator ?obj))))
      (when (typep perceived-object 'cop-perceived-object)
        (publish *cop-feedback-pub*
                 (make-message
                  "vision_msgs/cop_feedback"
                  perception_primitive (perception-primitive perceived-object)
                  eval_whitelist (vector (object-id perceived-object))
                  evaluation 1.0))))))

(defun cop-failed-pick-up (op &key ?f ?obj ?side ?error)
  (declare (ignore ?side))
  (when (eq op :assert)
    (with-fields ((error-code (error_id error))) (cram-plan-failures:result ?error)
      (let ((perceived-object (reference (newest-valid-designator ?obj))))
        (when (typep perceived-object 'cop-perceived-object)
          (publish *cop-feedback-pub*
                   (make-message
                    "vision_msgs/cop_feedback"
                    perception_primitive (perception-primitive
                                          perceived-object)
                    eval_whitelist (vector (object-id perceived-object))
                    evaluation 0.0
                    error (vector
                           (make-message
                            "vision_msgs/system_error"
                            error_id error-code
                            node_name *ros-node-name*
                            error_description (symbol-name ?f))))))))))

(defun cop-object-not-found (op &key ?f)
  (when (eq op :assert)
    (handler-case
        (let* ((desig (newest-valid-designator
                       (cram-plan-failures:object-not-found-desig ?f)))
               (perceived-object (and desig (valid desig) (reference desig))))
          (when (and
                 desig perceived-object
                 (typep perceived-object 'cop-perceived-object))
            (publish *cop-feedback-pub*
                     (make-message
                      "vision_msgs/cop_feedback"
                      perception_primitive (perception-primitive
                                            perceived-object)
                      eval_whitelist (vector (object-id perceived-object))
                      evaluation 0.0
                      error (vector
                             (make-message
                              "vision_msgs/system_error"
                              error_id (roslisp-msg-protocol:symbol-code
                                        'vision_msgs-msg:system_error
                                        :contradicting_vision_results)
                              node_name *ros-node-name*
                              error_description "could not re-locate object"))))))
      (designator-error () nil))))

(defun cop-object-relocated (obj &optional (distance 0.0))
  (let ((perceived-object (and (valid obj) (reference obj))))
    (ros-info (cop feedback) "distance: ~a~%" distance)
    (when perceived-object
      (publish *cop-feedback-pub*
               (make-message
                "vision_msgs/cop_feedback"
                perception_primitive (perception-primitive perceived-object)
                eval_whitelist (vector (object-id perceived-object))
                evaluation (if (> distance 0.4) 0 1))))))

(register-production-handler 'object-picked-up #'cop-successful-pick-up)
(register-production-handler 'object-in-hand-failure #'cop-failed-pick-up)
(register-production-handler 'object-not-found-failure #'cop-object-not-found)
