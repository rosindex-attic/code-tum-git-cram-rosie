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

(in-package :kuka-pm)

(defvar *left-arm-action-client* nil)
(defvar *right-arm-action-client* nil)

(define-condition manipulation-action-error (simple-plan-failure)
  ((final-status :initarg :final-status :reader final-status)
   (result :initarg :result :reader result)))

(defun manipulation-actions-init ()
  (setf *left-arm-action-client*
        (actionlib:make-action-client "/left_arm" "cogman_msgs/ArmHandAction"))
  (setf *right-arm-action-client*
        (actionlib:make-action-client "/right_arm" "cogman_msgs/ArmHandAction")))

(register-ros-init-function manipulation-actions-init)

(defun execute-arm-action (action-description)
  (let ((action-goal (make-instance 'cogman_msgs-msg:<armhandgoal>
                                    :command (trajectory-type action-description)
                                    :pose_name (format nil "~a_~a"
                                                       (string-downcase (symbol-name (side action-description)))
                                                       (stored-pose-type action-description))
                                    ;; :joint_angles (unused)
                                    :hand_primitive (hand-primitive action-description)
                                    :object_type (object-type action-description)
                                    :end_effector_loid (if (eql (end-effector-pose action-description) 0)
                                                           ;; Parameter not set, use default
                                                           0
                                                           (jlo:id (end-effector-pose action-description)))
                                    :obstacle_ids (map 'vector #'identity (obstacles action-description))
                                    :distance (grasp-distance action-description)
                                    :supporting_plane 0.86
                                    )))
    (multiple-value-bind (result state)
        (ecase (side action-description)
          (:left
             (roslisp:ros-info (kuka-manip process-module) "sending left goal.")
             (actionlib:call-goal *left-arm-action-client* action-goal))
          (:right
             (roslisp:ros-info (kuka-manip process-module) "sending right goal.")
             (actionlib:call-goal *right-arm-action-client* action-goal)))
      (roslisp:ros-info (kuka-manip process-module) "Goal terminated with status `~a'." state)
      (if (eq state :succeeded)
          result
          (error 'manipulation-action-error :final-status state :result result)))))
