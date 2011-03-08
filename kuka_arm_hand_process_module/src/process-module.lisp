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

(defvar *manipulation-action-designator* nil)

(def-process-module kuka-arm-hand-manipulation (input)
  (let ((action (reference input)))
    (roslisp:ros-info
     (kuka-manip process-module)
     "[Manipulation process module] received input ~a~%"
     (description input))
    (setf *manipulation-action-designator* input)
    (with-failure-handling
        ((manipulation-action-error (e)
           (roslisp:ros-warn
            (kuka-manip process-module)
            "Received MANIPULATION-ACTION-ERROR condition (terminal state ~a)."
            (final-status e))
           (roslisp:with-fields ((err (error_id error))) (result e)
             (case (car (rassoc
                         err
                         (roslisp-msg-protocol:symbol-codes
                          'vision_msgs-msg:system_error)))
               (:manipulation_pose_unreachable
                  (fail 'manipulation-pose-unreachable :result (result e)))
               (:grasp_failed
                  (fail 'manipulation-failed :result (result e)))
               (:contradicting_tactile_feedback
                  (return (result e)))
               (t (fail 'manipulation-failed :result (result e)))))))
      (ecase (side action)
        (:left (execute-arm-action action))
        (:right (execute-arm-action action))
        (:both (let ((left (copy-trajectory-action action))
                     (right (copy-trajectory-action action)))
                 (setf (slot-value left 'side) :left)
                 (setf (slot-value right 'side) :right)
                 (par
                   (execute-arm-action left)
                   (execute-arm-action right)))))))
  (roslisp:ros-info (kuka-manip process-module) "[Manipulation process module] returning."))
