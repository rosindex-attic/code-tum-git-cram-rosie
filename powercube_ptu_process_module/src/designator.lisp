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

(in-package :ptu-pm)

;;; We need to use a wrapper class for the action goal since we need
;;; to keep a jlo proxy object at least until we send the action.
(defclass ptu-goal ()
  ((jlo :reader jlo :initarg :jlo)
   (mode :reader mode :initarg :mode)))

(defun make-action-goal (ptu-goal)
  (declare (type ptu-goal ptu-goal))
  (roslisp:make-message "cogman_msgs/PtuGoal"
                        lo_id (jlo:id (jlo ptu-goal))
                        mode (ecase (mode ptu-goal)
                               (:point 0)
                               (:follow 1))))

(defun as-jlo (pose)
  (typecase pose
    (jlo:jlo
       (roslisp:ros-warn (ptu process-module)
                         "Received deprecated pose type JLO")
       pose)
    (t (pose->jlo pose))))

(def-fact-group powercube-ptu (action-desig)

  (<- (ptu-action-goal ?pose ?mode ?goal)
    (instance-of ptu-goal ?goal)
    (lispfun as-jlo ?pose ?jlo)
    (slot-value ?goal jlo ?jlo)
    (slot-value ?goal mode ?mode))
  
  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to see))
    (desig-prop ?desig (pose ?pose))
    (ptu-action-goal ?pose :point ?act))

  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to follow))
    (desig-prop ?desig (pose ?pose))
    (ptu-action-goal ?pose :follow ?act)))