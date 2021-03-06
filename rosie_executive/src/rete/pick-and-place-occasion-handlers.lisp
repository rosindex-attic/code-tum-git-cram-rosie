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

(defun find-desig-z-value (obj-desig pose)
  (let* ((loc-desig (cadr (find 'at (description obj-desig)
                                :key #'car)))
         (z-value-bdgs (lazy-car
                        (prolog
                         `(desig-z-value ,loc-desig
                                         ,(cl-transforms:origin pose) ?z)))))
    (cond (z-value-bdgs
           (var-value '?z z-value-bdgs))
          ((parent obj-desig)
           (find-desig-z-value (parent obj-desig) pose)))))

(defun on-object-picked-up (op &key ?obj ?side)
  (when (eq op :assert)
    (let* ((obj-pose (cl-tf:transform-pose
                      *tf* :pose (obj-desig-location ?obj)
                      :target-frame "/map"))
           (hand-in-base (cl-tf:lookup-transform
                          *tf*
                          :target-frame "/base_link"
                          :source-frame (format nil "/~a_arm_hand_link"
                                                (string-downcase (symbol-name ?side)))))
           (height-value (or (when (value *table-height-map-fl*)
                               (height-map-lookup
                                (value *table-height-map-fl*)
                                (cl-transforms:x (cl-transforms:origin obj-pose))
                                (cl-transforms:y (cl-transforms:origin obj-pose))))
                             (find-desig-z-value ?obj obj-pose)
                             (prog1
                                 (tf:z (cl-transforms:origin obj-pose))
                               (ros-warn
                                on-object-picked-up
                                "Warning: could not infer valid z value for object pose."))))
           (new-loc (make-designator
                     'location
                     `((in gripper)
                       (side ,?side)
                       (pose ,(cl-tf:transform-pose
                               *tf* :pose (cl-tf:make-pose-stamped
                                           (cl-tf:frame-id obj-pose) (ros-time)
                                           (cl-transforms:make-3d-vector
                                            (cl-transforms:x (cl-transforms:origin obj-pose))
                                            (cl-transforms:y (cl-transforms:origin obj-pose))
                                            height-value)
                                           (cl-transforms:orientation obj-pose))
                               :target-frame (format nil "/~a_arm_hand_link"
                                                     (string-downcase (symbol-name ?side)))))
                       (height ,(-  (cl-transforms:z (cl-transforms:origin obj-pose))
                                    height-value))
                       ;;; UGLY HACK: since we cannot reach all
                       ;;; put-down locations, we currently use the
                       ;;; hand orientation we used while picking up
                       ;;; the object
                       (orientation ,(cl-transforms:rotation hand-in-base))))))
      (make-designator 'object
                       `((at ,new-loc) . ,(remove 'at (description ?obj) :key #'car))
                       ?obj))))

(crs:register-production-handler 'object-picked-up #'on-object-picked-up)

