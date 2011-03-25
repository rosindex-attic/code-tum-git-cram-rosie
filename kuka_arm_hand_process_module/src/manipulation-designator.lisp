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

(in-package :kuka-manip-desig)

(defclass trajectory-action ()
  ((side :initform nil :reader side :initarg :side
         :documentation "Which arm :right or :left")
   (trajectory-type :initform "" :reader trajectory-type :initarg :trajectory-type
                    :documentation "The type of the action: :cartesian, :reach-primitive and :joint-pos")
   (stored-pose-type :initform "" :reader stored-pose-type :initarg :store-pose-type
                     :documentation "One of the stored joint based poses, :navigate...")
   (object-type :initform "" :reader object-type :initarg :object-type
                :documentation "The object type (e.g :top)")
   (hand-primitive :initform "" :reader hand-primitive :initarg :hand-primitive
                   :documentation "The hand primitive to use.")
   (end-effector-pose :initform 0 :reader end-effector-pose :initarg :end-effector-pose
                      :documentation "JLO of the commanded end effector pose.")
   (obstacles :initform nil :reader obstacles :initarg :obstacles
              :documentation "List of jlo objects that describe obstacles")
   (grasp-distance :initform 0 :reader grasp-distance :initarg :grasp-distance
                   :documentation "Stop at this distance to the commanded pose.")
   (supporting-plane :initform 0 :reader supporting-plane :initarg :supporting-plane
                     :documentation "Height of the supporting plane (table).")))

(defun copy-trajectory-action (ref)
  (make-instance 'trajectory-action
    :side  (slot-value ref 'side)
    :trajectory-type (slot-value ref 'trajectory-type)
    :store-pose-type (slot-value ref 'stored-pose-type)
    :object-type (slot-value ref 'object-type)
    :hand-primitive (slot-value ref 'hand-primitive)
    :end-effector-pose (slot-value ref 'end-effector-pose)
    :obstacles (slot-value ref 'obstacles)
    :grasp-distance (slot-value ref 'grasp-distance)
    :supporting-plane (slot-value ref 'supporting-plane)))

(defun get-jlo-ids (objs)
  (reduce (lambda (ids obj)
            (typecase (reference obj)
              (perception-pm:cop-perceived-object
                 (cons (jlo:id (perception-pm:object-jlo (reference obj)))
                       ids))
              (t ids)))
          objs :initial-value nil))

(defun calculate-put-down-pose (dest-loc-desig obj-desig)
  (check-type dest-loc-desig location-designator)
  (check-type obj-desig object-designator)
  (with-desig-props (at) (current-desig obj-desig)
    (assert at ()
            "Object designator `~a' does not contain a location."
            (current-desig obj-desig))
    (with-desig-props (pose orientation)
        at
      (assert pose () "Location designator `~a' does not contain a valid pose." at)
      (assert orientation () "Location designator `~a' does not contain a valid orientation" at)
      (let* ((put-down-in-base (cl-tf:transform-pose
                                *tf* :pose (reference dest-loc-desig)
                                :target-frame "/base_link"))
             (grasp-offset (cl-transforms:rotate
                            orientation
                            (cl-transforms:v-inv (cl-transforms:origin pose))))
             (put-down-pose (cl-tf:make-pose-stamped
                             "/base_link" (roslisp:ros-time)
                             (cl-transforms:v+
                              (cl-transforms:origin put-down-in-base)
                              grasp-offset
                              (cl-transforms:make-3d-vector 0 0 0.005))
                             orientation)))
        (roslisp:ros-info (kuka-arm-hand process-module)
                          "Using put-down-pose ~a with offset ~a" put-down-pose grasp-offset)
        put-down-pose))))

(defun side-str (side)
  (etypecase side
    (symbol (string-downcase (symbol-name side)))
    (string side)))

(defun calculate-lift-pose (side &optional (distance 0.20))
  (let ((gripper-pose (cl-tf:lookup-transform
                       *tf*
                       :target-frame "/base_link"
                       :source-frame (concatenate
                                      'string
                                      "/"
                                      (side-str side)
                                      "_arm_hand_link")))
        (offset-transform (cl-transforms:make-transform
                           (cl-transforms:make-3d-vector 0 0 distance)
                           (cl-transforms:make-quaternion 0 0 0 1))))
    (assert gripper-pose () "Cannot look up gripper frame.")
    (let ((lift-pose (cl-transforms:transform* offset-transform gripper-pose)))
      (cl-tf:make-pose-stamped
       "/base_link" (cl-tf:stamp gripper-pose)
       (cl-transforms:translation lift-pose)
       (cl-transforms:rotation lift-pose)))))

(defun get-grasp-type (side)
  (let ((hand-trans (cl-transforms:transform->matrix
                     (cl-tf:lookup-transform
                      *tf*
                      :target-frame "/base_link"
                      :source-frame (ecase side
                                      (:right "/right_arm_hand_link")
                                      (:left "/left_arm_hand_link"))))))
    ;; We represent our hand transformation as a matrix. Then we check
    ;; for the y-axis of the transform being parallel or orthogonal to
    ;; the z axis of base_link. In other words, if
    ;; 
    ;; (0 1) + (1 1) < (2 1),
    ;;
    ;; the y axis is approximately parallel to z in base_link,
    ;; otherwise orthogonal.
    (if (< (+ (abs (aref hand-trans 0 1))
              (abs (aref hand-trans 1 1)))
           (abs (aref hand-trans 2 1)))
        :side-grasp
        :top-grasp)))

(defun calculate-carry-pose (side &optional grasp)
  (let ((carried-obj (var-value '?o (car
                                     (force-ll
                                      (rete-holds '(object-in-hand ?o ?s)))))))
    (assert carried-obj () "Not carrying an object in ~a gripper." side)
    (ecase (or grasp (get-grasp-type side))
      (:side-grasp (ecase side
                     (:right
                        (cl-tf:make-pose-stamped
                         "/gs_cam" (roslisp:ros-time)
                         (cl-transforms:make-3d-vector 0.35 0.10 0.20)
                         (cl-transforms:q*
                          (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 1 0) (/ pi 2))
                          (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 1 0 0) pi))))
                     (:left
                        (cl-tf:make-pose-stamped
                         "/gs_cam" (roslisp:ros-time)
                         (cl-transforms:make-3d-vector -0.35 0.10 0.20)
                         (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 1 0) (/ pi 2))))))
      (:top-grasp (cl-tf:make-pose-stamped
                   "/gs_cam" (roslisp:ros-time)
                   (cl-transforms:make-3d-vector (ecase side
                                                   (:right 0.35)
                                                   (:left -0.35))
                                                 -0.10 0.25)
                   (cl-transforms:make-quaternion -0.5 0.5 0.5 0.5))))))

(defun obj-desig-jlo-id (desig)
  "Returns the jlo id of the current perceived object of the object
  designator. Throws an error if the object doesn't contain a
  PERCEIVED-OBJECT."
  (let ((po (reference (current-desig desig))))
    (assert po () "Designator `~a' doesn't containt a valid PERCEIVED-OBJECT.")
    (check-type (object-jlo po) jlo:jlo)
    (object-jlo po)))

;;; Possible trajectory configs:
;;; (to grasp)
;;; (to unhand)
;;; (to carry)
;;; (to navigate)

(def-fact-group rosie-manipulation-designators (action-desig)
  ;; (grasp-info ?obj-desig ?object-type ?hand_primitive ?distance-to-grasp)
  ;; (<- (grasp-info ?obj "Mug" "3pinch" 0.14)
  ;;   (desig-prop ?obj (type mug)))

  ;; (<- (grasp-info ?obj "Jug" "3pinch" 0.05)
  ;;   (desig-prop ?obj (type jug)))

  ;; (<- (grasp-info ?obj "IceTea" "3pinch" 0.10)
  ;;   (desig-prop ?obj (type icetea)))

  ;; (<- (grasp-info ?obj "front" "3pinch" 0.04)
  ;;   (desig-prop ?obj (type coke)))

  (<- (grasp-info ?obj "Cluster" "3pinch" 0.0)
    ;; (desig-prop ?obj (type ?_))
    )
  
  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (side ?side))
    (findall ?o (and (desig-prop ?desig (obstacle ?o))
                     (format "bla ~a~%" ?o))
             ?obstacles)
    (lisp-fun get-jlo-ids ?obstacles ?obstacle-ids)
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "reach_primitive")
    (lisp-fun obj-desig-jlo-id ?obj ?obj-loc-jlo)
    (slot-value ?act end-effector-pose ?obj-loc-jlo)
    (grasp-info ?obj ?object-type ?hand-primitive ?dist)
    (slot-value ?act object-type ?object-type)
    (slot-value ?act hand-primitive ?hand-primitive)
    (slot-value ?act grasp-distance ?dist)
    (slot-value ?act obstacles ?obstacle-ids)
    ;; Todo: infer supporting plane height (with liswip)
    )

  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (or (desig-prop ?desig (to navigate))
        (desig-prop ?desig (pose parked)))
    (desig-prop ?desig (side ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "arm_joint_pose")
    (slot-value ?act stored-pose-type "parking"))

  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (pose open))
    (desig-prop ?desig (side ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "arm_joint_pose")
    (slot-value ?act stored-pose-type "open"))

  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to show))
    (desig-prop ?desig (side ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (lisp-fun side-str ?side ?side-str)
    (slot-value ?act trajectory-type "arm_joint_pose")
    (slot-value ?act stored-pose-type "show"))
  
  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to lift))
    (desig-prop ?desig (side ?side))
    (findall ?o (desig-prop ?desig (obstacle ?o)) ?obstacles)
    (lisp-fun get-jlo-ids ?obstacles ?obstacle-ids)
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act obstacles ?obstacle-ids)
    (lisp-fun calculate-lift-pose ?side ?lift-pose)
    (lisp-fun pose->jlo ?lift-pose ?lift-jlo)
    (slot-value ?act trajectory-type "arm_cart_loid")
    (slot-value ?act end-effector-pose ?lift-jlo))

  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to carry))
    (desig-prop ?desig (side ?side))
    (findall ?o (desig-prop ?desig (obstacle ?o)) ?obstacles)
    (lisp-fun get-jlo-ids ?obstacles ?obstacle-ids)
    (instance-of trajectory-action ?act)
    (lisp-fun calculate-carry-pose ?side ?carry-pose)
    (lisp-fun pose->jlo ?carry-pose ?carry-jlo)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "arm_cart_loid")
    (slot-value ?act end-effector-pose ?carry-jlo)
    (slot-value ?act obstacles ?obstacle-ids))

  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (side ?side))
    (desig-prop ?desig (at ?loc-desig))
    (desig-prop ?desig (obj ?obj-desig))
    (findall ?o (desig-prop ?desig (obstacle ?o)) ?obstacles)
    (lisp-fun get-jlo-ids ?obstacles ?obstacle-ids)
    (grasp-info ?obj-desig ?obj-type ?_ ?_)
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "arm_cart_move_loid")
    (lisp-fun calculate-put-down-pose ?loc-desig ?obj-desig ?pose)
    (lisp-fun pose->jlo ?pose ?jlo)
    (slot-value ?act end-effector-pose ?jlo)
    (slot-value ?act obstacles ?obstacle-ids))

  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to open))
    (desig-prop ?desig (gripper ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "hand_primitive")
    (slot-value ?act hand-primitive "open_thumb90"))

  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to close))
    (desig-prop ?desig (gripper ?side))
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "hand_primitive")
    (slot-value ?act hand-primitive "3pinch"))

  (<- (action-desig ?desig ?act)
    (trajectory-desig? ?desig)
    (desig-prop ?desig (pose ?p))
    (desig-prop ?desig (side ?side))
    (or (lisp-type ?p cl-tf:stamped-transform)
        (lisp-type ?p cl-tf:pose-stamped))
    (lisp-fun pose->jlo ?p ?jlo)
    (instance-of trajectory-action ?act)
    (slot-value ?act side ?side)
    (slot-value ?act trajectory-type "arm_cart_loid")
    (slot-value ?act end-effector-pose ?jlo)))
