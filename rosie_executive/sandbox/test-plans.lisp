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

(in-package :rex)

(def-top-level-plan nav-to (loc)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((loc (location `((on table) (name ,loc))))
                       (see-loc (location `((to see) (location ,loc)))))
      (location-costmap:publish-pose (reference see-loc))
      (achieve `(loc Robot ,see-loc)))))

(def-top-level-plan perceive-objects (loc)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((loc (location `((on table) (name ,loc))))
                       (obj (object `((type object) (at ,loc)))))
      (perceive-all obj))))

(def-top-level-plan re-perceive-obj (desig)
  (pursue
    (maybe-run-process-modules)
    (seq
      (perceive-all desig))))

(def-top-level-plan perceive-obj (obj-props place)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((loc (location `((on table) (name ,place))))
                       (obj (object `((at ,loc) . ,obj-props))))
      (perceive obj))))

(def-top-level-plan pick-and-place-obj (obj-properties from to)
  "Picks up an object that matches `obj-properties' from location
`from' and places it at location `to'. `from' and `to' are lispified
names of points that are provided by the map_annotation server."
  (pursue
    (maybe-run-process-modules)
    (with-designators ((from (location `((on counter-top) (name ,from))))
                       (to (location `((on counter-top) (name ,to) (in reach))))
                       (obj (object `(,@obj-properties (at ,from))))
                       )
      (achieve `(loc ,obj ,to))
      (with-designators ((obj-see-loc (location `((to see) (obj ,obj)))))
        (at-location (obj-see-loc)
          (achieve '(arm-parked :both))))
      (perceive obj))))

(def-top-level-plan park-arms ()
  (pursue
    (maybe-run-process-modules)
    (seq
      (achieve '(arm-parked :both)))))

(def-top-level-plan test-reach ()
  (pursue
    (maybe-run-process-modules)
    (loop for i from 1 to 100 do
         (with-designators ((loc (location `((on table))))
                            (obj (object `((type jug) (at ,loc)))))
           (achieve `(object-in-hand ,obj :left))
           (clear-belief)
           (sleep 10)))))

(def-top-level-plan right-carry ()
  (pursue
    (maybe-run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((carry-desig (action '((type trajectory) (to carry) (side :right)))))
        (achieve `(arms-at ,carry-desig))))))

(def-top-level-plan both-open ()
  (pursue
    (maybe-run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((open-desig (action '((type trajectory) (pose open) (side :both)))))
        (achieve `(arms-at ,open-desig))))))
        
(def-top-level-plan both-closed ()
  (pursue
    (maybe-run-process-modules)
    (seq
      (sleep 0.5)
      (with-designators ((close-desig (action '((type trajectory) (to close) (gripper :both)))))
        (achieve `(arms-at ,close-desig))))))

(def-top-level-plan pick-and-place-closest-obj (obj-properties from to)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((from (location `((on counter-top) (name ,from))))
                       (to (location `((on counter-top) (name ,to) (in reach))))
                       (obj (object `(,@obj-properties (at ,from)))))
      (let* ((objs (perceive-all obj))
             (closest-obj (closest-object objs)))
        (equate obj closest-obj)
        (achieve `(loc ,closest-obj ,to))
        (with-designators ((obj-see-loc (location `((to see) (obj ,closest-obj)))))
          (at-location (obj-see-loc)
            (achieve '(arm-parked :both))))
        (perceive closest-obj)))))

(def-top-level-plan pick-and-place-right-most-obj (obj-properties from to)
  (pursue
    (maybe-run-process-modules)
    (with-designators ((from (location `((on counter-top) (name ,from)
                                         (pose ,(cl-tf:make-pose-stamped
                                                 "/map" 0.0
                                                 (cl-transforms:make-3d-vector -2.18 1.37 0.92)
                                                 (cl-transforms:make-quaternion 0 0 0 1))))))
                       (to (location `((on counter-top) (name ,to) (in reach)
                                       (pose ,(cl-tf:make-pose-stamped
                                               "/map" 0.0
                                               (cl-transforms:make-3d-vector -2.1 1.75 0.86)
                                               (cl-transforms:make-quaternion 0 0 0 1))))))
                       (obj (object `(,@obj-properties (at ,from)))))
      (let* ((objs (perceive-all obj))
             (right-most-object (right-most-object objs)))
        (equate obj right-most-object)
        (achieve `(loc ,right-most-object ,to))
        (setf right-most-object (perceive right-most-object))
        (rex-reasoning:cop-object-relocated right-most-object)))))

(defun closest-object (objects)
  (car
   (sort (mapcar #'identity objects)
         (lambda (o-1 o-2)
           (<
            (cl-transforms:v-norm (cl-transforms:origin
                                   (tf:transform-pose
                                    *tf* :target-frame "/base_footprint"
                                    :pose (perception-pm:object-pose o-1))))
            (cl-transforms:v-norm (cl-transforms:origin
                                   (tf:transform-pose
                                    *tf* :target-frame "/base_footprint"
                                    :pose (perception-pm:object-pose o-2))))))
         :key #'reference)))

(defun right-most-object (objects)
  (car
   (sort (mapcar #'identity objects)
         (lambda (o-1 o-2)
           (<
            (cl-transforms:y (cl-transforms:origin
                              (tf:transform-pose
                               *tf* :target-frame "/base_footprint"
                               :pose (perception-pm:object-pose o-1))))
            (cl-transforms:y (cl-transforms:origin
                              (tf:transform-pose
                               *tf* :target-frame "/base_footprint"
                               :pose (perception-pm:object-pose o-2))))))
         :key #'reference)))

(defun left-most-object (objects)
  (car
   (sort (mapcar #'identity objects)
         (lambda (o-1 o-2)
           (>
            (cl-transforms:y (cl-transforms:origin
                              (tf:transform-pose
                               *tf* :target-frame "/base_footprint"
                               :pose (perception-pm:object-pose o-1))))
            (cl-transforms:y (cl-transforms:origin
                              (tf:transform-pose
                               *tf* :target-frame "/base_footprint"
                               :pose (perception-pm:object-pose o-2))))))
         :key #'reference)))
