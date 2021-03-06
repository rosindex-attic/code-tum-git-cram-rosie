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

(desig-props:def-desig-package kuka-arm-hand-designator
    (:documentation "Package for reasoning and designator related stuff.")
  (:use #:common-lisp
        #:crs
        #:cut
        #:desig
        #:designators-ros
        #:cram-roslisp-common
        #:cram-plan-failures
        #:cram-plan-knowledge
        #:cljlo-utils)
  (:nicknames :kuka-manip-desig)
  (:export #:trajectory-action #:trajectory-type #:stored-pose-type
           #:object-type #:hand-primitive #:end-effector-pose
           #:obstacles #:grasp-distance #:supporting-plane
           #:copy-trajectory-action)
  (:import-from #:perception-process-module
                #:object-jlo)
  (:desig-properties  #:grasp #:obj #:side #:to #:navigate #:pose #:parked
                      #:type #:trajectory #:open #:show #:carry #:lift
                      #:put-down #:at #:gripper #:close #:pose #:orientation
                      #:obstacle #:pre-put-down #:open-cart))

#.`(defpackage kuka-arm-hand-process-module
       (:documentation "Package for the kuka arm-hand process module")
     (:use #:cpl
           #:cram-process-modules
           #:cram-roslisp-common
           #:kuka-arm-hand-designator
           #:cram-plan-failures
           #:desig)
     (:nicknames :kuka-pm)
     (:shadowing-import-from #:desig #:name)
     (:export
      #:kuka-arm-hand-manipulation
      ,@(let ((r nil))
          (do-external-symbols (s :kuka-arm-hand-designator r) (push s r)))))

