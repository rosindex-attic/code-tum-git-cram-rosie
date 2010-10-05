
(in-package :rex-reasoning)

(defclass manipulaton-metadata ()
  ((obj-location :initarg :obj-location :accessor obj-location)
   (obj-type :initarg :obj-type :accessor obj-type)
   (location :initarg :location :accessor location)
   (status :initarg :success :accessor status)))

(defclass pick-and-plance-metadata ()
  ((pick-up-metadata :initarg :pick-up-metadata
                     :accessor pick-up-metadata)
   (put-down-metadata :initarg :put-down-metadata
                      :accessor put-down-metadata)))

(def-fact-group pick-and-place-analysis ()
  (<- (pick-up-obj-desig ?tsk ?obj-desig)
    (task-goal ?tsk (achieve (object-in-hand ?obj-desig ?_))))

  (<- (obj-desig-type ?desig ?type)
    (or (desig-prop ?desig (type ?type))
        (desig-prop ?desig (perception-pm::shape-model ?type))
        (desig-prop ?desig (perception-pm::segment-prototype ?type))))

  (<- (put-down-location ?loc-desig)
    ;; The location the robot stands at while trying to put down an
    ;; object
    (task ?tsk)
    (task-goal ?tsk (achieve (arms-at ?action-desig)))
    (desig-prop ?action-desig (to put-down))
    (task-location-context ?tsk ?loc-desig))

  (<- (pick-up-metadata (?tsk ?type ?obj ?obj-loc ?pick-up-loc ?outcome))
    (task-goal ?tsk (achieve (object-in-hand ?_ ?_)))
    (task-goal ?reach-tsk (achieve (arms-at ?traj)))
    (desig-prop ?traj (to grasp))
    (desig-prop ?traj (obj ?obj))
    (obj-desig-type ?obj ?type)
    (lisp-fun obj-desig-locaction ?obj ?obj-loc)
    (task-location-context ?reach-tsk ?pick-up-loc)
    (task-outcome ?reach-tsk ?outcome))

  (<- (pick-up-loc (?_ ?_ ?_ ?loc ?_) ?loc))
  
  (<- (put-down-metadata (?tsk ?type ?obj ?destination ?put-down-loc ?outcome))
    (task ?tsk)
    (task-goal ?tsk (achieve (object-placed-at ?_ ?dest)))
    (task-goal ?put-down-tsk (achieve (arms-at ?traj)))
    (desig-prop ?traj (to put-down))
    (desig-prop ?traj (obj ?obj))
    (obj-desig-type ?obj ?type)
    (lisp-fun current-desig ?dest ?destination)
    (task-outcome ?put-down-tsk ?outcome)
    (task-location-context ?put-down-tsk ?put-down-loc))

  (<- (perceive-metadata (?tsk ?type ?obj ?obj-loc ?robot-loc ?outcome))
    (task ?tsk)
    (task-goal ?tsk (perceive ?obj))
    (obj-desig-type ?obj ?type)
    (desig-prop ?obj (at ?obj-loc))
    (task-goal ?at-loc-tsk (at-location (?robot-loc)))
    (subtask+ ?tsk ?at-loc-tsk)
    (lisp-fun reference ?robot-loc ?x)
    (task-outcome ?tsk ?outcome)))

(defun enable-episode-knowledge (filename)
  (setf cet:*episode-knowledge*
        (cet:load-episode-knowledge filename)))

(def-fact-group execution-trace-interface ()
  (<- (with-execution-trace ?filename)
    (lisp-fun enable-episode-knowledge ?filename ?_)))

(def-fact-group pick-and-place-visualization ()
  (<- (clear-markers)
    (lisp-fun clear-markers ?_))
  
  (<- (visualize-robot-pose ?pose ?outcome)
    (lisp-fun visualize-robot-pose ?pose ?outcome ?_))

  ;; Visualizes the trajectory of the tf frame ?frame during ?tsk
  (<- (visualize-trajectory ?tsk ?frame)
    (bound ?frame)
    (task-started-at ?tsk ?st)
    (task-ended-at ?tsk ?et)
    (task-outcome ?tsk ?outcome)
    (lisp-fun visualize-tf-trajectory ?frame ?st ?et ?outcome ?_))

  (<- (visualize-obj ?obj-desig)
    (obj-desig-type ?obj-desig ?type)
    (lisp-fun obj-desig-location ?obj-desig ?loc)
    (format "asdf ~a ~a~%" ?obj-desig ?type)
    (lisp-fun visualize-object ?type ?loc ?_))

  (<- (visualize-tsk-location ?goal ?tsk ?outcome)
    (task ?tsk)
    (task-goal ?tsk ?goal)
    (task-outcome ?tsk ?outcome)
    (once (and (subtask+ ?tsk ?at-loc-tsk)
               (task-goal ?at-loc-tsk (at-location (?robot-loc)))))
    (lisp-fun loc-desig-location ?robot-loc ?pose)
    (visualize-robot-pose ?pose ?outcome))

  (<- (visualize-arm-trajectory ?action ?tsk ?outcome)
    (task ?tsk)
    (task-goal ?tsk (achieve (arms-at ?traj)))
    (task-outcome ?tsk ?outcome)
    (desig-prop ?traj (to ?action))
    (desig-prop ?traj (side ?side))
    (lisp-fun symbol-name ?side ?side-str)
    (lisp-fun string-downcase ?side-str ?side-lc-str)
    (string-concat ?side-lc-str "_arm_hand_link" ?frame)
    (visualize-trajectory ?tsk ?frame))

  (<- (visualize-arm-trajectory ?goal ?tsk ?outcome)
    (task ?tsk)
    (task-goal ?tsk ?goal)
    (once (and (subtask+ ?tsk ?manip-tsk)
               (task-goal ?manip-tsk (achieve (arms-at ?traj)))))
    (task-outcome ?tsk ?outcome)
    (desig-prop ?traj (side ?side))
    (lisp-fun symbol-name ?side ?side-str)
    (lisp-fun string-downcase ?side-str ?side-lc-str)
    (string-concat ?side-lc-str "_arm_hand_link" ?frame)
    (visualize-trajectory ?tsk ?frame))

  (<- (visualize-robot-trajectory ?action ?tsk ?outcome)
    (task ?tsk)
    (task-goal ?tsk (achieve (arms-at ?traj)))
    (task-outcome ?tsk ?outcome)
    (desig-prop ?traj (to ?action))
    (visualize-trajectory ?tsk "base_footprint"))

  (<- (visualize-robot-trajectory ?goal ?tsk ?outcome)
    (task ?tsk)
    (task-goal ?tsk ?goal)
    (task-outcome ?tsk ?outcome)
    (visualize-trajectory ?tsk "base_footprint"))

  (<- (visualize-object-detection ?tsk ?type)
    (task ?tsk)
    (task-goal ?tsk (perceive ?obj))
    (task-outcome ?tsk :succeeded)
    (task-result ?tsk ?o)
    (obj-desig-type ?o ?type)
    (visualize-obj ?o)))

(defun pose-stamped->msg (ps)
  (roslisp:make-message "geometry_msgs/PoseStamped"
                        (frame_id header) (cl-tf:frame-id ps)
                        (stamp header) (cl-tf:stamp ps)
                        (x position pose) (cl-transforms:x (cl-transforms:origin ps))
                        (y position pose) (cl-transforms:y (cl-transforms:origin ps))
                        (z position pose) (cl-transforms:z (cl-transforms:origin ps))
                        (x orientation pose) (cl-transforms:x (cl-transforms:orientation ps))
                        (y orientation pose) (cl-transforms:y (cl-transforms:orientation ps))
                        (z orientation pose) (cl-transforms:z (cl-transforms:orientation ps))
                        (w orientation pose) (cl-transforms:w (cl-transforms:orientation ps))))

(roslisp:def-service-callback rosie_executive-srv:QueryManipulationInfo (execution_trace_filename)
  (handler-case
      (flet ((make-plan-info (bdgs)
               (map 'vector (lambda (bdg)
                              (with-vars-bound
                                  (?obj-type ?obj-loc ?robot-loc ?outcome)
                                  bdg
                                (when (and ?obj-type ?obj-loc ?robot-loc ?outcome)
                                  (roslisp:make-message "rosie_executive/PlanInfo"
                                                        obj_type (symbol-name ?obj-type)
                                                        obj_location (pose-stamped->msg (reference ?obj-loc))
                                                        robot_location (pose-stamped->msg  (reference ?robot-loc))
                                                        status (symbol-name ?outcome)))))
                    bdgs)))
                  
        (cet:with-offline-episode-knowledge execution_trace_filename
          (roslisp:make-response :perceives (make-plan-info
                                             (force-ll
                                              (prolog `(perceive-metadata (?_ ?obj-type ?_ ?obj-loc
                                                                              ?robot-loc ?outcome)))))
                                 :pick_ups (make-plan-info
                                            (force-ll
                                             (prolog `(pick-up-metadata (?_ ?obj-type ?_ ?obj-loc
                                                                            ?robot-loc ?outcome)))))
                                 :put_downs (make-plan-info
                                             (force-ll
                                              (prolog `(put-down-metadata (?_ ?obj-type ?_ ?obj-loc
                                                                              ?robot-loc ?outcome))))))))
    (error (e)
      (roslisp:ros-warn (query-manipulation-info rosie-executive) "Service callback failed: ~a" e)
      (roslisp:make-response))))

(roslisp:def-service-callback rosie_executive-srv:ListExecutionTraces ()
  (handler-case
      (roslisp:make-response
       :filenames (map 'vector #'namestring
                       (directory (merge-pathnames
                                   (make-pathname :type "ek" :name :wild)
                                   (make-pathname
                                    :directory (roslisp:get-param "~execution_trace_dir"))))))
    (error (e)
      (roslisp:ros-warn (list-execution-traces rosie-executive) "Service callback failed: ~a" e)
      (roslisp:make-response))))

(defun init-pick-and-place-info-srv ()
  (roslisp:register-service "~query_manipulation_info" rosie_executive-srv:QueryManipulationInfo)
  (roslisp:register-service "~list_exectuon_traces" rosie_executive-srv:ListExecutionTraces))

(cram-roslisp-common:register-ros-init-function init-pick-and-place-info-srv)
