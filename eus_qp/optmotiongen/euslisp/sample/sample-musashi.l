(load "package://tendon_tools/../model/musashiwholemodel/musashiwholemodel.l")
(require "package://musashi_utils/euslisp/musashi-robot-util")

(load "../bspline-configuration-task.l")
(load "../instant-manipulation-configuration-task.l")
(load "../sqp-optimization.l")
(load "../bspline-dynamic-configuration-task.l")
(require "irteus/demo/sample-arm-model.l")
(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")
(require "models/room610-chair-object.l")

(defun setup-musashi-for-sqp-optimization
     (&key (only-kinematics? nil)) 
  (musashiwhole :verbose nil)
  (send *robot* :put :rleg-contact-coords 
        (make-cascoords :name :rleg-contact-coords 
                        :parent (send *robot* :link "RLEG_FOOT_LINK") 
                        :coords (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "RLEG_FOOT_LINK")
                        :coords (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "RARM_HAND_LINK")
                        :coords (send (send (send *robot* :link "RARM_HAND_LINK") :copy-worldcoords)
                                      :translate (scale 1e3 (float-vector -0.0042 -0.0233 -0.2345)))
                        ))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "LARM_HAND_LINK")
                        :coords (send (send (send *robot* :link "LARM_HAND_LINK") :copy-worldcoords)
                                      :translate (scale 1e3 (float-vector -0.0042 0.0233 -0.2345)))
                        ))
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords)) 
  (unless only-kinematics?
    ;; create support polygon
    (create-support-polygon *robot* (send (send *robot* :larm :end-coords) :parent))
    (create-support-polygon *robot* (send (send *robot* :rarm :end-coords) :parent))
    (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
    (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
    ) 
  )
(warn "(setup-musashi-for-sqp-optimization)~%")

(defun sample-robot-sqp-instant-manip-config-task
    (&key
     (optimize-torque? t)
     (avoid-collision? t)
     (fix-obj? nil)
     (loop-num 50)
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup robot
  ;;(setq *robot* (instance IRT-musashiwhole-robot :init))
  (musashiwhole :verbose nil)
  (send *robot* :put :rleg-contact-coords 
        (make-cascoords :name :rleg-contact-coords 
                        :parent (send *robot* :link "RLEG_FOOT_LINK") 
                        :coords (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "RLEG_FOOT_LINK")
                        :coords (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "RARM_HAND_LINK")
                        :coords (send (send (send *robot* :link "RARM_HAND_LINK") :copy-worldcoords)
                                      :translate (scale 1e3 (float-vector -0.0042 -0.0233 -0.2345)))
                        ))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "LARM_HAND_LINK")
                        :coords (send (send (send *robot* :link "LARM_HAND_LINK") :copy-worldcoords)
                                      :translate (scale 1e3 (float-vector -0.0042 0.0233 -0.2345)))
                        ))
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  ;; setup object
  (setq *obj* (instance room610-chair-object :init))
  (send (car (send *obj* :links)) :weight (* 1 1e3)) ;; 1 kg
  (send-all (send *obj* :bodies) :set-color (float-vector 0.8 0.4 0.2))
  (send *obj* :put :object-coords
        (make-cascoords :parent (car (send *obj* :links)) :coords (send *obj* :copy-worldcoords)))
  (send (send *obj* :handle-side-larm)
        :transform (make-coords :pos (float-vector -60 0 0) :rpy (list pi/2 -pi/2 0)))
  (send (send *obj* :handle-side-rarm)
        :transform (make-coords :pos (float-vector -60 0 0) :rpy (list -pi/2 -pi/2 0)))
  (send *obj* :put :convex-hull-body
        (null-output (convex-hull-3d (send (car (send *obj* :bodies)) :vertices))))
  (send (car (send *obj* :links)) :assoc (send *obj* :get :convex-hull-body))
  (send *obj* :newcoords
        (make-coords :pos (float-vector 1000 0 0)))
  ;; setup environment
  (setq *ground-rleg* (make-cube 300 300 20))
  (send *ground-rleg* :set-color (float-vector 0.5 0.5 0.25))
  (send *ground-rleg* :newcoords
        (make-coords :pos (float-vector 0 -200 200) :rpy (list 0 0 (deg2rad -20))))
  (setq *arrow-lleg* (instance arrow-object :init))
  (send *arrow-lleg* :newcoords
        (make-coords :pos (float-vector 0 100 0) :rpy (list 0 0 0)))
  (when fix-obj?
    (setq *arrow-obj* (instance arrow-object :init))
    (send *arrow-obj* :newcoords
          (make-coords :pos (float-vector 600 0 100)))
    )
  (setq *env-list* (remove nil (list *ground-rleg* *arrow-lleg* (if fix-obj? *arrow-obj*))))
  (send-all *env-list* :worldcoords)
  
  ;; (setup-musashi-for-sample-sqp-optimization)
  ;; (setup-for-sample-sqp-optimization-manip)
  
  ;; setup contact
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg")))
  (setq *robot-obj-env*
        (instance robot-object-environment :init :robot *robot* :object *obj* :contact-list *contact-list*))
  (send *robot-obj-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
  (send *robot-obj-env* :contact "rleg" :rotate pi :z :local)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *obj*) *env-list*))

  (setq *instant-manip-config-task*
        (instance* instant-manipulation-configuration-task :init
                   :robot-env *robot-obj-env*
                   :drive-joint-list
                   (if optimize-torque? (send *robot-obj-env* :drive-joint-list) nil)
                   :posture-joint-list
                   (send *robot-obj-env* :robot :joint-list)
                   :posture-joint-angle-list
                   (send-all (send *robot-obj-env* :robot :joint-list) :joint-angle)
                   :kin-target-coords-list
                   (remove
                    nil
                    (list (send *robot-obj-env* :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send *robot-obj-env* :object :handle-side-rarm)
                          (send *robot-obj-env* :object :handle-side-larm)
                          (if fix-obj? (send *arrow-obj* :copy-worldcoords))
                          ))
                   :kin-attention-coords-list
                   (remove
                    nil
                    (list (send *robot-obj-env* :robot :rleg-end-coords)
                          (send *robot-obj-env* :robot :lleg-end-coords)
                          (send *robot-obj-env* :robot :rarm-end-coords)
                          (send *robot-obj-env* :robot :larm-end-coords)
                          (if fix-obj? (send *robot-obj-env* :object :get :object-coords))
                          ))
                   :contact-target-coords-list
                   (list (send *robot-obj-env* :contact "rleg" :contact-coords)
                         (send *arrow-lleg* :copy-worldcoords)
                         (send *robot-obj-env* :object :handle-side-rarm)
                         (send *robot-obj-env* :object :handle-side-larm)
                         )
                   :contact-attention-coords-list
                   (list (send *robot-obj-env* :robot :rleg-end-coords)
                         (send *robot-obj-env* :robot :lleg-end-coords)
                         (send *robot-obj-env* :robot :rarm-end-coords)
                         (send *robot-obj-env* :robot :larm-end-coords)
                         )
                   :contact-constraint-list
                   (list
                    (instance default-contact-constraint
                              :init-from-support-polygon
                              :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                              :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                              )
                    (instance default-contact-constraint
                              :init-from-support-polygon
                              :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                              :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                              )
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    )
                   :contact-target-coords-obj-list
                   (list (send *robot-obj-env* :object :handle-side-rarm)
                         (send *robot-obj-env* :object :handle-side-larm)
                         )
                   :contact-constraint-obj-list
                   (list
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    )
                   :act-react-pair-list
                   (list
                    (list (send *robot-obj-env* :object :handle-side-rarm) (send *robot-obj-env* :object :handle-side-rarm))
                    (list (send *robot-obj-env* :object :handle-side-larm) (send *robot-obj-env* :object :handle-side-larm))
                    )
                   :collision-pair-list
                   (if avoid-collision?
                       (mapcar
                        #'(lambda (l)
                            (list l (send *robot-obj-env* :object :get :convex-hull-body)))
                        (flatten
                         (mapcar #'(lambda (limb) (send *robot-obj-env* :robot limb :links))
                                 (list :torso :rleg :lleg))
                         )))
                   :collision-distance-margin 0.05
                   :delta-linear-joint 100 ;; [mm]
                   :delta-rotational-joint 30 ;; [deg]
                   (if optimize-torque?
                       (list
                        :norm-regular-scale-max 1e-6
                        ))
                   ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-manip-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-element-jacobian-manipulation :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num loop-num)
  )
(warn "(sample-robot-sqp-instant-manip-config-task)~%")
