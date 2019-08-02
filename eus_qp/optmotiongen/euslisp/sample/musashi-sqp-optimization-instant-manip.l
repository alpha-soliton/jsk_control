(load "package://tendon_tools/../model/musashiwholemodel/musashiwholemodel.l")
(require "package://musashi_utils/euslisp/musashi-robot-util")

(load "../instant-manipulation-configuration-task.l")
;;(load "./sample-sqp-optimization-instant.l")
(require "models/room610-chair-object.l")

(load "../instant-configuration-task.l")
(load "../sqp-optimization.l")
(load "../validate-jacobian.l")
(require "models/arrow-object.l")


(defun setup-for-sample-sqp-optimization-manip
    ()
  ;; assoc grasp coords
  (send *robot* :put :rarm-grasp-coords
        (make-cascoords :name :rarm-grasp-coords
                        :parent (send *robot* :link "RARM_HAND_LINK")
                        :coords (send (send *robot* :rarm-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :larm-grasp-coords
        (make-cascoords :name :larm-grasp-coords
                        :parent (send *robot* :link "LARM_HAND_LINK")
                        :coords (send (send *robot* :larm-end-coords) :copy-worldcoords)
                        ))

  ;; setup object
  (setq *obj* (instance room610-chair-object :init))
  (send (car (send *obj* :links)) :weight (* 10 1e3)) ;; 10 kg
  (send *obj* :put :convex-hull-body
        (null-output (convex-hull-3d (send (car (send *obj* :bodies)) :vertices))))
  (send (car (send *obj* :links)) :assoc (send *obj* :get :convex-hull-body))
  (send (send *obj* :handle-side-larm)
        :transform (make-coords :pos (float-vector -60 0 0) :rpy (list pi/2 -pi/2 0)))
  (send (send *obj* :handle-side-rarm)
        :transform (make-coords :pos (float-vector -60 0 0) :rpy (list -pi/2 -pi/2 0)))
  (send-all (send *obj* :bodies) :set-color (float-vector 0.8 0.4 0.2))
  (send *obj* :newcoords
        (make-coords :pos (float-vector 1000 0 0)))

  ;; assoc object coords
  (send *obj* :put :object-coords
        (make-cascoords :name :object-coords
                        :parent (car (send *obj* :links))
                        :coords (send *obj* :copy-worldcoords)
                        ))
  )


(defun setup-musashi-for-sample-sqp-optimization
    (&key (only-kinematics? nil))
  ;;(setq *robot* (instance hrp2jsknts-robot :init))
  (musashiwhole :verbose nil)
  ;; assoc contact coords
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "RLEG_FOOT_LINK")
                        :coords (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "LLEG_FOOT_LINK")
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
    ;;(create-support-polygon *robot* (send *robot* :hand :rarm :link "RARM_HAND_LINK"))
    ;;(create-support-polygon *robot* (send *robot* :hand :larm :link "LARM_HAND_LINK"))
    ;;(create-support-polygon *robot* (send *robot* :link "RLEG_FOOT_LINK"))
    ;;(create-support-polygon *robot* (send *robot* :link "LLEG_FOOT_LINK"))
    (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
    (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
    (create-support-polygon *robot* (send (send *robot* :rarm :end-coords) :parent))
    (create-support-polygon *robot* (send (send *robot* :larm :end-coords) :parent))
    )
  )

(defun sample-sqp-optimization-instant-manip
    (&key
     (optimize-torque? t)
     (avoid-collision? t)
     (fix-obj? nil)
     (fix-grasp? t)
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup environment
  (setq *ground-rleg* (make-cube 400 400 20))
  (send *ground-rleg* :set-color (float-vector 0.5 0.5 0.25))
  (send *ground-rleg* :newcoords
        (make-coords :pos (float-vector 0 -500 100) :rpy (list 0 0 (deg2rad -20))))
  (setq *arrow-lleg* (instance arrow-object :init))
  (send *arrow-lleg* :newcoords
        (make-coords :pos (float-vector 0 100 0) :rpy (list 0 0 0)))
  (setq *arrow-obj* (instance arrow-object :init))
  (send *arrow-obj* :newcoords
        (make-coords :pos (float-vector 400 0 100) :rpy (list (deg2rad 10) (deg2rad 20) (deg2rad 5))))
  (setq *env-list* (list *ground-rleg* *arrow-lleg* *arrow-obj*))
  (send-all *env-list* :worldcoords)
  ;; setup robot
  ;;(setup-hrp2jsknts-for-sample-sqp-optimization)
  (setup-musashi-for-sample-sqp-optimization)
  (setup-for-sample-sqp-optimization-manip)
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
                   (cond (fix-grasp?
                          (remove
                           nil
                           (list (send *robot-obj-env* :contact "rleg" :contact-coords)
                                 (send *arrow-lleg* :copy-worldcoords)
                                 (send *robot-obj-env* :object :handle-side-rarm)
                                 (send *robot-obj-env* :object :handle-side-larm)
                                 (if fix-obj? (send *arrow-obj* :copy-worldcoords))
                                 ))
                          )
                         (t
                          ;; todo
                          ))
                   :kin-attention-coords-list
                   (remove
                    nil
                    (list (send *robot-obj-env* :robot :get :rleg-contact-coords)
                          (send *robot-obj-env* :robot :get :lleg-contact-coords)
                          (send *robot-obj-env* :robot :get :rarm-grasp-coords)
                          (send *robot-obj-env* :robot :get :larm-grasp-coords)
                          (if fix-obj? (send *robot-obj-env* :object :get :object-coords))
                          ))
                   :contact-target-coords-list
                   (cond (fix-grasp?
                          (list (send *robot-obj-env* :contact "rleg" :contact-coords)
                                (send *arrow-lleg* :copy-worldcoords)
                                (send *robot-obj-env* :object :handle-side-rarm)
                                (send *robot-obj-env* :object :handle-side-larm)
                                )
                          )
                         (t
                          ;; todo
                          ))
                   :contact-attention-coords-list
                   (list (send *robot-obj-env* :robot :get :rleg-contact-coords)
                         (send *robot-obj-env* :robot :get :lleg-contact-coords)
                         (send *robot-obj-env* :robot :get :rarm-grasp-coords)
                         (send *robot-obj-env* :robot :get :larm-grasp-coords)
                         )
                   :contact-constraint-list
                   (list
                    (instance* default-contact-constraint
                               :init
                               :mu-trans 0.2
                               :mu-rot 0.01
                               :max-fz 400
                               (let* ((poly
                                       (send *robot-obj-env* :robot :get :rleg_foot_link-support-polygon))
                                      (vs
                                       (mapcar #'(lambda (v)
                                                   (send (send (send *robot-obj-env* :robot :get :rleg-contact-coords) :worldcoords)
                                                         :inverse-transform-vector v))
                                               (send poly :vertices)
                                               ))
                                      )
                                 (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                       :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                       :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                       :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                       ))
                               )
                    (instance* default-contact-constraint
                               :init
                               :mu-trans 0.2
                               :mu-rot 0.01
                               :max-fz 400
                               (let* ((poly
                                       (send *robot-obj-env* :robot :get :lleg_foot_link-support-polygon))
                                      (vs
                                       (mapcar #'(lambda (v)
                                                   (send (send (send *robot-obj-env* :robot :get :lleg-contact-coords) :worldcoords)
                                                         :inverse-transform-vector v))
                                               (send poly :vertices)
                                               ))
                                      )
                                 (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                       :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                       :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                       :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                       ))
                               )
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 100 100 100))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 100 100 100))
                    )
                   :contact-target-coords-obj-list
                   (cond (fix-grasp?
                          (list (send *robot-obj-env* :object :handle-side-rarm)
                                (send *robot-obj-env* :object :handle-side-larm)
                                )
                          )
                         (t
                          ;; todo
                          ))
                   :contact-constraint-obj-list
                   (list
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 100 100 100))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 100 100 100))
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
                   :collision-distance-margin 0.1
                   :delta-linear-joint 100 ;; [mm]
                   :delta-rotational-joint 30 ;; [deg]
                   (if optimize-torque?
                       (list
                        :norm-regular-scale-max 5e-5
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
                        (validate-element-jacobian-manipulation :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  )
(warn "(sample-sqp-optimization-instant-manip)~%")
