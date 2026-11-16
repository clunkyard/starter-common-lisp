;;;; A robot on the core binding supplied by the arena.
;;;; The core owns tracking, geometry, parsing and act serialization.
;;;; Bearing is earned: a scan contact's bearing is off by up to its
;;;; bearing-error (degrees, a fixed share of the arc), so a wide beam finds a
;;;; robot and a narrow one aims at it. The robot narrows the beam, holds it
;;;; on the target and aims along a line fitted through its recent sightings.
;;;; Contacts within 120 units are exact.
;;;; Clunkyard robot API 2.0.0: one core under every language, the same
;;;; answers everywhere. Facts (a call shows its arguments):
;;;; match: mode ruleset control robots is_3d tick_rate max_ticks
;;;;   robot_id(slot)
;;;; clock: tick seconds remaining_ticks
;;;; arena: size gravity terrain_kind ground_height(x y)
;;;; me: health position velocity heading pitch roll angular_velocity
;;;;   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
;;;;   from_mount(heading pitch) mount_toward(point)
;;;; battery: charge capacity regen sustainable_drive compute
;;;;   drive_cost(fraction)
;;;; drive: hover_lift
;;;; gun: aim_now cooldown_ticks spread max_power damage_scale
;;;;   can_fire(power reserve) shot_speed(power) shot_cost(power)
;;;;   reload_ticks(power) shot_damage(power) power_for_speed(speed)
;;;;   power_for_flight_time(range seconds) lead(contact power)
;;;;   lead_point(position velocity power)
;;;; scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
;;;; sensors: contacts nearest cues contact(slot) position_of(contact)
;;;; tracker: cue nearest_bearing_rate velocity(slot) position(slot)
;;;;   history(slot) last_seen(slot)
;;;; nav: zone zone.margin(point) zone.contains(point margin)
;;;;   zone.distance_from(point) storm storm.margin(point)
;;;;   storm.contains(point margin) storm.distance_from(point) hill
;;;;   hill.margin(point) hill.contains(point margin)
;;;;   hill.distance_from(point) pickups nearest_pickup(kind)
;;;;   field_distance(a b)
;;;; score: me best armour
;;;; angles: normalize(degrees) angle_diff(target current)
;;;;   bearing_to(from to) elevation_to(from to) direction(heading pitch)
;;;;   distance(a b) to_chassis(heading pitch attitude)
;;;;   from_chassis(heading pitch attitude) clamp(value low high)
;;;; raw: number(path) text(path) has(path) flag(path) count(path) line()
;;;; Controls: drive.set(throttle turn pitch_turn)
;;;;   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
;;;;   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
;;;;   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
;;;;   status.set(text value) debug.log(line) raw.extra(json)
;;;; Every control argument is optional; a second call keeps what the first
;;;; set. Absent facts are your language's none. Names are snake_case here;
;;;; your language cases them its own way. Angles are degrees from the +x
;;;; axis; pitch is positive up.
;;;; Manual: https://clunkyard.com/manual#sdk
(defpackage :bot (:use :cl :clunkyard))
(in-package :bot)

(defun steer (towards current) (angles-clamp (/ (angles-angle-diff towards current) 45d0)))

(defparameter *search-arc* 40d0) ; degrees: sweeps the circle in half a second
(defparameter *hold-width* 60d0) ; units across at the target: it stays in the beam from tick to tick
(defparameter *min-arc* 6d0)     ; degrees: the narrowest the beam holds
(defparameter *window* 12)       ; ticks of sightings the aim averages
(defparameter *sightings* 8)     ; narrow sightings in the window before a shot
(defparameter *look-every* 40)   ; ticks between looks around the target, for a nearer robot
(defparameter *reacquire* 10)    ; ticks to look again where a lost target was

;; Dodges taken, and the tick the current one ends.
(defvar *dodges* 0)
(defvar *dodge-end* 0)
;; The last target's slot, for looking again where it was lost.
(defvar *slot* 0)

;; A straight line through the narrowest of the last *window* ticks' sightings:
;; where the robot is now and how fast it moves, and how many sightings it rests
;; on. Each sighting is off by its bearing-error; the line averages that out.
(defun fit (bot slot tick)
  (let* ((recent (remove-if-not (lambda (s) (> (sighting-tick s) (- tick *window*))) (coerce (tracker-history bot slot) 'list)))
         (best (when recent (reduce #'min recent :key #'sighting-bearing-error)))
         (seen (when best (remove-if-not (lambda (s) (<= (sighting-bearing-error s) (* 1.5d0 best))) recent))) ; the narrow ones
         (n (length seen)))
    (when (>= n 2)
      (let* ((t0 (/ (reduce #'+ seen :key #'sighting-tick) (float n 1d0)))
             (spread (loop for s in seen sum (expt (- (sighting-tick s) t0) 2)))
             ;; One axis: the line's position now and its slope per tick.
             (axis (lambda (along)
                     (let* ((mean (/ (loop for s in seen sum (funcall along (sighting-position s))) n))
                            (slope (/ (loop for s in seen sum (* (- (sighting-tick s) t0) (- (funcall along (sighting-position s)) mean))) spread)))
                       (cons (+ mean (* slope (- tick t0))) (* slope (match-tick-rate bot)))))))
        (let ((x (funcall axis #'vec3-x)) (y (funcall axis #'vec3-y)) (z (funcall axis #'vec3-z)))
          (values (make-vec3 :x (car x) :y (car y) :z (car z))
                  (make-vec3 :x (cdr x) :y (cdr y) :z (cdr z))
                  n))))))

(defun starter-turn (bot)
  (let* ((position (me-position bot)) (heading (me-heading bot)) (pitch (me-pitch bot))
         (tick (clock-tick bot)) (target (sensors-nearest bot)) (heard (tracker-cue bot))
         (last-seen (and (/= *slot* 0) (tracker-last-seen bot *slot*)))
         (free (and (match-is-3d bot) (zerop (vec3-z (arena-gravity bot))))))
    (drive-forces bot :brake 0.2d0)
    (cond
      (target
       (let* ((range (contact-range target)) (bearing (contact-bearing target)) (elevation (contact-elevation target))
              (throttle (cond ((> range 350) 0.7d0) ((< range 200) -0.4d0) (t 0.2d0)))
              ;; Halve the beam each tick until it is *hold-width* across at the target:
              ;; narrow enough for a precise bearing, wide enough to keep it in.
              (hold (max *min-arc* (* (/ 180 pi) 2 (atan (/ *hold-width* 2) range))))
              ;; In a crowd a narrow beam sees one robot. Every *look-every* ticks,
              ;; widen it for a tick: a nearer robot it finds becomes the target.
              (arc (if (zerop (mod tick *look-every*)) *search-arc* (max hold (min *search-arc* (/ (scan-arc (scanner-last bot)) 2)))))
              (power (gun-power-for-flight-time bot range 1d0))
              (noisy (> (contact-bearing-error target) 0)) (aim nil) (ready t))
         (setf *slot* (contact-slot target))
         ;; An exact contact: gun-lead is the intercept if the target holds course.
         ;; A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
         (if noisy
             (multiple-value-bind (where velocity count) (fit bot *slot* tick)
               (setf aim (and where (gun-lead-point bot where velocity power))
                     ready (and count (>= count *sightings*))))
             (setf aim (gun-lead bot target power)))
         (let ((aim-heading (if aim (solution-heading aim) bearing)) (aim-pitch (if aim (solution-pitch aim) elevation))
               (now (gun-aim-now bot)))
           (drive-set bot :throttle throttle :turn (steer bearing heading) :pitch-turn (steer elevation pitch))
           (scanner-set bot :heading bearing :arc arc :pitch elevation)
           (gun-aim bot :heading aim-heading :pitch aim-pitch)
           (when (and ready (< (abs (angles-angle-diff aim-heading (direction-heading now))) 3)
                      (< (abs (- aim-pitch (direction-pitch now))) 3) (gun-can-fire bot power 10d0))
             (gun-fire bot :power power))
           (status-set bot :text (if ready "TRACK" "NARROW") :value range))))
      ;; Lost it: look again where it was last seen, widening the beam, before searching.
      ((and last-seen (<= (- tick last-seen) *reacquire*))
       (let* ((where (or (tracker-position bot *slot*) position))
              (bearing (angles-bearing-to position where))
              (elevation (if free (angles-elevation-to position where) 0d0)))
         (drive-set bot :throttle 0.5d0 :turn (steer bearing heading) :pitch-turn (steer elevation pitch))
         (scanner-set bot :heading bearing :arc (min *search-arc* (* 2 (scan-arc (scanner-last bot)))) :pitch elevation)
         (status-set bot :text "REACQUIRE")))
      (heard
       (let* ((bearing (angles-bearing-to position (cue-position heard))) (distance (angles-distance position (cue-position heard)))
              (elevation (if free (angles-elevation-to position (cue-position heard)) 0d0))
              (error (min (cue-bearing-error heard) 30d0))
              (across (* (/ (- (mod tick 5) 2) 2d0) error)) (up (if free (* (/ (- (mod (floor tick 5) 3) 1) 2d0) error) 0d0)))
         (drive-set bot :throttle 0.7d0 :turn (steer bearing heading) :pitch-turn (steer elevation pitch))
         (scanner-set bot :heading (+ bearing across) :arc (scanner-arc-for-reach bot (+ distance 100)) :pitch (+ elevation up))
         (status-set bot :text "CUE" :value distance)))
      (t
       (drive-set bot :throttle 0.5d0 :turn 0.33d0 :pitch-turn (steer 0d0 pitch))
       (scanner-set bot :heading (angles-normalize (+ (scan-heading (scanner-last bot)) 36)) :arc *search-arc*
                    :pitch (if free (* (- (mod (floor tick 10) 3) 1) 35d0) 0d0))
       (status-set bot :text "SEARCH")))
    ;; A simple dodge: vary it, or predict the shooter.
    (progn
      (when (and (>= tick *dodge-end*) (find :muzzle (sensors-cues bot) :key #'contact-source))
        (incf *dodges*)
        (setf *dodge-end* (+ tick 10)))
      (when (< tick *dodge-end*)
        ;; Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
        (let ((left (oddp *dodges*)))
          (if (eq (match-control bot) :forces)
              (progn (drive-set bot :throttle 0.5d0) (drive-forces bot :strafe (if left -1d0 1d0)))
              (drive-set bot :throttle 1d0 :turn (if left 1d0 -1d0)))
          (status-set bot :text (if left "dodge:left" "dodge:right") :value *dodges*))))
    (when (< (battery-charge bot) (/ (battery-capacity bot) 4))
      (drive-set bot :throttle (* (or (drive-get bot :throttle) 0d0) 0.5d0)))
    (let ((storm (nav-storm bot)) (zone (nav-zone bot)))
      (when (and storm (> (or (nav-storm-margin storm position) -1d9) -60))
        (let ((away (angles-bearing-to (storm-center storm) position)))
          (drive-set bot :throttle 0.8d0 :turn (angles-clamp (/ (angles-angle-diff away heading) 30d0)))))
      (when (and zone (< (or (nav-zone-margin zone position) 1d9) 60))
        (let ((climb (if (eq (zone-shape zone) :sphere) (angles-elevation-to position (zone-center zone)) 0d0)))
          (drive-set bot :throttle 0.8d0 :turn (angles-clamp (/ (angles-angle-diff (angles-bearing-to position (zone-center zone)) heading) 30d0))
                     :pitch-turn (angles-clamp (/ (- climb pitch) 30d0))))))
    ))

(run (make-robot :on-turn #'starter-turn))
