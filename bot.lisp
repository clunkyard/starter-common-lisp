;;;; A robot on the core binding supplied by the arena.
;;;; The core owns tracking, geometry, parsing and act serialization.
;;;; Clunkyard robot API 2.0.0: one core under every language, the same
;;;; answers everywhere. Facts (a call shows its arguments):
;;;; match: mode ruleset control robots is_3d tick_rate max_ticks
;;;;   robot_id(slot)
;;;; clock: tick seconds remaining_ticks
;;;; arena: size gravity terrain_kind ground_height(x y)
;;;; me: health position velocity heading pitch roll angular_velocity
;;;;   attitude grounded altitude applied
;;;; battery: charge capacity regen sustainable_drive compute
;;;;   drive_cost(fraction)
;;;; drive: hover_lift
;;;; gun: aim_now cooldown_ticks spread max_power damage_scale
;;;;   can_fire(power reserve) shot_speed(power) shot_cost(power)
;;;;   reload_ticks(power) shot_damage(power) power_for_speed(speed)
;;;;   power_for_flight_time(range seconds) lead(contact power)
;;;;   lead_point(position velocity power)
;;;; scanner: last proximity_range reach(arc) arc_for_reach(range)
;;;; sensors: contacts nearest cues contact(slot) position_of(contact)
;;;; tracker: cue nearest_bearing_rate velocity(slot) position(slot)
;;;;   last_seen(slot)
;;;; nav: zone zone.margin(point) zone.contains(point margin)
;;;;   zone.distance_from(point) storm storm.margin(point)
;;;;   storm.contains(point margin) storm.distance_from(point) hill
;;;;   hill.margin(point) hill.contains(point margin)
;;;;   hill.distance_from(point) pickups nearest_pickup(kind)
;;;;   field_distance(a b)
;;;; score: me best armour
;;;; angles: normalize(degrees) angle_diff(target current)
;;;;   bearing_to(from to) elevation_to(from to) direction(heading pitch)
;;;;   distance(a b) clamp(value low high)
;;;; raw: number(path) text(path) has(path) flag(path) count(path) line()
;;;; Controls: drive.set(throttle turn pitch_turn)
;;;;   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
;;;;   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
;;;;   tracker.configure(smoothing forget_ticks cue_ticks)
;;;;   status.set(text value) raw.extra(json)
;;;; Every control argument is optional; a second call keeps what the first
;;;; set. Absent facts are your language's none. Names are snake_case here;
;;;; your language cases them its own way. Angles are degrees from the +x
;;;; axis; pitch is positive up.
;;;; Manual: https://clunkyard.com/manual#sdk
(defpackage :bot (:use :cl :clunkyard))
(in-package :bot)

(defun steer (towards current) (angles-clamp (/ (angles-angle-diff towards current) 45d0)))

(defun calibration-work (work tick)
  "The calibration kernel, xorshift-sqrt v1: WORK rounds of a 64-bit xorshift
and a square-root sum."
  (declare (type fixnum work tick))
  (let ((x (logxor 88172645463325252 tick)) (sum 0d0))
    (declare (type (unsigned-byte 64) x) (type double-float sum))
    (dotimes (i work)
      (setf x (logxor x (ldb (byte 64 0) (ash x 13)))
            x (logxor x (ash x -7))
            x (logxor x (ldb (byte 64 0) (ash x 17))))
      (incf sum (sqrt (coerce (ash x -11) 'double-float))))
    (mod (+ (floor sum) (ash x -40)) 1000000)))


(defun starter-turn (bot)
  (let* ((position (me-position bot)) (heading (me-heading bot)) (pitch (me-pitch bot))
         (target (sensors-nearest bot)) (heard (tracker-cue bot))
         (free (and (match-is-3d bot) (zerop (vec3-z (arena-gravity bot))))))
    (drive-forces bot :brake 0.2d0)
    (cond
      (target
       (let* ((range (contact-range target)) (bearing (contact-bearing target)) (elevation (contact-elevation target))
              (throttle (cond ((> range 350) 0.7d0) ((< range 200) -0.4d0) (t 0.2d0)))
              (power (gun-power-for-flight-time bot range 1d0)) (aim (gun-lead bot target power))
              (aim-heading (if aim (solution-heading aim) bearing)) (aim-pitch (if aim (solution-pitch aim) elevation))
              (now (gun-aim-now bot)))
         (drive-set bot :throttle throttle :turn (steer bearing heading) :pitch-turn (steer elevation pitch))
         (scanner-set bot :heading bearing :arc 20d0 :pitch elevation)
         (gun-aim bot :heading aim-heading :pitch aim-pitch)
         (when (and (< (abs (angles-angle-diff aim-heading (direction-heading now))) 3)
                    (< (abs (- aim-pitch (direction-pitch now))) 3) (gun-can-fire bot power 10d0))
           (gun-fire bot :power power))
         (status-set bot :text "TRACK" :value range)))
      (heard
       (let* ((bearing (angles-bearing-to position (cue-position heard))) (distance (angles-distance position (cue-position heard)))
              (elevation (if free (angles-elevation-to position (cue-position heard)) 0d0))
              (error (min (cue-bearing-error heard) 30d0)) (tick (clock-tick bot))
              (across (* (/ (- (mod tick 5) 2) 2d0) error)) (up (if free (* (/ (- (mod (floor tick 5) 3) 1) 2d0) error) 0d0)))
         (drive-set bot :throttle 0.7d0 :turn (steer bearing heading) :pitch-turn (steer elevation pitch))
         (scanner-set bot :heading (+ bearing across) :arc (scanner-arc-for-reach bot (+ distance 100)) :pitch (+ elevation up))
         (status-set bot :text "CUE" :value distance)))
      (t
       (drive-set bot :throttle 0.5d0 :turn 0.33d0 :pitch-turn (steer 0d0 pitch))
       (scanner-set bot :heading (angles-normalize (+ (scan-heading (scanner-last bot)) 36)) :arc 40d0
                    :pitch (if free (* (- (mod (floor (clock-tick bot) 10) 3) 1) 35d0) 0d0))
       (status-set bot :text "SEARCH")))
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
    ;; The arena's rate calibration only; safe to delete.
    (let ((work (raw-number bot "state.calibration.work")))
      (when (and work (plusp work)) (status-set bot :text "WORK" :value (calibration-work (truncate work) (clock-tick bot)))))))

(run (make-robot :on-turn #'starter-turn))
