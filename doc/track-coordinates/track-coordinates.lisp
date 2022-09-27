;;; PostStript Functions

(defmacro with-ps-file (stream file &body body)
  "Send PostStript commands to FILE and insert the obligatory lines.
The file is suitable for including in a LaTeX document, although
the bounding box might have to be adjusted."
  `(with-open-file (,stream ,file
                            :direction :output
                            :if-exists :supersede)
     (format stream "%!PS-Adobe-3.0 EPSF-3.0~%")
     (format stream "%%BoundingBox: 0 0 400 200~%")
     ,@body
     (format ,stream "~&showpage~%")))

(defun ps-line (stream xy1 xy2)
  "Print the PostStript code for drawing a line.
xy1 and xy2 are cons cells."
  (format stream "~&~F ~F moveto~%" (car xy1) (cdr xy1))
  (format stream "~F ~F lineto~%" (car xy2) (cdr xy2))
  (format stream "stroke~%"))

(defun ps-arc (stream xc yc radius angle arc)
  "Print the PostStript code for drawing an arc."
  (labels ((rad->deg (angle) 
			 (- (* angle (/ 180 pi))
				90)))
    (format stream "~F ~F ~F ~F ~F arc~%" 
            xc yc radius
            (rad->deg angle)
            (rad->deg (+ angle arc)))
    (format stream "stroke~%")))

;;; Equations

(defun straight (d q1 x0 y0 theta l sigma-start sigma-end)
  "Return the world coordinates for track coordinates D and Q1 on a
straight segment."
  (let ((d-prime (+ (* d (1+ (* q1 (/ (- sigma-end sigma-start) l))))
                    (* q1 sigma-start))))
    (cons (+ x0 
             (* d-prime (cos theta)) 
             (* (- q1) (sin theta)))
          (+ y0
             (* d-prime (sin theta))
             (* q1 (cos theta))))))

(defun r-prime (r q1 sigma beta)
  "Return the radius of a skewed curve."
  (- r (* q1 (/ sigma (tan beta)))))

(defun xc-prime (xc q1 sigma theta beta)
  "Return the x-coordinate for the center of a skewed curve."
  (+ xc (* q1 sigma (/ (sin (+ theta beta))
					   (sin beta)))))

(defun yc-prime (yc q1 sigma theta beta)
  "Return the y-coordinate for the center of a skewed curve."
  (- yc (* q1 sigma (/ (cos (+ theta beta))
                       (sin beta)))))

(defun curve (alpha q1 x0 y0 theta r arc sigma-start)
  "Return the world coordinates for track coordinates ALHPA and Q1 on
a cuved segment."
  (let* ((xc (- x0 (* r (sin theta))))
         (yc (+ y0 (* r (cos theta))))
         (beta (/ arc 2))
         (radius (- (r-prime r q1 sigma-start beta) q1)))
    (cons (+ (xc-prime xc q1 sigma-start theta beta)
             (* radius (sin (+ theta alpha))))
          (- (yc-prime yc q1 sigma-start theta beta) 
             (* radius (cos (+ theta alpha)))))))

(defun solve-quadratic (a b c)
  "Solve a quadratic (or linear) equation of the form 
Ax^2 + Bx + C = 0."
  (if (zerop a)
      (- (/ c b))
      (let ((discriminant (- (* b b) (* 4 a c))))
        (values (/ (+ (- b) (sqrt discriminant)) (* 2 a))
                (/ (- (- b) (sqrt discriminant)) (* 2 a))))))

(defun solve-for-q1 (x y radius arc skew)
  "Return the track coordinate Q1 for world coordinates (X, Y) on a
skewed curve."
  (let ((beta (/ arc 2)))
	(multiple-value-bind (q1+ q1-)
		(solve-quadratic
		 (+ (* skew skew) 
			(/ (* -2 skew) (tan beta)) 
			-1)
		 (* 2 (+ (* (/ skew (sin beta))
                    (- (* radius (cos beta)) x))
                 radius))
		 (+ (* x x) 
			(* y y)
            (- (* radius radius))))
	  (if (plusp radius) q1- q1+))))

(defun alpha (x xc y yc arc)
  "Return the angle traveled through the curve for world
coordinates (X, Y) on skewed curve."
    (+ (atan (* (- y yc) (signum arc))
             (* (- x xc) (signum arc)))
       (/ arc 2)))

;;; Drawing Functions

(defmacro do-range ((value start range steps) &body body)
  "Loop over a range."
  (let ((count (gensym))
        (increment (gensym))
        (g-steps (gensym)))
    `(let* ((,g-steps ,steps)
            (,increment (/ ,range (- ,g-steps 1))))
       (do ((,count 0 (+ ,count 1))
            (,value ,start (+ ,value ,increment)))
           ((= ,count ,g-steps))
         ,@body))))

(defmacro do-lines (((x next-x x-start x-range x-steps)
					 (y next-y y-start y-range y-steps))
					&body body)
  "Loop over a grid setting X, Y, NEXT-X, NEXT-Y to grid points.
Drawing lines between each (X, Y) and (NEXT-X, NEXT-Y) point results
in a set of lines of constant y-value in the x-dimension."
  (let ((x-count (gensym))
		(x-interval (gensym))
        (g-x-start (gensym))
        (g-x-steps (gensym))
		(y-count (gensym))
		(y-interval (gensym))
        (g-y-steps (gensym)))
	`(let ((,g-x-start ,x-start)
           (,g-x-steps ,x-steps)
           (,g-y-steps ,y-steps))
	  (do* ((,y-count 0 (+ ,y-count 1))
			(,y-interval (/ ,y-range ,g-y-steps))
			(,y ,y-start (+ ,y ,y-interval))
			(,next-y ,y ,y))
		   ((> ,y-count ,g-y-steps))
		(do* ((,x-count 0 (+ ,x-count 1))
			  (,x-interval (/ ,x-range ,g-x-steps))
			  (,x ,g-x-start ,next-x)
			  (,next-x (+ ,g-x-start ,x-interval) (+ ,next-x ,x-interval)))
			 ((= ,x-count ,g-x-steps))
		  ,@body)))))

(defmacro do-grid ((x-spec y-spec) &body body)
  "Draw a grid.  Each -SPEC is a list of the form (var next-var start
range steps).  Drawing a line between each (x-var, y-var) and
(x-next-var, y-next-var) results in a x-steps by y-steps grid."
  `(progn 
	(do-lines (,x-spec ,y-spec) ,@body)
	(do-lines (,y-spec ,x-spec) ,@body)))

(defun draw-straight (stream x0 y0 angle width length divisions
                             &key (start-skew 0.0) (end-skew 0.0))
  "Draw a straight segment with lines of constant d and q1."
  (do-grid ((d next-d 0 length divisions)
			(q1 next-q1 (/ width -2) width 2))
	(ps-line stream
			 (straight d q1 x0 y0 angle length start-skew end-skew)
			 (straight next-d next-q1 x0 y0 angle length start-skew end-skew))))

(defun draw-curve (stream x0 y0 start-angle width radius arc divisions
                          &key (skew 0.0))
  "Draw a curved segment with lines of constant d and q1."
  (do-grid ((angle next-angle 0 arc divisions)
			(q1 next-q1 (/ width -2) width 2))
	(ps-line stream
			 (curve angle q1 x0 y0 start-angle radius arc skew)
			 (curve next-angle next-q1 x0 y0 start-angle radius arc skew))))

(defun draw-arc (stream x0 y0 radius q1 angle arc skew 
				 &optional show-full-circle)
  "Draw an arc of a skewed curve.  If SHOW-FULL-CIRCLE is non-nil the
complete circle is drawn.  Note that ANGLE and ARC are needed for the
skew calculations even if the full circle is drawn."
  (ps-arc stream 
          (xc-prime x0 q1 skew angle (/ arc 2)) 
          (yc-prime y0 q1 skew angle (/ arc 2)) 
          (- (r-prime radius q1 skew (/ arc 2)) q1)
          angle 
		  (if show-full-circle 
              (* 2 pi)
              arc)))

(defun draw-corner (stream x0 y0 length width radius arc
                           &key (skew 0.0))
  "Draw a curve with preceeding and following straight segments."
  (let ((angle (/ (- pi arc) 2)))
    (draw-straight stream x0 y0 angle width length 1
                   :end-skew skew)
    (incf x0 (- (* length (cos angle))
                (* radius (sin angle))))
    (incf y0 (+ (* length (sin angle))
                (* radius (cos angle))))
	(do-range (q1 (/ width -2) width 3)
	  (draw-arc stream x0 y0 radius q1 angle arc skew))
	(incf angle arc)
	(incf x0 (* radius (sin angle)))
	(decf y0 (* radius (cos angle)))
	(draw-straight stream x0 y0 angle width length 1 
				   :start-skew (- skew))))

(defun draw-circles (stream x0 y0 skew)
  "Draw circles of constant q1 with coordinate axes."
  (let ((size 75)
        (radius 40)
		(width 40))
    (ps-line stream (cons (- x0 size) y0) (cons (+ x0 size) y0))
    (ps-line stream (cons x0 (- y0 size)) (cons x0 (+ y0 size)))
	(do-range (q1 (/ width -2) width 3)
      (draw-arc stream x0 y0 radius q1 (- (/ pi 2) 1) 2 skew 
				:circle))))

;;; Figures

;; Overlap
(with-ps-file stream "overlap.eps"
  (draw-corner stream 50 40 75 50 (/ 50 3.0) 2)
  (draw-corner stream 250 40 75 50 (/ 50 3.0) 2
                 :skew -0.7))

;; Straight
(with-ps-file stream "straight.eps"
  (draw-straight stream 30 50 0.6 50 200 10)
  (draw-straight stream 250 50 0.6 50 200 10 
                 :start-skew 0.4 :end-skew -0.8))

;; Curve
(with-ps-file stream "curve.eps"
  (draw-curve stream 30 50 -0.2 50 100 2 10)
  (draw-curve stream 250 50 -0.2 50 100 2 10
              :skew -0.5))

;; Circles
(with-ps-file stream "circles.eps"
  (draw-circles stream 30 250 0)
  (draw-circles stream 200 250 -0.3)
  (draw-circles stream 370 250 (/ pi -6))
  (draw-circles stream 30 50 -1)
  (draw-circles stream 200 50 (- (/ pi 2)))
  (draw-circles stream 370 50 0.5))


(defun solve-curve (x y radius theta arc skew)
  "Return the track coordinates for the given world coordinates."
  (let ((q1 (solve-for-q1 x y radius arc skew)))
	(values (alpha x (xc-prime 0 q1 skew theta (/ arc 2)) 
				   y (yc-prime 0 q1 skew theta (/ arc 2))
				   arc)
			q1)))

(defun solve-curve-points (width radius arc divisions
					&key (skew 0.0))
  "Verify the q1 and alpha solutions."
  (let ((beta (/ arc 2)))
	(do-lines ((angle next-angle 0 arc divisions)
			(q1 next-q1 (/ width -2) width 2))
	(let* ((theta (- (/ pi 2) beta))
		   (point (curve angle q1 
						 (* radius (cos beta))
						 (* -1 radius (sin beta))
						 theta radius arc skew)))
	  (multiple-value-bind 
			(alpha-out q1-out)
		  (solve-curve (car point) (cdr point) radius theta arc skew)
		(format t "(~3,2F, ~3,2F) (~3,2F, ~3,2F) (~3,2F, ~3,2F)~%" 
				angle q1
				(car point) (cdr point)
				alpha-out q1-out))))))
