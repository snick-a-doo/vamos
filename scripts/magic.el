;;  magic.el - a utility for plotting friction functions.
;;
;;  Copyright (C) 2003 Sam Varner
;;
;;  This file is part of Vamos Automotive Simulator.
;;
;;  This program is free software; you can redistribute it and/or modify
;;  it under the terms of the GNU General Public License as published by
;;  the Free Software Foundation; either version 2 of the License, or
;;  (at your option) any later version.
;;
;;  This program is distributed in the hope that it will be useful,
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;  GNU General Public License for more details.
;;
;;  You should have received a copy of the GNU General Public License
;;  along with this program; if not, write to the Free Software
;;  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

;; The function `magic' is for use with car definition files used by
;; Vamos Automotive Simulator <http://vamos.sourceforge.net>.  The
;; Gnuplot plotting program is required.  
;;
;; Open a file and place the point inside a friction parameter list
;; and call `magic'.  Press 'x' to dismiss the Gnuplot window.
;; `magic' uses the values currently in the buffer; you don't need to
;; save the file between edits.  Gnuplot is run as a synchonus
;; subprocess.  So you need to dismiss the window before you can do
;; anything else in Emacs.

(defvar magic-camber 0.0) ;; Camber angle in degrees
(defvar magic-Fz 2.0) ;; Normal force in kN
(defvar magic-longitudinal-range '(-50 50))
(defvar magic-transverse-range '(-50 50))
(defvar magic-aligning-range '(-50 50))

(defun magic-num (index lst)
  "Return the INDEXth item in the list LST as a number."
  (string-to-number (nth index lst)))

(defun magic ()
  "Plot the friction function for the parameters at point."
  (interactive)
  (save-excursion
	;; Find the previous tag and remember its name.
	(re-search-backward "<\\(.*\\)>")
	(let ((type (buffer-substring (match-beginning 1) (match-end 1))))
	  ;; Find and split the coefficients.
	  (re-search-forward "\\[[ \t]*\\(.*\\)[ \t]*\\]")
	  (let* ((param (buffer-substring (match-beginning 1) (match-end 1)))
			 (equation "")
			 (coeff (split-string param "[ \t]*,[ \t]*"))
			 (file "magic-eq")) ; should be a temporary file
		;; Write the appropriate equation.
		(cond ((string= type "longitudinal")
			   (setq equation (magic-write-longitudinal coeff)))
			  ((string= type "transverse")
			   (setq equation (magic-write-transverse coeff)))
			  ((string= type "aligning")
			   (setq equation (magic-write-aligning coeff)))
			  (t (error "Point is not inside find friction parameter tag.")))
		
		(and (string= equation "")
			 (error "Could not set equation"))
		;; Write the equation to a file and call Gnuplot.
		(find-file file)
		(kill-region (point-min) (point-max))
		(insert equation)
		(save-buffer)
		(kill-buffer (current-buffer))
		(shell-command (concat "gnuplot -persist " file))))))


(defun magic-write-longitudinal (coeff)
  "Write the magic formula for longitudinal force."
  (let* ((Fz magic-Fz)
		 (Fz2 (* magic-Fz magic-Fz))
		 (C (magic-num 0 coeff))
		 (D (+ (* (magic-num 1 coeff) Fz2) (* (magic-num 2 coeff) Fz)))
		 (B (* (+ (* (magic-num 3 coeff) Fz2) (* (magic-num 4 coeff) Fz))
			   (exp (* -1 (magic-num 5 coeff) Fz))
			   (/ 1 (* C D))))
		 (E (+ (* (magic-num 6 coeff) Fz2) 
			   (* (magic-num 7 coeff) Fz) 
			   (magic-num 8 coeff)))
		 (Sh (+ (* (magic-num 9 coeff) Fz) (magic-num 10 coeff))))
	(format "plot [x=%f:%f] %f*sin (%f*atan(%f*(1 - %f)*(x + %f) + %f*atan(%f*(x + %f))))" 
			(nth 0 magic-longitudinal-range)
			(nth 1 magic-longitudinal-range)
			D C B E Sh E B Sh)))


(defun magic-write-transverse (coeff)
  "Write the magic formula for transverse force."
  (let* ((Fz magic-Fz)
		 (Fz2 (* magic-Fz magic-Fz))
		 (C (magic-num 0 coeff))
		 (D (+ (* (magic-num 1 coeff) Fz2) (* (magic-num 2 coeff) Fz)))
		 (B (* (magic-num 3 coeff) 
			   (sin (* 2 (atan (/ Fz (magic-num 4 coeff)))))
			   (- 1 (* (magic-num 5 coeff) (abs magic-camber)))
			   (/ 1 (* C D))))
		 (E (+ (* (magic-num 6 coeff) Fz) (magic-num 7 coeff)))
		 (Sh (+ (* (magic-num 8 coeff) magic-camber)
				(* (magic-num 9 coeff) Fz)
				(magic-num 10 coeff)))
		 (Sv (+ (* (+ (* (magic-num 11 coeff) Fz) (magic-num 12 coeff))
				   magic-camber Fz)
				(* (magic-num 13 coeff) Fz)
				(magic-num 14 coeff))))
	(format "plot [x=%f:%f] %f*sin(%f*atan(%f*(1 - %f)*(x + %f) + %f*atan(%f*(x + %f)))) + %f"
			(nth 0 magic-transverse-range)
			(nth 1 magic-transverse-range)
			D C B E Sh E B Sh Sv)))


(defun magic-write-aligning (coeff)
  "Write the magic formula for aligning torque."
  (let* ((Fz magic-Fz)
		 (Fz2 (* magic-Fz magic-Fz))
		 (C (magic-num 0 coeff))
		 (D (+ (* (magic-num 1 coeff) Fz2) (* (magic-num 2 coeff) Fz)))
		 (B (* (+ (* (magic-num 3 coeff) Fz2) (* (magic-num 4 coeff) Fz))
			   (- 1 (* (magic-num 6 coeff) (abs magic-camber)))
			   (exp (* -1 (magic-num 5 coeff) Fz))
			   (/ 1 (* C D))))
		 (E (* (+ (* (magic-num 7 coeff) Fz2) 
				  (* (magic-num 8 coeff) Fz) 
				  (magic-num 9 coeff))
			   (- 1 (* (magic-num 10 coeff) (abs magic-camber)))))
		 (Sh (+ (* (magic-num 11 coeff) magic-camber) 
				(* (magic-num 12 coeff) Fz)
				(magic-num 13 coeff)))
		 (Sv (+ (* (+ (* (magic-num 14 coeff) Fz2)
					  (* (magic-num 15 coeff) Fz))
				   magic-camber)
				(* (magic-num 16 coeff) Fz)
				(magic-num 17 coeff))))
	(format "plot [x=%f:%f] %f*sin(%f*atan(%f*(1 - %f)*(x + %f) + %f*atan(%f*(x + %f)))) + %f" 
			(nth 0 magic-aligning-range)
			(nth 1 magic-aligning-range)
			D C B E Sh E B Sh Sv)))

