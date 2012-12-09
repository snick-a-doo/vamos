(defun in->m (length) 
  "Convert inches to meters"
  (interactive (list (string-to-number 
                      (read-from-minibuffer "Length in inches: "))))
  (insert (number-to-string (* length 0.02540))))

(defun lb->kg (weight) 
  "Convert pounds to kilograms ignoring that pounds is a force
  unit and kilograms is a mass unit."
  (interactive (list (string-to-number 
                      (read-from-minibuffer "Weight in pounds: "))))
  (insert (number-to-string (* weight 0.4536))))

(defun hp->w (power)
  "Convert horsepower to watts"
  (interactive (list (string-to-number 
                      (read-from-minibuffer "Horsepower: "))))
  (insert (number-to-string (* power 745.7))))
  