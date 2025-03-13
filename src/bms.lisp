
(defun init-bms () {
    (var v-cell-support (eq (first (trap (get-bms-val 'bms-v-cell-min))) 'exit-ok))
    (var temp-max 0)
    (var temp-min 0)
    (var v-min 0)
    (var v-max 0)
    (var temp-fet -281)
    (var num-cells 0)
    (var cell-v 0.0)
    (loopwhile t {
        (if (and (>= (get-bms-val 'bms-can-id) 0) (ext-bms)) {
            (setq temp-max (get-bms-val 'bms-temp-cell-max))
            (setq temp-min temp-max)
            
            (if v-cell-support {
                (if (= (get-bms-val 'bms-data-version) 1) {
                    (setq temp-min (get-bms-val 'bms-temps-adc 1))
                    (setq temp-fet (get-bms-val 'bms-temps-adc 3))
                })
                (setq v-min (get-bms-val 'bms-v-cell-min))
                (setq v-max (get-bms-val 'bms-v-cell-max))
            } {
                (setq num-cells (get-bms-val 'bms-cell-num))
                (if (> num-cells 0) {
                    (setq v-min (get-bms-val 'bms-v-cell 0))
                    (setq v-max v-min)
                    (looprange i 1 num-cells {
                        (setq cell-v (get-bms-val 'bms-v-cell i))
                        (if (< cell-v v-min) (setq v-min cell-v))
                        (if (> cell-v v-max) (setq v-max cell-v))
                    })
                })
            })
            (ext-bms v-min v-max temp-min temp-max temp-fet (get-bms-val 'bms-msg-age))
        })
        (sleep 0.2)
    })
})