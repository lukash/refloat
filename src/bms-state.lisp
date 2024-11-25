(def bms-fault-codes '(
    (NONE . 0)
    (BMS_COMM_TIMEOUT . 1)
    (BMS_SOFT_OVER_TEMP . 2)
    (BMS_HARD_OVER_TEMP . 3)
    (CELL_SOFT_OVER_VOLTAGE . 4)
    (CELL_HARD_OVER_VOLTAGE . 5)
    (CELL_SOFT_UNDER_VOLTAGE . 6)
    (CELL_HARD_UNDER_VOLTAGE . 7)
    (CELL_SOFT_OVER_TEMP . 8)
    (CELL_HARD_OVER_TEMP . 9)
    (CELL_SOFT_UNDER_TEMP . 10)
    (CELL_HARD_UNDER_TEMP . 11)
    (CELL_SOFT_BALANCE . 12)
    (CELL_HARD_BALANCE . 13)
))
(def bms-fault)
(def bms-vmin-limit-start)
(def bms-vmin-limit-end)
(def bms-vmax-limit-start)
(def bms-vmax-limit-end)
(def bms-tmax-limit-start)
(def bms-tmax-limit-end)
(def bms-discharge-tmax-limit-start)
(def bms-discharge-tmax-limit-end)
(def bms-tmin-limit-start -5)
(def bms-tmin-limit-end -10)
(def bms-cell-balance-start 0.05)
(def bms-cell-balance-end 0.1)
(def bms-config-update-time (systime))

(defun bms-set-fault (fault-code set-bit) {
    (if (eq fault-code 'NONE) {
        (setq bms-fault 0u32)
    }{
        (var bit-pos (- (assoc bms-fault-codes fault-code) 1))
        (if (= set-bit 1) (setq bms-fault (bitwise-or bms-fault (shl 1 bit-pos))) (setq bms-fault (bitwise-and bms-fault (bitwise-not (shl 1 bit-pos)))))
    })
})

(defun bms-update-config () {
    (if (!= (conf-get 'bms-limit-mode) 0) (conf-set 'bms-limit-mode 0)) ;Disable bms limit mode (soc temp volt) so it doesn't dump us.
    (setq bms-vmin-limit-start (conf-get 'bms-vmin-limit-start))
    (setq bms-vmin-limit-end (conf-get 'bms-vmin-limit-end))

    (setq bms-vmax-limit-start (conf-get 'bms-vmax-limit-start))
    (setq bms-vmax-limit-end (conf-get 'bms-vmax-limit-end))

    (setq bms-tmax-limit-start (conf-get 'bms-t-limit-start))
    (setq bms-tmax-limit-end (conf-get 'bms-t-limit-end))
    (setq bms-discharge-tmax-limit-start (+ bms-tmax-limit-start 10))
    (setq bms-discharge-tmax-limit-end (+ bms-tmax-limit-end 10))

    (setq bms-config-update-time (systime))
})

(defun bms-check-cell-volts () {
    (var num-cells (get-bms-val 'bms-cell-num))
    (if (> num-cells 0) {
        (var max-volt (get-bms-val 'bms-v-cell 0))
        (var min-volt (get-bms-val 'bms-v-cell 0))
        (looprange i 0 num-cells {
            (var cell-volt (get-bms-val 'bms-v-cell i))
            (if (> cell-volt max-volt)
                (setq max-volt cell-volt))
            (if (< cell-volt min-volt)
                (setq min-volt cell-volt))
            (cond
                ((> cell-volt bms-vmax-limit-end) {
                    (bms-set-fault 'CELL_HARD_OVER_VOLTAGE 1)
                    (bms-set-fault 'CELL_SOFT_OVER_VOLTAGE 0)
                    (break)
                })
                ((> cell-volt bms-vmax-limit-start)
                    (bms-set-fault 'CELL_SOFT_OVER_VOLTAGE 1))
                (t nil)
            )
            (cond
                ((< cell-volt bms-vmin-limit-end) {
                    (bms-set-fault 'CELL_HARD_UNDER_VOLTAGE 1)
                    (bms-set-fault 'CELL_SOFT_UNDER_VOLTAGE 0)
                    (break)
                })
                ((< cell-volt bms-vmin-limit-start)
                    (bms-set-fault 'CELL_SOFT_UNDER_VOLTAGE 1))
                (t nil)
            )
        })
        (var volt-diff (abs (- max-volt min-volt)))
        (cond
            ((> volt-diff bms-cell-balance-end) {
                (bms-set-fault 'CELL_HARD_BALANCE 1)
            })
            ((> volt-diff bms-cell-balance-start)
                (bms-set-fault 'CELL_SOFT_BALANCE 1))
            (t nil)
        )
    })
})

(defun bms-check-temps () {
    (looprange i 0 (get-bms-val 'bms-temp-adc-num) {
        (var adc-temp (get-bms-val 'bms-temps-adc i))
        (cond
            ((> adc-temp bms-tmax-limit-end) {
                (bms-set-fault 'CELL_HARD_OVER_TEMP 1)
                (bms-set-fault 'CELL_SOFT_OVER_TEMP 0)
                (break)
            })
            ((> adc-temp bms-tmax-limit-start)
                (bms-set-fault 'CELL_SOFT_OVER_TEMP 1))
            (t nil)
        )
        (cond
            ((< adc-temp bms-tmin-limit-end) {
                (bms-set-fault 'CELL_HARD_UNDER_TEMP 1)
                (bms-set-fault 'CELL_SOFT_UNDER_TEMP 0)
                (break)
            })
            ((< adc-temp bms-tmin-limit-start)
                (bms-set-fault 'CELL_SOFT_UNDER_TEMP 1))
            (t nil)
        )
    })
    (cond
        ((> (get-bms-val 'bms-temp-ic) bms-discharge-tmax-limit-end)
            (bms-set-fault 'BMS_HARD_OVER_TEMP 1))
        ((> (get-bms-val 'bms-temp-ic) bms-discharge-tmax-limit-start)
            (bms-set-fault 'BMS_SOFT_OVER_TEMP 1))
        (t nil)
    )
})

(defun bms-state-loop () {
    (bms-update-config)
    (bms-set-fault 'NONE 0)
    (loopwhile t {
        (if (>= (get-bms-val 'bms-can-id) 0) {
            (if (> (secs-since bms-config-update-time) 1.0) (bms-update-config))
            (if (> (get-bms-val 'bms-msg-age) 5.0) {
                (bms-set-fault 'BMS_COMM_TIMEOUT 1)
            } {
                (bms-set-fault 'NONE 0)
                (bms-check-cell-volts)
                (bms-check-temps)
            })
            (apply ext-bms-set-fault bms-fault)
        })
        (yield 100000)
    })
})