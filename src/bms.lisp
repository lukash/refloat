(defun bms-loop () {
    (var bms-vmin-limit-start (conf-get 'bms-vmin-limit-start))
    (var bms-vmax-limit-start (conf-get 'bms-vmax-limit-start))
    (var bms-tmax-limit-start (- (conf-get 'bms-t-limit-start) 3)) ; Start 3 degrees before Motor CFG -> BMS limiting functionality would happen.
    (var bms-tmin-limit-start 0)
    (var bms-cell-balance-start 0.5)
    (var bms-ic-tmax-limit-start (+ bms-tmax-limit-start 15)) ; Set bms temp limit to +15C past cell limit
    (var bms-config-update-time (systime))
    (var bms-fault 0u32)
    (var bms-fault-codes '(
        (NONE . 0)
        (BMS_CONNECTION . 1)
        (BMS_OVER_TEMP . 2)
        (BMS_CELL_OVER_VOLTAGE . 3)
        (BMS_CELL_UNDER_VOLTAGE . 4)
        (BMS_CELL_OVER_TEMP . 5)
        (BMS_CELL_UNDER_TEMP . 6)
        (BMS_CELL_BALANCE . 7)
    ))
    (defun bms-set-fault (fault-code) {
        (if (eq fault-code 'NONE) {
            (setq bms-fault 0u32)
        }{
            (setq bms-fault (bitwise-or bms-fault (shl 1 (- (assoc bms-fault-codes fault-code) 1))))
        })
    })
    (loopwhile t {
        (if (and (<= (get-duty) 0.05)  (> (secs-since bms-config-update-time) 1.0)) { ; Only bother updating config values while board is idle
            (setq bms-vmin-limit-start (conf-get 'bms-vmin-limit-start))
            (setq bms-vmax-limit-start (conf-get 'bms-vmax-limit-start))
            (setq bms-tmax-limit-start (- (conf-get 'bms-t-limit-start) 3))
            (setq bms-ic-tmax-limit-start (+ bms-tmax-limit-start 15))
            (setq bms-config-update-time (systime))
        })
        (if (>= (get-bms-val 'bms-msg-age) 2.0) {
            (bms-set-fault 'BMS_CONNECTION)
        } {
            (bms-set-fault 'NONE)
            (if (>= (get-bms-val 'bms-v-cell-max) bms-vmax-limit-start) (bms-set-fault 'BMS_CELL_OVER_VOLTAGE))
            (if (<= (get-bms-val 'bms-v-cell-min) bms-vmin-limit-start) (bms-set-fault 'BMS_CELL_UNDER_VOLTAGE))
            (if (>= (get-bms-val 'bms-temp-cell-max) bms-tmax-limit-start) (bms-set-fault 'BMS_CELL_OVER_TEMP))
            (if (<= (get-bms-val 'bms-temp-cell-max) bms-tmin-limit-start) (bms-set-fault 'BMS_CELL_UNDER_TEMP))
            (if (>= (abs (- v-cell-max v-cell-min)) bms-cell-balance-start)  (bms-set-fault 'BMS_CELL_BALANCE))
            (if (>= (get-bms-val 'bms-temp-ic) bms-ic-tmax-limit-start) (bms-set-fault 'BMS_OVER_TEMP))
        })
        (apply ext-bms-set-fault bms-fault)
        (yield 10000)
    })
})
