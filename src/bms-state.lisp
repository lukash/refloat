(defun bms-state-loop () {
    (var bms-vmin-limit-start (conf-get 'bms-vmin-limit-start))
    (var bms-vmax-limit-start (conf-get 'bms-vmax-limit-start))
    (var bms-tmax-limit-start (- (conf-get 'bms-t-limit-start) 3)) ; start 3 degrees before we start limiting
    (var bms-cell-balance-start 0.5)
    (var bms-discharge-tmax-limit-start (+ bms-tmax-limit-start 10)) ; set bms mosfet discharge limit to +10 deg c past cells
    (var bms-config-update-time (systime))
    (var bms-fault 0u32)

    (var v-cell-support (>= (+ (first (sysinfo 'fw-ver)) (* (second (sysinfo 'fw-ver)) 0.01)) 6.06))

    (var bms-fault-codes '(
        (NONE . 0)
        (BMS_COMM_TIMEOUT . 1)
        (BMS_OVER_TEMP . 2)
        (CELL_OVER_VOLTAGE . 3)
        (CELL_UNDER_VOLTAGE . 4)
        (CELL_OVER_TEMP . 5)
        (CELL_UNDER_TEMP . 6)
        (CELL_BALANCE . 7)
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
            (setq bms-tmax-limit-start (- (conf-get 'bms-t-limit-start) 3)) ; start 3 degrees before we start limiting
            (setq bms-discharge-tmax-limit-start (+ bms-tmax-limit-start 10)) ; set bms mosfet discharge limit to +10 deg c past cells
            (setq bms-config-update-time (systime))
        })
        (if (>= (get-bms-val 'bms-msg-age) 2.0) {
            (bms-set-fault 'BMS_COMM_TIMEOUT)
        } {
            (bms-set-fault 'NONE)
            (var v-cell-min)
            (var v-cell-max)
            (if v-cell-support {
                (setq v-cell-min (get-bms-val 'bms-v-cell-min))
                (setq v-cell-max (get-bms-val 'bms-v-cell-max))
            } {
                (var num-cells (get-bms-val 'bms-cell-num))
                (if (> num-cells 1) {
                    (setq v-cell-max (get-bms-val 'bms-v-cell 0))
                    (setq v-cell-min (get-bms-val 'bms-v-cell 0))
                    (looprange i 1 num-cells {
                        (var cell-volt (get-bms-val 'bms-v-cell i))
                        (if (> cell-volt v-cell-max)
                            (setq v-cell-max cell-volt))
                        (if (< cell-volt v-cell-min)
                            (setq v-cell-min cell-volt))
                    })
                })
            })
            (if (>= v-cell-max bms-vmax-limit-start) (bms-set-fault 'CELL_OVER_VOLTAGE))
            (if (<= v-cell-min bms-vmin-limit-start) (bms-set-fault 'CELL_UNDER_VOLTAGE))
            (if (>= (get-bms-val 'bms-temp-cell-max) bms-tmax-limit-start) (bms-set-fault 'CELL_OVER_TEMP))
            (if (>= (abs (- v-cell-max v-cell-min)) bms-cell-balance-start)  (bms-set-fault 'CELL_BALANCE))
            (if (>= (get-bms-val 'bms-temp-ic) bms-discharge-tmax-limit-start) (bms-set-fault 'BMS_OVER_TEMP))
        })
        (apply ext-bms-set-fault bms-fault)
        (yield 50000)
    })
})