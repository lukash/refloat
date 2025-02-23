(defun bms-loop () {
    (var vmin-limit)
    (var vmax-limit)
    (var tmax-limit)
    (var tmin-limit -10)
    (var cell-balance 0.5)
    (var mosfet-tmax-limit)
    (var config-update-time)
    (var fault 0u32)
    (var v-cell-support (eq (first (trap (get-bms-val 'bms-v-cell-min))) 'exit-ok))
    (var bms-data-version-support (and (eq (first (trap (get-bms-val 'bms-data-version))) 'exit-ok) (> (get-bms-val 'bms-data-version) 0)))
    (defun update-config () {
        (setq vmin-limit (conf-get 'bms-vmin-limit-start))
        (setq vmax-limit (conf-get 'bms-vmax-limit-start))
        (setq tmax-limit (- (conf-get 'bms-t-limit-start) 3)) ; Start 3 degrees before Motor CFG -> BMS limiting functionality would happen.
        (setq mosfet-tmax-limit (+ tmax-limit 15)) ; Set bms temp limit to +15C past cell limit
        (setq config-update-time (systime))
    })
    (var fault-codes '(
        (BMSF_NONE . 0)
        (BMSF_CONNECTION . 1)
        (BMSF_OVER_TEMP . 2)
        (BMSF_CELL_OVER_VOLTAGE . 3)
        (BMSF_CELL_UNDER_VOLTAGE . 4)
        (BMSF_CELL_OVER_TEMP . 5)
        (BMSF_CELL_UNDER_TEMP . 6)
        (BMSF_CELL_BALANCE . 7)
    ))
    (defun set-fault (fault-code) {
        (if (eq fault-code 'BMSF_NONE) {
            (setq fault 0u32)
        }{
            (setq fault (bitwise-or fault (shl 1 (- (assoc fault-codes fault-code) 1))))
        })
    })
    (update-config)
    (loopwhile t {
        (if (> (secs-since config-update-time) 1.0) {
            (update-config)
        })
        (if (or (= vmin-limit 0.0) (= vmax-limit 0.0) (= tmax-limit 0.0)) {
            (set-fault 'BMSF_NONE)
        } {
            (if (>= (get-bms-val 'bms-msg-age) 2.0) {
                (set-fault 'BMSF_CONNECTION)
            } {
                (set-fault 'BMSF_NONE)

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

                (var t-cell-min)
                (var t-cell-max)
                (if bms-data-version-support {
                    (setq t-cell-min (get-bms-val 'bms-temps-adc 1))
                    (setq t-cell-max (get-bms-val 'bms-temps-adc 2))
                    (var t-mosfet-temp (get-bms-val 'bms-temps-adc 3))
                    (if (> t-mosfet-temp -280) {
                        (if (>= t-mosfet-temp mosfet-tmax-limit) (set-fault 'BMSF_OVER_TEMP))
                    })
                } {
                    (setq t-cell-min (get-bms-val 'bms-temp-cell-max))
                    (setq t-cell-max t-cell-min)
                })

                (if (>= v-cell-max vmax-limit) (set-fault 'BMSF_CELL_OVER_VOLTAGE))
                (if (<= v-cell-min vmin-limit) (set-fault 'BMSF_CELL_UNDER_VOLTAGE))
                (if (>= t-cell-max tmax-limit) (set-fault 'BMSF_CELL_OVER_TEMP))
                (if (<= t-cell-min tmin-limit) (set-fault 'BMSF_CELL_UNDER_TEMP))
                (if (>= (abs (- v-cell-max v-cell-min)) cell-balance) (set-fault 'BMSF_CELL_BALANCE))
            })
        })
        (ext-bms-set-fault fault)
        (yield 200000)
    })
})
