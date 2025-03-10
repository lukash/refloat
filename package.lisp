(import "src/package_lib.bin" 'package-lib)

(load-native-lib package-lib)

; Switch Balance App to UART App
(if (= (conf-get 'app-to-use) 9) (conf-set 'app-to-use 3))

; Set firmware version:
(apply ext-set-fw-version (sysinfo 'fw-ver))
(def version_major (first (sysinfo 'fw-ver)))
(def version_minor (second (sysinfo 'fw-ver)))

; Setup thread for BMS Tiltback
(if (or (and (= version_major 6) (>= version_minor 6)) (>= version_major 7)) {
    (loopwhile-thd 50 t {
        (if (and (>= (get-bms-val 'bms-can-id) 0) (ext-bms)) {
            (var bms-temp-cell-max (get-bms-val 'bms-temp-cell-max))
            (var bms-temp-cell-min bms-temp-cell-max)
            (var bms-temp-mosfet -281)
            (if (= (get-bms-val 'bms-data-version) 1) {
                (setq bms-temp-cell-min (get-bms-val 'bms-temps-adc 1))
                (setq bms-temp-mosfet (get-bms-val 'bms-temps-adc 3))
            })
            (ext-bms (get-bms-val 'bms-v-cell-min) (get-bms-val 'bms-v-cell-max) bms-temp-cell-min bms-temp-cell-max bms-temp-mosfet (get-bms-val 'bms-msg-age))
        })
        (sleep 0.2)
    })
})

; Set to 1 to monitor debug variables
(define debug 1)

(if (= debug 1)
    (loopwhile t
        (progn
            (define pid_value (ext-dbg 1))
            (define proportional (ext-dbg 2))
            (define integral (ext-dbg 3))
            (define rate_p (ext-dbg 4))
            (define setpoint (ext-dbg 5))
            (define atr_offset (ext-dbg 6))
            (define erpm (ext-dbg 7))
            (define current (ext-dbg 8))
            (define atr_filtered_current (ext-dbg 9))
            (sleep 0.1)
)))
