(import "src/package_lib.bin" 'package-lib)

(load-native-lib package-lib)

; Switch Balance App to UART App
(if (= (conf-get 'app-to-use) 9) (conf-set 'app-to-use 3))

; Set firmware version:
(apply ext-set-fw-version (sysinfo 'fw-ver))

(def version_major (first (sysinfo 'fw-ver)))
(def version_minor (second (sysinfo 'fw-ver)))
(if (or (eq (first (trap (get-bms-val 'bms-v-cell-min))) 'exit-ok) (or (>= version_major 7) (and (>= version_major 6) (>= version_minor 5)))) {
    (loopwhile (and (< (get-bms-val 'bms-can-id) 0) (< (secs-since 0) 10.0)) (yield 1000000))
    (if (>= (get-bms-val 'bms-can-id) 0) {
        (import "src/bms.lisp" 'bms)
        (read-eval-program bms)
        (spawn bms-loop)
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
