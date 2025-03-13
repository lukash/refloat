(import "src/package_lib.bin" 'package-lib)
(load-native-lib package-lib)

(apply ext-set-fw-version (sysinfo 'fw-ver))

; Init bms
(if (>= (first (sysinfo 'fw-ver)) 6) {
    (import "bms.lisp" 'bms)
    (read-eval-program bms)
    (spawn "bms" 50 init-bms)
})
