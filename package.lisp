(import "src/package_lib.bin" 'package-lib)
(load-native-lib package-lib)

(apply ext-set-fw-version (sysinfo 'fw-ver))
