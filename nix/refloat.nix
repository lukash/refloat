{
  lib,
  stdenv,
  gcc-arm-embedded,
  vesc-tool,
  python3,
}:
stdenv.mkDerivation {
  pname = "refloat";
  version = "0.0.0-dev";

  src = ../.;

  postPatch = ''
    # Python shebang is failing for some reason. Replace it with full shebang
    sed -i '1s,.*,#!${lib.getExe python3},' rjsmin.py
    echo "$version" > ./version
  '';

  doFixup = false;

  nativeBuildInputs = [
    gcc-arm-embedded
    vesc-tool
    python3
  ];

  installPhase = ''
    mkdir $out
    cp src/package_lib.elf $out/refloat.elf
    cp refloat.vescpkg $out
    cat src/conf/confparser.h | sed -n 's/^#define .\+_SIGNATURE\W\+\([0-9]*\)/\1/p' > $out/config_signature.txt
  '';
}
