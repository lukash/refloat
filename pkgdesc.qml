import QtQuick 2.15

Item {
    property string pkgName: "Refloat"
    property string pkgDescriptionMd: "package_README-gen.md"
    property string pkgLisp: "package.lisp"
    property string pkgQml: "ui.qml"
    property bool pkgQmlIsFullscreen: false
    property string pkgOutput: "refloat.vescpkg"

    function isCompatible (fwRxParams) {
        if (fwRxParams.hwTypeStr() != "vesc") {
            return false;
        }

        return true;
    }
}
