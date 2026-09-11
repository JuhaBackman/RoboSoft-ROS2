// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

import QtQuick 2.15
import QtQuick.Controls 2.15

Button {
    id: control
    hoverEnabled: true

    background: Rectangle {
        implicitWidth: 100
        implicitHeight: 40
        radius: 4
        color: control.down || control.checked
               ? control.palette.highlight
               : (control.hovered
                  ? (window.darkTheme ? "#50555a" : "#d2d6dc")
                  : control.palette.button)
        border.width: control.visualFocus ? 2 : 1
        border.color: control.visualFocus
                      ? control.palette.highlight
                      : "#70757a"
        opacity: control.enabled ? 1.0 : 0.45

        Behavior on color {
            ColorAnimation { duration: 80 }
        }
    }
}
