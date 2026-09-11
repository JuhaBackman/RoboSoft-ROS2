// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

import QtQuick 2.15
import QtQuick.Controls 2.15

MenuItem {
    id: control

    contentItem: Label {
        text: control.checkable && control.checked
              ? "\u2713 " + control.text : control.text
        color: control.highlighted
               ? control.palette.highlightedText
               : control.palette.buttonText
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }

    background: Rectangle {
        implicitWidth: 300
        implicitHeight: 40
        color: control.highlighted
               ? control.palette.highlight
               : control.palette.button
    }
}
