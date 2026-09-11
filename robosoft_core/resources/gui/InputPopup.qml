// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.VirtualKeyboard 2.15

Popup {
    id: dialog
    modal: true
    focus: true
    closePolicy: Popup.NoAutoClose
    parent: window ? window.contentItem : null
    x: 0
    y: 0
    width: parent ? parent.width : 800
    height: parent ? parent.height : 480

    property string titleText: qsTr("Edit text")
    property var acceptHandler: null
    signal acceptedText(string text)

    function openWithText(value) {
        openFor(titleText, value, function(text) {
            acceptedText(text)
        })
    }

    function openFor(title, value, handler) {
        titleText = title
        editor.text = value
        acceptHandler = handler
        open()
    }

    onOpened: {
        Qt.callLater(function() {
            editor.selectAll()
            editor.forceActiveFocus()
            Qt.inputMethod.show()
        })
    }

    onClosed: Qt.inputMethod.hide()

    contentItem: Rectangle {
        anchors.fill: parent
        color: window.darkTheme ? "#1e1e1e" : "#f5f6f7"
        radius: 8

        Column {
            anchors {
                fill: parent
                margins: 20
                bottomMargin: inputPanel.visible ? inputPanel.height + 20 : 20
            }
            spacing: 12

            Label {
                text: dialog.titleText
                font.pixelSize: 20
                horizontalAlignment: Qt.AlignHCenter
                width: parent.width
            }

            TextField {
                id: editor
                width: parent.width
                placeholderText: qsTr("Type your text")
            }

            Row {
                spacing: 12
                width: parent.width
                RoboButton {
                    text: qsTr("Cancel")
                    width: (parent.width - 12) / 2
                    onClicked: dialog.close()
                }
                RoboButton {
                    text: qsTr("OK")
                    width: (parent.width - 12) / 2
                    onClicked: {
                        if (dialog.acceptHandler) {
                            dialog.acceptHandler(editor.text)
                        } else {
                            dialog.acceptedText(editor.text)
                        }
                        dialog.acceptHandler = null
                        dialog.close()
                    }
                }
            }
        }

        InputPanel {
            id: inputPanel
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            active: dialog.visible
            visible: active
        }
    }
}
