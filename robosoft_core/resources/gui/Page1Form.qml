// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Page {
    id: setupform

    header: Label {
        text: qsTr("SETUP")
        font.pixelSize: Qt.application.font.pixelSize * 2
        padding: 10
    }

    ColumnLayout {
        anchors.centerIn: parent
        anchors.verticalCenterOffset: -30
        width: Math.min(500, parent.width - 40)
        spacing: 12

        GridLayout {
            Layout.fillWidth: true
            columns: 2
            columnSpacing: 12
            rowSpacing: 12

            Label { text: qsTr("Robot:") }
            Label {
                Layout.fillWidth: true
                text: guiNode.robotDisplayName
                font.bold: true
            }

            Label { text: qsTr("Task:") }
            RowLayout {
                Layout.fillWidth: true
                spacing: 6

                TextField {
                    id: taskFile
                    Layout.fillWidth: true
                    text: guiNode.defaultTaskFile
                    readOnly: guiNode.virtualKeyboardEnabled
                    TapHandler {
                        enabled: guiNode.virtualKeyboardEnabled
                        onTapped: taskInput.openFor(qsTr("Task file"), taskFile.text,
                                                     function(value) { taskFile.text = value })
                    }
                }

                RoboButton {
                    id: taskChooserButton
                    Layout.preferredWidth: implicitHeight
                    text: "\u25bc"
                    onClicked: {
                        taskMenuItems.model = guiNode.availableTaskFiles()
                        if (taskMenuItems.model.length === 0) {
                            operationText.text = qsTr("No TASK files found")
                        } else {
                            taskMenu.popup(taskChooserButton, 0,
                                           taskChooserButton.height)
                        }
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            RoboButton {
                Layout.fillWidth: true
                text: "LOAD"
                onClicked: guiNode.selectTask(taskFile.text)
            }
            RoboButton {
                Layout.fillWidth: true
                text: "SAVE"
                onClicked: guiNode.saveTask(taskFile.text)
            }
        }

        Item { Layout.preferredHeight: 24 }

        GridLayout {
            Layout.fillWidth: true
            columns: 2
            columnSpacing: 12
            rowSpacing: 10

            Label { text: qsTr("ROS status:") }
            Label { id: rosStatus; text: qsTr("Waiting") }

            Label { text: qsTr("MAIN state:") }
            Label { id: mainState; text: "-"; font.bold: true }

            Label {
                id: waitingNodesLabel
                text: qsTr("Waiting nodes:")
                Layout.alignment: Qt.AlignTop
                visible: waitingNodes.visible
            }
            Label {
                id: waitingNodes
                Layout.fillWidth: true
                text: ""
                wrapMode: Text.Wrap
                visible: text.length > 0
            }

            Label {
                id: waitingMessagesLabel
                text: qsTr("Waiting messages:")
                Layout.alignment: Qt.AlignTop
                visible: waitingMessages.visible
            }
            Label {
                id: waitingMessages
                Layout.fillWidth: true
                text: ""
                wrapMode: Text.Wrap
                visible: text.length > 0
            }
        }

        Label { id: operationText; Layout.fillWidth: true; wrapMode: Text.Wrap }
    }

    RoboButton {
        id: themeChooserButton
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 16
        text: qsTr("Theme: %1 \u25bc").arg(window.darkTheme ? "Dark" : "Light")
        onClicked: themeMenu.popup(themeChooserButton, 0,
                                   -themeMenu.implicitHeight)
    }

    InputPopup { id: taskInput; parent: window.contentItem }

    Menu {
        id: taskMenu
        Instantiator {
            id: taskMenuItems
            model: []
            delegate: RoboMenuItem {
                required property string modelData
                text: modelData
                onTriggered: taskFile.text = modelData
            }
            onObjectAdded: (index, object) => taskMenu.insertItem(index, object)
            onObjectRemoved: (index, object) => taskMenu.removeItem(object)
        }
    }

    Menu {
        id: themeMenu
        RoboMenuItem {
            text: qsTr("Dark")
            checkable: true
            checked: window.darkTheme
            onTriggered: window.darkTheme = true
        }
        RoboMenuItem {
            text: qsTr("Light")
            checkable: true
            checked: !window.darkTheme
            onTriggered: window.darkTheme = false
        }
    }

    Connections {
        target: guiNode
        function onTaskLoaded(taskName) {
            operationText.text = qsTr("Loaded: %1").arg(taskName)
        }
        function onTaskSaved(filepath) {
            operationText.text = qsTr("Saved: %1").arg(filepath)
        }
        function onErrorOccurred(message) { operationText.text = message }
        function onSetupStatusChanged(rosStatusText, mainStateText, waitingFor) {
            rosStatus.text = rosStatusText
            mainState.text = mainStateText
            waitingNodes.text = waitingFor.indexOf("nodes: ") === 0
                    ? waitingFor.substring(7) : ""
            waitingMessages.text = waitingFor.indexOf("messages: ") === 0
                    ? waitingFor.substring(10) : ""
        }
    }
}
