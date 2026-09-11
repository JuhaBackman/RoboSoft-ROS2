// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Page {
    id: debugPage

    function refreshTopics() {
        topicSelector.model = guiNode.availableTopics()
    }

    function addSelectedTopic() {
        if (topicSelector.currentIndex < 0 || !topicSelector.currentText)
            return

        var topic = topicSelector.currentText
        for (var i = 0; i < selectedTopics.count; ++i) {
            if (selectedTopics.get(i).topicName === topic)
                return
        }

        selectedTopics.append({
            "topicName": topic,
            "collapsed": false,
            "sample": "Waiting for data...",
            "errorText": ""
        })
        guiNode.startTopicMonitor(topic)
    }

    function topicIndex(topic) {
        for (var i = 0; i < selectedTopics.count; ++i) {
            if (selectedTopics.get(i).topicName === topic)
                return i
        }
        return -1
    }

    header: ColumnLayout {
        width: parent.width
        spacing: 6

        Label {
            text: qsTr("Debug")
            font.pixelSize: Qt.application.font.pixelSize * 2
            padding: 10
        }

        ListModel {
            id: timingModel
            ListElement { componentName: "Main"; targetMs: 100; durationMs: 0; averageHz: 0; targetHz: 10; frequencySamples: 0; marginMs: 0; latenessMs: 0; misses: 0; received: false }
            ListElement { componentName: "NMPC"; targetMs: 100; durationMs: 0; averageHz: 0; targetHz: 10; frequencySamples: 0; marginMs: 0; latenessMs: 0; misses: 0; received: false }
        }

        Repeater {
            model: timingModel
            delegate: ColumnLayout {
                required property int index
                required property string componentName
                required property real targetMs
                required property real durationMs
                required property real averageHz
                required property real targetHz
                required property int frequencySamples
                required property real marginMs
                required property real latenessMs
                required property int misses
                required property bool received
                Layout.fillWidth: true
                Layout.leftMargin: 10
                Layout.rightMargin: 10
                spacing: 2

                Label {
                    text: received
                          ? componentName + "  callback " + durationMs.toFixed(1)
                            + " ms  " + averageHz.toFixed(2) + " Hz avg ("
                            + frequencySamples + "/100)  misses " + misses
                          : componentName + "  waiting for timing data"
                    font.pixelSize: 15
                }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 2
                    Rectangle {
                        Layout.fillWidth: true
                        height: 16
                        color: "#343a40"
                        border.color: "#777777"
                        Rectangle {
                            anchors.right: parent.right
                            anchors.top: parent.top
                            anchors.bottom: parent.bottom
                            width: parent.width * Math.max(0, Math.min(1, marginMs / Math.max(1, targetMs)))
                            color: "#39b54a"
                        }
                    }
                    Rectangle { width: 2; height: 20; color: "white" }
                    Rectangle {
                        Layout.fillWidth: true
                        height: 16
                        color: "#343a40"
                        border.color: "#777777"
                        Rectangle {
                            anchors.left: parent.left
                            anchors.top: parent.top
                            anchors.bottom: parent.bottom
                            width: parent.width * Math.max(0, Math.min(1, latenessMs / Math.max(1, targetMs)))
                            color: "#d83232"
                        }
                    }
                }
                RowLayout {
                    Layout.fillWidth: true
                    Label { text: marginMs.toFixed(1) + " ms margin"; color: "#65d875"; Layout.fillWidth: true }
                    Label { text: latenessMs.toFixed(1) + " ms late"; color: "#ff6666"; Layout.fillWidth: true; horizontalAlignment: Text.AlignRight }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.leftMargin: 10
            Layout.rightMargin: 10
            Layout.bottomMargin: 8

            ComboBox {
                id: topicSelector
                Layout.fillWidth: true
                model: []
                onPressedChanged: if (pressed) debugPage.refreshTopics()
            }
            RoboButton {
                text: qsTr("Refresh")
                onClicked: debugPage.refreshTopics()
            }
            RoboButton {
                text: qsTr("Add")
                enabled: topicSelector.currentIndex >= 0
                onClicked: debugPage.addSelectedTopic()
            }
        }
    }

    ListModel { id: selectedTopics }

    Label {
        anchors.centerIn: parent
        visible: selectedTopics.count === 0
        text: qsTr("Select a ROS topic from the drop-down list")
        opacity: 0.65
    }

    ListView {
        id: topicGroups
        anchors.fill: parent
        model: selectedTopics
        spacing: 4
        clip: true

        delegate: Column {
            id: topicGroup
            required property int index
            required property string topicName
            required property bool collapsed
            required property string sample
            required property string errorText
            width: ListView.view.width

            Rectangle {
                width: parent.width
                height: 54
                color: "white"
                border.color: "black"
                border.width: 2

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 14
                    anchors.rightMargin: 8

                    Label {
                        text: (topicGroup.collapsed ? "▶  " : "▼  ") + topicGroup.topicName
                        color: "black"
                        font.pixelSize: 20
                        font.bold: true
                        Layout.fillWidth: true
                    }
                    RoboButton {
                        text: qsTr("Remove")
                        onClicked: {
                            guiNode.stopTopicMonitor(topicGroup.topicName)
                            selectedTopics.remove(topicGroup.index)
                        }
                    }
                }

                TapHandler {
                    onTapped: selectedTopics.setProperty(
                                  topicGroup.index, "collapsed", !topicGroup.collapsed)
                }
            }

            Column {
                width: parent.width
                visible: !topicGroup.collapsed

                Repeater {
                    model: topicGroup.errorText.length > 0
                           ? ["error: " + topicGroup.errorText]
                           : topicGroup.sample.split("\n")

                    delegate: Rectangle {
                        required property int index
                        required property string modelData
                        width: topicGroup.width
                        height: modelData.trim().length > 0 ? 38 : 0
                        visible: height > 0
                        color: index % 2 ? "#bdbdbd" : "#d0d0d0"
                        border.color: "#555555"
                        border.width: 1

                        property int colon: modelData.indexOf(":")
                        property string fieldName: colon >= 0
                                                   ? modelData.slice(0, colon).trim()
                                                   : modelData.trim()
                        property string fieldValue: colon >= 0
                                                    ? modelData.slice(colon + 1).trim()
                                                    : ""

                        Row {
                            anchors.fill: parent
                            anchors.leftMargin: 20
                            anchors.rightMargin: 10

                            Text {
                                width: parent.width * 0.42
                                anchors.verticalCenter: parent.verticalCenter
                                text: fieldName
                                color: "black"
                                font.pixelSize: 16
                                elide: Text.ElideRight
                            }
                            Text {
                                width: parent.width * 0.58
                                anchors.verticalCenter: parent.verticalCenter
                                text: fieldValue
                                color: "black"
                                font.pixelSize: 16
                                font.family: "monospace"
                                elide: Text.ElideRight
                            }
                        }
                    }
                }
            }
        }
    }

    Connections {
        target: guiNode
        function onTimerPerformanceUpdated(component, targetPeriodMs, actualPeriodMs,
                                           callbackDurationMs, marginMs, latenessMs,
                                           deadlineMisses) {
            for (var i = 0; i < timingModel.count; ++i) {
                if (timingModel.get(i).componentName === component) {
                    timingModel.setProperty(i, "targetMs", targetPeriodMs)
                    timingModel.setProperty(i, "durationMs", callbackDurationMs)
                    timingModel.setProperty(i, "marginMs", marginMs)
                    timingModel.setProperty(i, "latenessMs", latenessMs)
                    timingModel.setProperty(i, "misses", deadlineMisses)
                    timingModel.setProperty(i, "received", true)
                    break
                }
            }
        }
        function onTimerFrequencyUpdated(component, averageHz, targetHz,
                                         sampleCount) {
            for (var i = 0; i < timingModel.count; ++i) {
                if (timingModel.get(i).componentName === component) {
                    timingModel.setProperty(i, "averageHz", averageHz)
                    timingModel.setProperty(i, "targetHz", targetHz)
                    timingModel.setProperty(i, "frequencySamples", sampleCount)
                    break
                }
            }
        }
        function onTopicSample(topicName, yaml) {
            var row = debugPage.topicIndex(topicName)
            if (row >= 0) {
                selectedTopics.setProperty(row, "sample", yaml)
                selectedTopics.setProperty(row, "errorText", "")
            }
        }
        function onTopicMonitorError(topicName, message) {
            var row = debugPage.topicIndex(topicName)
            if (row >= 0)
                selectedTopics.setProperty(row, "errorText", message)
        }
    }

    Component.onCompleted: refreshTopics()
}
