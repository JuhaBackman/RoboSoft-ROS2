// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Page {
    id: controlform
    property bool automaticMode: false
    property bool running: false
    property real vehicleX: 0
    property real vehicleY: 0
    property real vehicleHeading: 0
    property real vehicleSpeed: 0
    property real lateralError: 0
    property bool odometryReceived: false
    property string mapViewMode: "centered"
    property real mapZoom: 1.0
    property real mapPixelsPerMeter: 20.0
    property bool historyVisible: true
    property var routePoints: []
    property var activeRoutePoints: []
    property var vehicleHistory: []
    property var remoteRobots: []
    property var lidarLandmarks: []
    property var lidarObservations: []
    property real mainExecutionTimeMs: 0
    property real nmpcExecutionTimeMs: 0
    property bool mainExecutionTimeReceived: false
    property bool nmpcExecutionTimeReceived: false
    property bool lidarSafetyStop: false
    property bool emergencyStopActive: false
    readonly property var remoteRobotColors: [
        "#29b6f6", "#ef5350", "#ab47bc", "#66bb6a", "#ff7043",
        "#26a69a", "#ec407a", "#7e57c2"
    ]

    function remoteRobotIndex(name) {
        for (var i = 0; i < remoteRobots.length; ++i) {
            if (remoteRobots[i].name === name)
                return i
        }
        return -1
    }

    function updateRemoteOdometry(name, x, y, heading, velocity) {
        var robots = remoteRobots.slice()
        var index = remoteRobotIndex(name)
        var robot = index >= 0 ? Object.assign({}, robots[index]) : {
            "name": name,
            "color": remoteRobotColors[robots.length % remoteRobotColors.length],
            "history": [], "path": []
        }
        robot.x = x
        robot.y = y
        robot.heading = heading
        robot.speed = velocity
        robot.received = true
        if (historyVisible) {
            var history = robot.history.slice()
            if (history.length === 0 ||
                    Math.hypot(x - history[history.length - 1].x,
                               y - history[history.length - 1].y) > 0.25) {
                history.push({"x": x, "y": y})
                if (history.length > 2000)
                    history.shift()
                robot.history = history
            }
        }
        if (index >= 0)
            robots[index] = robot
        else
            robots.push(robot)
        remoteRobots = robots
    }

    function updateRemotePath(name, points) {
        var robots = remoteRobots.slice()
        var index = remoteRobotIndex(name)
        var robot = index >= 0 ? Object.assign({}, robots[index]) : {
            "name": name,
            "color": remoteRobotColors[robots.length % remoteRobotColors.length],
            "history": [], "received": false
        }
        robot.path = points
        if (index >= 0)
            robots[index] = robot
        else
            robots.push(robot)
        remoteRobots = robots
    }

    function clearHistories() {
        vehicleHistory = []
        var robots = remoteRobots.slice()
        for (var i = 0; i < robots.length; ++i) {
            var robot = Object.assign({}, robots[i])
            robot.history = []
            robots[i] = robot
        }
        remoteRobots = robots
    }

    function executionTimeColor(milliseconds, received) {
        if (!received)
            return window.darkTheme ? "#aab7ae" : "#4f5b53"
        if (milliseconds < 80)
            return window.darkTheme ? "#55cc66" : "#187b2d"
        if (milliseconds <= 100)
            return window.darkTheme ? "#ffeb3b" : "#9a6700"
        return window.darkTheme ? "#ff5252" : "#b3261e"
    }

    function executionTimeText(name, milliseconds, received) {
        return name + ": " + (received ? milliseconds.toFixed(1) + " ms" : "--")
    }

    function safetyStopText() {
        if (lidarSafetyStop && emergencyStopActive)
            return qsTr("SAFETY STOP: LIDAR + E-STOP")
        if (lidarSafetyStop)
            return qsTr("SAFETY STOP: LIDAR")
        return qsTr("SAFETY STOP: E-STOP")
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true

            Frame {
                Layout.fillWidth: true
                Layout.fillHeight: true
                padding: 0

                Rectangle {
                    id: mapArea
                    anchors.fill: parent
                    color: window.darkTheme ? "#26342b" : "#ffffff"
                    clip: true

                    Canvas {
                        id: routeCanvas
                        anchors.fill: parent
                        onPaint: {
                            var ctx = getContext("2d")
                            ctx.clearRect(0, 0, width, height)
                            var all = controlform.routePoints.concat(
                                        controlform.vehicleHistory,
                                        controlform.lidarLandmarks)
                            all.push({"x": controlform.vehicleX,
                                      "y": controlform.vehicleY})
                            for (var robotIndex = 0;
                                 robotIndex < controlform.remoteRobots.length;
                                 ++robotIndex) {
                                var remote = controlform.remoteRobots[robotIndex]
                                if (remote.path)
                                    all = all.concat(remote.path)
                                if (controlform.historyVisible && remote.history)
                                    all = all.concat(remote.history)
                                if (remote.received)
                                    all.push({"x": remote.x, "y": remote.y})
                            }
                            if (all.length < 1)
                                return
                            var minX = all[0].x, maxX = all[0].x
                            var minY = all[0].y, maxY = all[0].y
                            for (var i = 1; i < all.length; ++i) {
                                minX = Math.min(minX, all[i].x)
                                maxX = Math.max(maxX, all[i].x)
                                minY = Math.min(minY, all[i].y)
                                maxY = Math.max(maxY, all[i].y)
                            }
                            var centerX = (minX + maxX) / 2
                            var centerY = (minY + maxY) / 2
                            var spanX = Math.max(10, maxX - minX)
                            var spanY = Math.max(10, maxY - minY)
                            var pixelsPerMeter = controlform.mapPixelsPerMeter
                                                 * controlform.mapZoom
                            var autoCenterScreenX = width / 2
                            var autoCenterScreenY = height / 2 - 20
                            var autoPixelsPerMeter = controlform.mapZoom *
                                    Math.min(Math.max(1, width - 120) / spanX,
                                             Math.max(1, height - 80) / spanY)
                            var worldMinX, worldMaxX, worldMinY, worldMaxY
                            if (controlform.mapViewMode === "auto") {
                                worldMinX = centerX - autoCenterScreenX /
                                            autoPixelsPerMeter
                                worldMaxX = centerX + (width -
                                            autoCenterScreenX) /
                                            autoPixelsPerMeter
                                worldMinY = centerY - (height -
                                            autoCenterScreenY) /
                                            autoPixelsPerMeter
                                worldMaxY = centerY + autoCenterScreenY /
                                            autoPixelsPerMeter
                            } else {
                                worldMinX = controlform.vehicleX -
                                            width / (2 * pixelsPerMeter)
                                worldMaxX = controlform.vehicleX +
                                            width / (2 * pixelsPerMeter)
                                worldMinY = controlform.vehicleY -
                                            height / (2 * pixelsPerMeter)
                                worldMaxY = controlform.vehicleY +
                                            height / (2 * pixelsPerMeter)
                            }
                            function sx(x) {
                                if (controlform.mapViewMode === "auto")
                                    return autoCenterScreenX +
                                           (x - centerX) * autoPixelsPerMeter
                                return width / 2 +
                                       (x - controlform.vehicleX) * pixelsPerMeter
                            }
                            function sy(y) {
                                if (controlform.mapViewMode === "auto")
                                    return autoCenterScreenY -
                                           (y - centerY) * autoPixelsPerMeter
                                return height / 2 -
                                       (y - controlform.vehicleY) * pixelsPerMeter
                            }
                            function niceGridSpacing(rawSpacing) {
                                rawSpacing = Math.max(0.000001, rawSpacing)
                                var exponent = Math.pow(
                                            10, Math.floor(Math.log(rawSpacing)
                                                           / Math.LN10))
                                var fraction = rawSpacing / exponent
                                if (fraction <= 1)
                                    return exponent
                                if (fraction <= 2)
                                    return 2 * exponent
                                if (fraction <= 5)
                                    return 5 * exponent
                                return 10 * exponent
                            }
                            var metersPerPixelX = (worldMaxX - worldMinX) /
                                                  Math.max(1, width)
                            var metersPerPixelY = (worldMaxY - worldMinY) /
                                                  Math.max(1, height)
                            var gridSpacing = niceGridSpacing(
                                        40 * Math.max(metersPerPixelX,
                                                      metersPerPixelY))
                            ctx.strokeStyle = window.darkTheme
                                              ? "#415348" : "#d5ddd7"
                            ctx.lineWidth = 1
                            var gridX = Math.ceil(worldMinX / gridSpacing) *
                                        gridSpacing
                            for (; gridX <= worldMaxX; gridX += gridSpacing) {
                                ctx.beginPath()
                                ctx.moveTo(sx(gridX), 0)
                                ctx.lineTo(sx(gridX), height)
                                ctx.stroke()
                            }
                            var gridY = Math.ceil(worldMinY / gridSpacing) *
                                        gridSpacing
                            for (; gridY <= worldMaxY; gridY += gridSpacing) {
                                ctx.beginPath()
                                ctx.moveTo(0, sy(gridY))
                                ctx.lineTo(width, sy(gridY))
                                ctx.stroke()
                            }
                            if (controlform.routePoints.length > 1) {
                                ctx.strokeStyle = window.darkTheme
                                                  ? "#4caf50" : "#2e7d32"
                                ctx.lineWidth = 3
                                ctx.beginPath()
                                ctx.moveTo(sx(controlform.routePoints[0].x),
                                           sy(controlform.routePoints[0].y))
                                for (i = 1; i < controlform.routePoints.length; ++i)
                                    ctx.lineTo(sx(controlform.routePoints[i].x),
                                               sy(controlform.routePoints[i].y))
                                ctx.stroke()
                            }
                            if (controlform.activeRoutePoints.length > 1) {
                                ctx.strokeStyle = window.darkTheme
                                                  ? "#00bcd4" : "#0277bd"
                                ctx.lineWidth = 5
                                ctx.beginPath()
                                ctx.moveTo(sx(controlform.activeRoutePoints[0].x),
                                           sy(controlform.activeRoutePoints[0].y))
                                for (i = 1;
                                     i < controlform.activeRoutePoints.length; ++i)
                                    ctx.lineTo(
                                        sx(controlform.activeRoutePoints[i].x),
                                        sy(controlform.activeRoutePoints[i].y))
                                ctx.stroke()
                            }
                            // The driven history is the live trace and is
                            // intentionally drawn over the loaded task route.
                            if (controlform.historyVisible &&
                                    controlform.vehicleHistory.length > 1) {
                                ctx.strokeStyle = window.darkTheme
                                                  ? "#ffc107" : "#d17d00"
                                ctx.lineWidth = 2
                                ctx.beginPath()
                                ctx.moveTo(sx(controlform.vehicleHistory[0].x),
                                           sy(controlform.vehicleHistory[0].y))
                                for (i = 1; i < controlform.vehicleHistory.length; ++i)
                                    ctx.lineTo(sx(controlform.vehicleHistory[i].x),
                                               sy(controlform.vehicleHistory[i].y))
                                ctx.stroke()
                            }
                            for (robotIndex = 0;
                                 robotIndex < controlform.remoteRobots.length;
                                 ++robotIndex) {
                                remote = controlform.remoteRobots[robotIndex]
                                if (remote.path && remote.path.length > 1) {
                                    ctx.strokeStyle = remote.color
                                    ctx.lineWidth = 3
                                    ctx.beginPath()
                                    ctx.moveTo(sx(remote.path[0].x),
                                               sy(remote.path[0].y))
                                    for (i = 1; i < remote.path.length; ++i)
                                        ctx.lineTo(sx(remote.path[i].x),
                                                   sy(remote.path[i].y))
                                    ctx.stroke()
                                }
                                if (controlform.historyVisible && remote.history &&
                                        remote.history.length > 1) {
                                    ctx.strokeStyle = remote.color
                                    ctx.lineWidth = 2
                                    ctx.beginPath()
                                    ctx.moveTo(sx(remote.history[0].x),
                                               sy(remote.history[0].y))
                                    for (i = 1; i < remote.history.length; ++i)
                                        ctx.lineTo(sx(remote.history[i].x),
                                                   sy(remote.history[i].y))
                                    ctx.stroke()
                                }
                            }
                            // Fixed screen-size symbols remain legible at every zoom.
                            ctx.lineWidth = 2
                            ctx.strokeStyle = window.darkTheme ? "#f1f3f4" : "#202124"
                            for (var landmark of controlform.lidarLandmarks) {
                                ctx.beginPath()
                                ctx.arc(sx(landmark.x), sy(landmark.y), 6, 0, 2 * Math.PI)
                                ctx.stroke()
                            }
                            ctx.strokeStyle = window.darkTheme ? "#ff80ab" : "#ad1457"
                            for (var observation of controlform.lidarObservations) {
                                var ox = sx(observation.x), oy = sy(observation.y)
                                ctx.beginPath()
                                ctx.moveTo(ox - 5, oy - 5)
                                ctx.lineTo(ox + 5, oy + 5)
                                ctx.moveTo(ox - 5, oy + 5)
                                ctx.lineTo(ox + 5, oy - 5)
                                ctx.stroke()
                            }
                            ctx.save()
                            ctx.translate(sx(controlform.vehicleX),
                                          sy(controlform.vehicleY))
                            ctx.rotate(-controlform.vehicleHeading)
                            ctx.fillStyle = "#ffc107"
                            ctx.strokeStyle = "black"
                            ctx.lineWidth = 2
                            ctx.beginPath()
                            // ROS yaw zero points along +X. Canvas Y grows
                            // downwards, so -yaw gives the visual CCW rotation.
                            ctx.moveTo(14, 0)
                            ctx.lineTo(-11, 9)
                            ctx.lineTo(-11, -9)
                            ctx.closePath()
                            ctx.fill()
                            ctx.stroke()
                            ctx.restore()
                            ctx.fillStyle = window.darkTheme
                                            ? "#f1f3f4" : "#202124"
                            ctx.font = "bold 14px sans-serif"
                            ctx.textAlign = "center"
                            ctx.textBaseline = "top"
                            ctx.fillText(guiNode.robotDisplayName,
                                         sx(controlform.vehicleX),
                                         sy(controlform.vehicleY) + 20)
                            ctx.font = "13px sans-serif"
                            ctx.fillText(
                                        controlform.vehicleSpeed.toFixed(2) +
                                        " m/s",
                                        sx(controlform.vehicleX),
                                        sy(controlform.vehicleY) + 37)
                            ctx.fillText(
                                        qsTr("Lateral error: ") +
                                        controlform.lateralError.toFixed(2) +
                                        " m",
                                        sx(controlform.vehicleX),
                                        sy(controlform.vehicleY) + 54)
                            for (robotIndex = 0;
                                 robotIndex < controlform.remoteRobots.length;
                                 ++robotIndex) {
                                remote = controlform.remoteRobots[robotIndex]
                                if (!remote.received)
                                    continue
                                ctx.save()
                                ctx.translate(sx(remote.x), sy(remote.y))
                                ctx.rotate(-remote.heading)
                                ctx.fillStyle = remote.color
                                ctx.strokeStyle = "black"
                                ctx.lineWidth = 2
                                ctx.beginPath()
                                ctx.moveTo(14, 0)
                                ctx.lineTo(-11, 9)
                                ctx.lineTo(-11, -9)
                                ctx.closePath()
                                ctx.fill()
                                ctx.stroke()
                                ctx.restore()
                                ctx.fillStyle = remote.color
                                ctx.font = "bold 14px sans-serif"
                                ctx.textAlign = "center"
                                ctx.textBaseline = "top"
                                ctx.fillText(remote.name, sx(remote.x),
                                             sy(remote.y) + 20)
                                ctx.font = "13px sans-serif"
                                ctx.fillText(remote.speed.toFixed(2) + " m/s",
                                             sx(remote.x), sy(remote.y) + 37)
                            }

                            // Show the metric size of one grid square. The
                            // bar has exactly the same pixel length as the
                            // current world-anchored grid spacing.
                            var activePixelsPerMeter =
                                    controlform.mapViewMode === "auto" ?
                                    autoPixelsPerMeter : pixelsPerMeter
                            var gridBarPixels = gridSpacing *
                                                activePixelsPerMeter
                            var roundedGrid = Math.round(gridSpacing)
                            var gridText = Math.abs(gridSpacing - roundedGrid) <
                                           0.000001 ?
                                           roundedGrid.toString() :
                                           gridSpacing.toPrecision(2)
                            var scaleRight = width - 14
                            var scaleBottom = height - 12
                            var scaleLeft = scaleRight - gridBarPixels
                            ctx.fillStyle = window.darkTheme
                                            ? "rgba(24, 31, 27, 0.82)"
                                            : "rgba(255, 255, 255, 0.90)"
                            ctx.fillRect(scaleLeft - 9, scaleBottom - 37,
                                         gridBarPixels + 18, 43)
                            ctx.strokeStyle = window.darkTheme
                                              ? "#f1f3f4" : "#202124"
                            ctx.lineWidth = 2
                            ctx.beginPath()
                            ctx.moveTo(scaleLeft, scaleBottom - 8)
                            ctx.lineTo(scaleLeft, scaleBottom)
                            ctx.lineTo(scaleRight, scaleBottom)
                            ctx.lineTo(scaleRight, scaleBottom - 8)
                            ctx.stroke()
                            ctx.fillStyle = window.darkTheme
                                            ? "#f1f3f4" : "#202124"
                            ctx.font = "13px sans-serif"
                            ctx.textAlign = "center"
                            ctx.textBaseline = "bottom"
                            ctx.fillText("Grid: " + gridText + " m",
                                         (scaleLeft + scaleRight) / 2,
                                         scaleBottom - 11)
                        }
                    }
                    Column {
                        anchors.left: parent.left
                        anchors.top: parent.top
                        anchors.margins: 10
                        spacing: 2

                        Label {
                            text: controlform.executionTimeText(
                                      "MAIN", controlform.mainExecutionTimeMs,
                                      controlform.mainExecutionTimeReceived)
                            color: controlform.executionTimeColor(
                                       controlform.mainExecutionTimeMs,
                                       controlform.mainExecutionTimeReceived)
                            font.bold: true
                        }
                        Label {
                            text: controlform.executionTimeText(
                                      "NMPC", controlform.nmpcExecutionTimeMs,
                                      controlform.nmpcExecutionTimeReceived)
                            color: controlform.executionTimeColor(
                                       controlform.nmpcExecutionTimeMs,
                                       controlform.nmpcExecutionTimeReceived)
                            font.bold: true
                        }
                        Label {
                            visible: controlform.lidarSafetyStop ||
                                     controlform.emergencyStopActive
                            text: controlform.safetyStopText()
                            color: window.darkTheme ? "#ff5252" : "#b3261e"
                            font.bold: true
                            font.pixelSize: 16
                        }
                    }
                    Label {
                        anchors.centerIn: parent
                        visible: !controlform.odometryReceived
                        text: qsTr("Waiting for odometry")
                        color: window.darkTheme ? "#aab7ae" : "#4f5b53"
                    }
                }
            }

            ColumnLayout {
                Layout.margins: 10
                Layout.alignment: Qt.AlignTop
                Layout.preferredWidth: 100

                RoboButton {
                    Layout.preferredWidth: 100; Layout.preferredHeight: 100
                    text: controlform.automaticMode ? "AUTO" : "MAN"
                    enabled: guiNode.guiModeSelectionEnabled
                    onClicked: {
                        controlform.automaticMode = !controlform.automaticMode
                        guiNode.setRobotMode(controlform.automaticMode ? 1 : 0)
                    }
                }
                RoboButton {
                    Layout.preferredWidth: 100; Layout.preferredHeight: 100
                    text: controlform.running ? "STOP"
                                                   : (controlform.automaticMode
                                                      ? "START" : "RECORD")
                    onClicked: {
                        if (controlform.running) {
                            if (controlform.automaticMode)
                                guiNode.stopAutonomous()
                            else
                                guiNode.stopRecording()
                        } else {
                            if (controlform.automaticMode)
                                guiNode.startAutonomous()
                            else
                                guiNode.startRecording()
                        }
                    }
                }
                RoboButton {
                    Layout.preferredWidth: 100; Layout.preferredHeight: 100
                    text: "MAP"
                    onClicked: mapPopup.open()
                    Popup {
                        id: mapPopup; x: -170; y: -100; focus: true
                        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent
                        ColumnLayout {
                            ComboBox {
                                id: routeSelector
                                Layout.preferredWidth: 220
                                model: []
                                onActivated: guiNode.selectGuidance(currentIndex)
                            }
                            RoboButton {
                                text: controlform.mapViewMode === "auto"
                                      ? "View AUTO" : "View CENTER"
                                onClicked: {
                                    controlform.mapViewMode =
                                            controlform.mapViewMode === "auto"
                                            ? "centered" : "auto"
                                    controlform.mapZoom = 1.0
                                    routeCanvas.requestPaint()
                                }
                            }
                            RoboButton {
                                text: "Zoom +"
                                onClicked: {
                                    controlform.mapZoom = Math.min(
                                                8.0, controlform.mapZoom * 1.25)
                                    routeCanvas.requestPaint()
                                }
                            }
                            RoboButton {
                                text: "Zoom -"
                                onClicked: {
                                    controlform.mapZoom = Math.max(
                                                0.125, controlform.mapZoom / 1.25)
                                    routeCanvas.requestPaint()
                                }
                            }
                            RoboButton {
                                text: "Clear"
                                onClicked: {
                                    controlform.clearHistories()
                                    routeCanvas.requestPaint()
                                }
                            }
                            RoboButton {
                                text: controlform.historyVisible
                                      ? "History ON" : "History OFF"
                                onClicked: {
                                    controlform.historyVisible =
                                            !controlform.historyVisible
                                    if (!controlform.historyVisible)
                                        controlform.clearHistories()
                                    routeCanvas.requestPaint()
                                }
                            }
                        }
                    }
                }
                RoboButton {
                    Layout.preferredWidth: 100; Layout.preferredHeight: 100
                    text: "IMPLEMENT"
                    onClicked: implementPopup.open()
                    Popup {
                        id: implementPopup; x: -170; y: -180; focus: true
                        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent
                        ColumnLayout {
                            RoboButton { text: "transport"; onClicked: { guiNode.setImplementMode(0); implementPopup.close() } }
                            RoboButton { text: "headland"; onClicked: { guiNode.setImplementMode(1); implementPopup.close() } }
                            RoboButton { text: "working"; onClicked: { guiNode.setImplementMode(2); implementPopup.close() } }
                        }
                    }
                }
            }
        }

        ListModel {
            id: messages
            ListElement { level: "INFO"; message: "RoboSoft ROS 2 GUI" }
        }
        MessageView {
            id: messageView
            Layout.preferredHeight: 180
            Layout.fillWidth: true
            model: messages
            delegate: Text {
                required property string level
                required property string message
                text: level + "  " + message
                font.family: "Courier"; font.pixelSize: 15
                color: level === "INFO"
                       ? (window.darkTheme ? "#55cc66" : "#187b2d")
                       : (level === "WARN"
                          ? (window.darkTheme ? "yellow" : "#9a6700")
                          : (window.darkTheme ? "#ff5252" : "#b3261e"))
            }
        }
    }

    Connections {
        target: guiNode
        function onLidarLandmarksUpdated(points) {
            controlform.lidarLandmarks = points
            controlform.lidarObservations = []
            routeCanvas.requestPaint()
        }
        function onLidarObservationsUpdated(points) {
            var mapped = []
            if (controlform.odometryReceived) {
                var c = Math.cos(controlform.vehicleHeading)
                var s = Math.sin(controlform.vehicleHeading)
                for (var point of points) {
                    mapped.push({"x": controlform.vehicleX + c * point.x - s * point.y,
                                 "y": controlform.vehicleY + s * point.x + c * point.y})
                }
            }
            controlform.lidarObservations = mapped
            lidarExpiry.restart()
            routeCanvas.requestPaint()
        }
        function onTimerPerformanceUpdated(component, targetPeriodMs,
                                           actualPeriodMs, callbackDurationMs,
                                           marginMs, latenessMs,
                                           deadlineMisses) {
            if (component === "Main") {
                controlform.mainExecutionTimeMs = callbackDurationMs
                controlform.mainExecutionTimeReceived = true
            } else if (component === "NMPC") {
                controlform.nmpcExecutionTimeMs = callbackDurationMs
                controlform.nmpcExecutionTimeReceived = true
            }
        }
        function onSafetyStopChanged(lidarStop, emergencyStop) {
            controlform.lidarSafetyStop = lidarStop
            controlform.emergencyStopActive = emergencyStop
        }
        function onOdometryUpdated(x, y, heading, velocity) {
            controlform.odometryReceived = true
            controlform.vehicleX = x
            controlform.vehicleY = y
            controlform.vehicleHeading = heading
            controlform.vehicleSpeed = velocity
            if (controlform.historyVisible) {
                var history = controlform.vehicleHistory.slice()
                if (history.length === 0 ||
                        Math.hypot(x - history[history.length - 1].x,
                                   y - history[history.length - 1].y) > 0.25) {
                    history.push({"x": x, "y": y})
                    if (history.length > 2000)
                        history.shift()
                    controlform.vehicleHistory = history
                }
            }
            routeCanvas.requestPaint()
        }
        function onLateralErrorUpdated(error) {
            controlform.lateralError = error
            routeCanvas.requestPaint()
        }
        function onRouteUpdated(points) {
            controlform.routePoints = points
            routeCanvas.requestPaint()
        }
        function onActiveRouteUpdated(points) {
            controlform.activeRoutePoints = points
            routeCanvas.requestPaint()
        }
        function onRemoteOdometryUpdated(name, x, y, heading, velocity) {
            controlform.updateRemoteOdometry(name, x, y, heading, velocity)
            routeCanvas.requestPaint()
        }
        function onRemotePathUpdated(name, points) {
            controlform.updateRemotePath(name, points)
            routeCanvas.requestPaint()
        }
        function onGuidanceLinesChanged(names, activeIndex) {
            routeSelector.model = names
            routeSelector.currentIndex = activeIndex
        }
        function onDiagnosticUpdate(component, level, message) {
            messages.append({"level": level, "message": component + ": " + message})
            messageView.positionViewAtEnd()
        }
        function onRobotStateChanged(state, substate) {
            if (Number(state) === 1 && Number(substate) === 11)
                controlform.automaticMode = true
            else if (Number(state) === 1 &&
                     (Number(substate) === 9 || Number(substate) === 10))
                controlform.automaticMode = false
            controlform.running = Number(state) === 1 &&
                                  (Number(substate) === 10 || Number(substate) === 11)
        }
        function onRobotModeChanged(automatic) {
            controlform.automaticMode = automatic
        }
    }

    Connections {
        target: window
        function onDarkThemeChanged() { routeCanvas.requestPaint() }
    }
    Timer {
        id: lidarExpiry
        interval: 1000
        onTriggered: {
            controlform.lidarObservations = []
            routeCanvas.requestPaint()
        }
    }
}
