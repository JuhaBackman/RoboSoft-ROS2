// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

import QtQuick 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    id: window
    property bool darkTheme: true
    visible: true
    width: 1024
    height: 768
    title: guiNode.applicationTitle

    palette.window: darkTheme ? "#202124" : "#f5f6f7"
    palette.windowText: darkTheme ? "#f1f3f4" : "#202124"
    palette.base: darkTheme ? "#303134" : "#ffffff"
    palette.text: darkTheme ? "#f1f3f4" : "#202124"
    palette.button: darkTheme ? "#3c4043" : "#e4e7eb"
    palette.buttonText: darkTheme ? "#f1f3f4" : "#202124"
    palette.highlight: darkTheme ? "#5c6bc0" : "#3f51b5"
    palette.highlightedText: "#ffffff"

    Component.onCompleted: showFullScreen()

    SwipeView {
        id: swipeView
        anchors.fill: parent
        currentIndex: tabBar.currentIndex
        interactive: false

        Page1Form { }
        Page2Form { }
        Page3Form { }
    }

    footer: TabBar {
        id: tabBar
        currentIndex: swipeView.currentIndex
        TabButton { text: qsTr("Setup") }
        TabButton { text: qsTr("Control") }
        TabButton { text: qsTr("Debug") }
    }
}
