// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <QGuiApplication>
#include <QtPlugin>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QByteArray>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include "gui_main_node.hpp"

#ifdef ROBOSOFT_STATIC_LINUXFB_PLUGIN
// Static Qt platform plugins must be registered explicitly. Linking the
// archive alone leaves the plugin code in the executable but does not make it
// visible to QGuiApplication's platform-plugin loader.
Q_IMPORT_PLUGIN(QLinuxFbIntegrationPlugin)
#endif

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);          // ROS2 init
  const rclcpp::NodeOptions node_options;
  auto node = std::make_shared<rclcpp::Node>("robosoft_gui_node", node_options);

    // The GUI is intentionally simple and must also run on virtual machines
    // and framebuffer targets without a working EGL/OpenGL stack. A caller can
    // override this with e.g. QT_QUICK_BACKEND=rhi on accelerated hardware.
  if (qEnvironmentVariableIsEmpty("QT_QUICK_BACKEND")) {
    qputenv("QT_QUICK_BACKEND", QByteArray("software"));
  }

    // Wayland provides the input method through the compositor. On X11 and
    // framebuffer targets Qt Virtual Keyboard is used client-side.
  const bool is_wayland =
    qEnvironmentVariable("XDG_SESSION_TYPE").compare(
            QStringLiteral("wayland"), Qt::CaseInsensitive) == 0 ||
    !qEnvironmentVariable("WAYLAND_DISPLAY").isEmpty();
  if (!is_wayland) {
    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));
  }
  QGuiApplication app(argc, argv);
  // Basic avoids style-specific ripple layers whose scene coordinates can be
  // wrong with Qt's software renderer on virtual machines/framebuffers.
  QQuickStyle::setStyle(QStringLiteral("Basic"));
  robosoft_core::GuiNode guiNode(node);

  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty("guiNode", &guiNode);
  const QUrl url(QStringLiteral("qrc:/qml/Main.qml"));
  QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl & objUrl) {
      if (!obj && url == objUrl) {
        QCoreApplication::exit(-1);
      }
                     }, Qt::QueuedConnection);
  engine.load(url);

  if (engine.rootObjects().isEmpty()) {
    rclcpp::shutdown();
    return -1;
  }

    // rclcpp handles SIGINT/SIGTERM. Mirror that shutdown into the Qt event
    // loop so Ctrl-C terminates both loops instead of leaving app.exec() alive.
  QTimer ros_shutdown_timer;
  QObject::connect(&ros_shutdown_timer, &QTimer::timeout, &app, [&app]() {
      if (!rclcpp::ok()) {
        app.quit();
      }
    });
  ros_shutdown_timer.start(100);

    // Käynnistä ROS2 spinnin rinnalla (yksinkertainen tapa: eri thread)
  std::thread ros_spin_thread([&]() {
      rclcpp::spin(node);
    });

  int ret = app.exec();    // Qt event loop
  rclcpp::shutdown();
  ros_spin_thread.join();
  return ret;
}
