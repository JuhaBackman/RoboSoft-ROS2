// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "task.hpp"
#include <tinyxml2.h>

#include <cmath>
#include <cstdio>
#include <iostream>

namespace
{

bool testFormat(
  robosoft_core::Task::ExtensionFormat format,
  const char * path)
{
  robosoft_interfaces::msg::TaskData source;
  source.task_name = "extension_test";
  source.status = robosoft_interfaces::msg::TaskData::STATUS_RUNNING;
  robosoft_interfaces::msg::GuidanceLine line;
  line.name = "recorded";
  line.designator = "work line";
  line.guidance_group_id = "GGP1";
  line.guidance_pattern_id = "GPN1";
  line.movement_type =
    robosoft_interfaces::msg::GuidanceLine::MOVEMENT_ROTATE;
  line.sequence = 4;
  line.driving_direction =
    robosoft_interfaces::msg::GuidanceLine::DIRECTION_REVERSE;

  robosoft_interfaces::msg::RoutePoint point_a;
  point_a.latitude = 60.1;
  point_a.longitude = 25.1;
  point_a.yaw = 1.25F;
  point_a.speed = -0.5F;
  point_a.implement_state =
    robosoft_interfaces::msg::RoutePoint::IMPLEMENT_WORKING;
  point_a.designator = "start";
  auto point_b = point_a;
  point_b.latitude = 60.2;
  point_b.longitude = 25.2;
  point_b.speed = 0.0F;
  point_b.designator = "end";
  line.points = {point_a, point_b};
  source.guidance_lines.push_back(line);
  source.partfield_name = "PFD1";
  robosoft_interfaces::msg::LidarCluster cluster;
  cluster.latitude = 60.15;
  cluster.longitude = 25.15;
  cluster.covariance_xx = 0.01;
  cluster.covariance_xy = 0.002;
  cluster.covariance_yx = 0.003;
  cluster.covariance_yy = 0.02;
  source.lidar_clusters.push_back(cluster);

  robosoft_core::Task writer(source);
  writer.setExtensionFormat(format);
  if (!writer.saveToFile(path)) {return false;}

  tinyxml2::XMLDocument document;
  if (document.LoadFile(path) != tinyxml2::XML_SUCCESS) {return false;}
  const auto * root = document.FirstChildElement("ISO11783_TaskData");
  const auto * task = root ? root->FirstChildElement("TSK") : nullptr;
  if (!root || !task || !root->Attribute("VersionMajor") ||
    task->IntAttribute("G", -1) !=
    robosoft_interfaces::msg::TaskData::STATUS_RUNNING)
  {
    return false;
  }
  const auto * partfield = root->FirstChildElement("PFD");
  const auto * cluster_point = partfield ?
    partfield->FirstChildElement("PNT") : nullptr;
  const auto * group = partfield ? partfield->FirstChildElement("GGP") : nullptr;
  const auto * pattern = group ? group->FirstChildElement("GPN") : nullptr;
  const auto * line_string = pattern ? pattern->FirstChildElement("LSG") : nullptr;
  const auto * allocation = task->FirstChildElement("GAN");
  if (!cluster_point || !group || !pattern || !line_string || !allocation ||
    task->FirstChildElement("GAL") || task->FirstChildElement("GSF") ||
    std::string(allocation->Attribute("A") ? allocation->Attribute("A") : "") !=
    group->Attribute("A"))
  {
    return false;
  }
  if (format == robosoft_core::Task::ExtensionFormat::XML_ELEMENTS &&
    (!cluster_point->FirstChildElement("RoboSoftExtension") ||
    cluster_point->Attribute("RoboSoftCovarianceXY")))
  {
    return false;
  }

  robosoft_core::Task reader;
  if (!reader.loadFromFile(path)) {return false;}
  const auto result = reader.toMessage();
  if (result.guidance_lines.size() != 1) {return false;}
  const auto & loaded = result.guidance_lines.front();
  return result.status == source.status &&
         loaded.designator == line.designator &&
         loaded.guidance_group_id == line.guidance_group_id &&
         loaded.guidance_pattern_id == line.guidance_pattern_id &&
         loaded.movement_type == line.movement_type &&
         loaded.sequence == line.sequence &&
         loaded.driving_direction == line.driving_direction &&
         loaded.points.size() == 2 &&
         loaded.points.front().designator == point_a.designator &&
         loaded.points.front().speed == point_a.speed &&
         loaded.points.front().implement_state == point_a.implement_state &&
         result.lidar_clusters.size() == 1 &&
         result.lidar_clusters.front().latitude == cluster.latitude &&
         result.lidar_clusters.front().longitude == cluster.longitude &&
         result.lidar_clusters.front().covariance_xx ==
         cluster.covariance_xx &&
         result.lidar_clusters.front().covariance_xy ==
         cluster.covariance_xy &&
         result.lidar_clusters.front().covariance_yx ==
         cluster.covariance_yx &&
         result.lidar_clusters.front().covariance_yy ==
         cluster.covariance_yy;
}

bool testStandardGuidanceInput(const char * path)
{
  tinyxml2::XMLDocument document;
  auto * root = document.NewElement("ISO11783_TaskData");
  root->SetAttribute("VersionMajor", "4");
  root->SetAttribute("VersionMinor", "0");
  root->SetAttribute("DataTransferOrigin", 2);
  document.InsertEndChild(root);

  auto * partfield = document.NewElement("PFD");
  partfield->SetAttribute("A", "PFD9");
  partfield->SetAttribute("B", "Standard field");
  root->InsertEndChild(partfield);
  auto * group = document.NewElement("GGP");
  group->SetAttribute("A", "GGP9");
  partfield->InsertEndChild(group);
  auto * pattern = document.NewElement("GPN");
  pattern->SetAttribute("A", "GPN9");
  pattern->SetAttribute("B", "Standard curve");
  pattern->SetAttribute("C", 3);
  group->InsertEndChild(pattern);
  auto * line = document.NewElement("LSG");
  line->SetAttribute("A", 5);
  pattern->InsertEndChild(line);
  for (int index = 0; index < 2; ++index) {
    auto * point = document.NewElement("PNT");
    point->SetAttribute("A", index == 0 ? 6 : 7);
    point->SetAttribute("C", 60.1 + 0.1 * index);
    point->SetAttribute("D", 25.1 + 0.1 * index);
    line->InsertEndChild(point);
  }

  auto * task = document.NewElement("TSK");
  task->SetAttribute("A", "TSK9");
  task->SetAttribute("B", "Standard task");
  task->SetAttribute("E", "PFD9");
  task->SetAttribute("G", 1);
  root->InsertEndChild(task);
  auto * allocation = document.NewElement("GAN");
  allocation->SetAttribute("A", "GGP9");
  task->InsertEndChild(allocation);
  if (document.SaveFile(path) != tinyxml2::XML_SUCCESS) {return false;}

  robosoft_core::Task parsed;
  if (!parsed.loadFromFile(path)) {return false;}
  const auto result = parsed.toMessage();
  return result.task_name == "Standard task" &&
         result.partfield_name == "Standard field" &&
         result.guidance_lines.size() == 1 &&
         result.active_guidance_index == 0 &&
         result.guidance_lines.front().guidance_group_id == "GGP9" &&
         result.guidance_lines.front().guidance_pattern_id == "GPN9" &&
         result.guidance_lines.front().points.size() == 2 &&
         result.guidance_lines.front().points.front().latitude == 60.1 &&
         result.guidance_lines.front().points.front().longitude == 25.1;
}

}  // namespace

int main()
{
  robosoft_core::Task empty_template;
  const std::string empty_template_path = ROBOSOFT_EMPTY_TASK_PATH;
  if (!empty_template.loadFromFile(empty_template_path)) {
    std::cerr << "Empty task template could not be loaded\n";
    return 1;
  }
  const auto empty_task = empty_template.toMessage();
  if (empty_task.task_name != "New recorded task" ||
    empty_task.status != robosoft_interfaces::msg::TaskData::STATUS_PLANNED ||
    empty_task.active_guidance_index != -1 ||
    !empty_task.guidance_lines.empty())
  {
    std::cerr << "Empty task template contents are invalid\n";
    return 1;
  }

  for (const char * filename : {"TASKDATA.XML"}) {
    robosoft_core::Task packaged_task;
    const std::string path = std::string(ROBOSOFT_TASK_DIRECTORY) + "/" + filename;
    if (!packaged_task.loadFromFile(path) ||
      packaged_task.toMessage().guidance_lines.empty())
    {
      std::cerr << "Packaged standard task could not be loaded: " << filename << '\n';
      return 1;
    }
  }

  const char * xml_path = "/tmp/robosoft_extension_xml.xml";
  // The example pole map has A=5 points without RoboSoft extension metadata.
  robosoft_core::Task pole_map;
  if (!pole_map.loadFromFile(std::string(ROBOSOFT_TASK_DIRECTORY) + "/metsapelto.XML") ||
    pole_map.toMessage().lidar_clusters.empty())
  {
    std::cerr << "Plain TASK obstacle points were not loaded as landmarks\n";
    return 1;
  }
  const auto field_task = pole_map.toMessage();
  if (field_task.guidance_lines.size() != 11 ||
    field_task.guidance_lines.front().points.empty())
  {
    std::cerr << "Packaged field TASK routes are invalid\n";
    return 1;
  }
  const auto & first_route_point =
    field_task.guidance_lines.front().points.front();
  if (std::abs(first_route_point.yaw - 8.471420F) > 1e-5F ||
    std::abs(first_route_point.speed - 0.5F) > 1e-5F)
  {
    std::cerr << "Packaged field TASK route metadata is invalid\n";
    return 1;
  }
  const char * json_path = "/tmp/robosoft_extension_json.xml";
  const char * standard_path = "/tmp/robosoft_standard_guidance.xml";
  const bool xml_ok = testFormat(
      robosoft_core::Task::ExtensionFormat::XML_ELEMENTS, xml_path);
  const bool json_ok = testFormat(
      robosoft_core::Task::ExtensionFormat::DESIGNATOR_JSON, json_path);
  const bool standard_ok = testStandardGuidanceInput(standard_path);
  std::remove(xml_path);
  std::remove(json_path);
  std::remove(standard_path);
  if (!xml_ok || !json_ok || !standard_ok) {
    std::cerr << "Extension round-trip failed: xml=" << xml_ok
              << " json=" << json_ok << " standard=" << standard_ok << '\n';
    return 1;
  }
  return 0;
}
