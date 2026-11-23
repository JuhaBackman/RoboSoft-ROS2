// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "task.hpp"
#include <tinyxml2.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>

namespace
{

bool testFormat(
  robosoft_core::Task::ExtensionFormat format,
  const char * path)
{
  robosoft_interfaces::msg::TaskData source;
  source.task_name = "Extension test";
  source.farm_name = "Example farm";
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
  point_a.altitude = 1.2344F;
  auto point_b = point_a;
  point_b.latitude = 60.2;
  point_b.longitude = 25.2;
  point_b.speed = 0.0F;
  point_b.designator = "end";
  point_b.altitude = -0.5006F;
  line.points = {point_a, point_b};
  source.guidance_lines.push_back(line);
  source.partfield_name = "Recorded field";
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
  std::ostringstream warnings;
  auto * previous_stderr = std::cerr.rdbuf(warnings.rdbuf());
  const bool saved = writer.saveToFile(path);
  std::cerr.rdbuf(previous_stderr);
  if (!saved) {return false;}
  if (format == robosoft_core::Task::ExtensionFormat::DESIGNATOR_JSON &&
    (warnings.str().find("LSG.B exceeds") == std::string::npos ||
    warnings.str().find("PNT.B exceeds") == std::string::npos))
  {
    return false;
  }

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
  const auto * allocation_stamp = allocation ?
    allocation->FirstChildElement("ASP") : nullptr;
  const auto * farm = root->FirstChildElement("FRM");
  if (!cluster_point || !group || !pattern || !line_string || !allocation ||
    !allocation_stamp || !farm ||
    task->FirstChildElement("GAL") || task->FirstChildElement("GSF") ||
    std::string(task->Attribute("A") ? task->Attribute("A") : "") != "TSK1" ||
    std::string(farm->Attribute("A") ? farm->Attribute("A") : "") != "FRM1" ||
    std::string(farm->Attribute("B") ? farm->Attribute("B") : "") !=
    source.farm_name ||
    std::string(partfield->Attribute("A") ? partfield->Attribute("A") : "") !=
    "PFD1" || partfield->Attribute("B") ||
    std::string(partfield->Attribute("C") ? partfield->Attribute("C") : "") !=
    source.partfield_name || partfield->Unsigned64Attribute("D", 1) != 0 ||
    std::string(partfield->Attribute("F") ? partfield->Attribute("F") : "") !=
    "FRM1" ||
    std::string(allocation_stamp->Attribute("A") ?
    allocation_stamp->Attribute("A") : "") != "1970-01-01T00:00:00Z" ||
    allocation_stamp->IntAttribute("D", 0) != 4 ||
    std::string(allocation->Attribute("A") ? allocation->Attribute("A") : "") !=
    group->Attribute("A"))
  {
    return false;
  }
  const auto * written_point_a = line_string->FirstChildElement("PNT");
  const auto * written_point_b = written_point_a ?
    written_point_a->NextSiblingElement("PNT") : nullptr;
  if (!written_point_a || !written_point_b ||
    written_point_a->Int64Attribute("E") != 1234 ||
    written_point_b->Int64Attribute("E") != -501)
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
         result.task_id == "TSK1" && result.farm_id == "FRM1" &&
         result.partfield_id == "PFD1" &&
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
         std::abs(loaded.points.front().altitude - 1.234F) < 1e-4F &&
         std::abs(loaded.points.back().altitude + 0.501F) < 1e-4F &&
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
  partfield->SetAttribute("C", "Standard field");
  partfield->SetAttribute("D", 0);
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
  auto * stamp = document.NewElement("ASP");
  stamp->SetAttribute("A", "2026-01-02T03:04:05Z");
  stamp->SetAttribute("D", 1);
  allocation->InsertEndChild(stamp);
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

bool testPointAltitudeSemantics(const char * input_path, const char * output_path)
{
  tinyxml2::XMLDocument document;
  auto * root = document.NewElement("ISO11783_TaskData");
  root->SetAttribute("VersionMajor", "4");
  root->SetAttribute("VersionMinor", "0");
  root->SetAttribute("DataTransferOrigin", 2);
  document.InsertEndChild(root);
  auto * partfield = document.NewElement("PFD");
  partfield->SetAttribute("A", "PFD7");
  partfield->SetAttribute("C", "Altitude field");
  partfield->SetAttribute("D", 0);
  root->InsertEndChild(partfield);
  auto * group = document.NewElement("GGP");
  group->SetAttribute("A", "GGP7");
  partfield->InsertEndChild(group);
  auto * pattern = document.NewElement("GPN");
  pattern->SetAttribute("A", "GPN7");
  pattern->SetAttribute("C", 3);
  group->InsertEndChild(pattern);
  auto * line = document.NewElement("LSG");
  line->SetAttribute("A", 5);
  pattern->InsertEndChild(line);
  const int64_t altitude_values[] = {1234, -567, 0};
  for (int index = 0; index < 4; ++index) {
    auto * point = document.NewElement("PNT");
    point->SetAttribute("A", index == 0 ? 6 : (index == 3 ? 7 : 9));
    point->SetAttribute("C", 60.0 + index * 0.0001);
    point->SetAttribute("D", 25.0 + index * 0.0001);
    if (index < 3) {point->SetAttribute("E", altitude_values[index]);}
    line->InsertEndChild(point);
  }
  auto * task = document.NewElement("TSK");
  task->SetAttribute("A", "TSK7");
  task->SetAttribute("B", "Altitude task");
  task->SetAttribute("E", "PFD7");
  task->SetAttribute("G", 1);
  root->InsertEndChild(task);
  auto * allocation = document.NewElement("GAN");
  allocation->SetAttribute("A", "GGP7");
  auto * stamp = document.NewElement("ASP");
  stamp->SetAttribute("A", "2026-02-03T04:05:06Z");
  stamp->SetAttribute("D", 1);
  allocation->InsertEndChild(stamp);
  task->InsertEndChild(allocation);
  if (document.SaveFile(input_path) != tinyxml2::XML_SUCCESS) {return false;}

  robosoft_core::Task parsed;
  if (!parsed.loadFromFile(input_path)) {return false;}
  const auto message = parsed.toMessage();
  if (message.guidance_lines.size() != 1 ||
    message.guidance_lines.front().points.size() != 4)
  {
    return false;
  }
  const auto & points = message.guidance_lines.front().points;
  if (std::abs(points[0].altitude - 1.234F) > 1e-5F ||
    std::abs(points[1].altitude + 0.567F) > 1e-5F ||
    points[2].altitude != 0.0F || points[3].altitude != 0.0F)
  {
    return false;
  }
  if (!parsed.saveToFile(output_path)) {return false;}

  tinyxml2::XMLDocument output;
  if (output.LoadFile(output_path) != tinyxml2::XML_SUCCESS) {return false;}
  const auto * output_partfield =
    output.FirstChildElement("ISO11783_TaskData")->FirstChildElement("PFD");
  const auto * output_task =
    output.FirstChildElement("ISO11783_TaskData")->FirstChildElement("TSK");
  const auto * output_allocation = output_task->FirstChildElement("GAN");
  const auto * output_stamp = output_allocation->FirstChildElement("ASP");
  if (std::string(output_stamp->Attribute("A")) !=
    "2026-02-03T04:05:06Z" || output_stamp->IntAttribute("D") != 1)
  {
    return false;
  }
  const auto * output_line = output_partfield->FirstChildElement("GGP")
    ->FirstChildElement("GPN")->FirstChildElement("LSG");
  const auto * output_point = output_line->FirstChildElement("PNT");
  for (int index = 0; index < 4; ++index) {
    if (!output_point) {return false;}
    if (index < 3 &&
      output_point->Int64Attribute("E", 9999) != altitude_values[index])
    {
      return false;
    }
    if (index == 3 && output_point->Attribute("E")) {return false;}
    output_point = output_point->NextSiblingElement("PNT");
  }
  return true;
}

bool testFarmPartfieldAndDeviceMappings(
  const char * input_path, const char * output_path)
{
  tinyxml2::XMLDocument document;
  auto * root = document.NewElement("ISO11783_TaskData");
  root->SetAttribute("VersionMajor", "4");
  root->SetAttribute("VersionMinor", "0");
  root->SetAttribute("DataTransferOrigin", 2);
  document.InsertEndChild(root);
  auto * farm = document.NewElement("FRM");
  farm->SetAttribute("A", "FRM9");
  farm->SetAttribute("B", "Mapped farm");
  farm->SetAttribute("D", "PO 123");
  farm->SetAttribute("H", "FI");
  root->InsertEndChild(farm);
  auto * device = document.NewElement("DVC");
  device->SetAttribute("A", "DVC9");
  device->SetAttribute("B", "Incomplete device");
  root->InsertEndChild(device);
  auto * partfield = document.NewElement("PFD");
  partfield->SetAttribute("A", "PFD9");
  partfield->SetAttribute("B", "FIELD-009");
  partfield->SetAttribute("C", "Mapped field");
  partfield->SetAttribute("D", 12345);
  partfield->SetAttribute("E", "CTR9");
  partfield->SetAttribute("F", "FRM9");
  root->InsertEndChild(partfield);
  auto * task = document.NewElement("TSK");
  task->SetAttribute("A", "TSK9");
  task->SetAttribute("B", "Mapped task");
  task->SetAttribute("D", "FRM9");
  task->SetAttribute("E", "PFD9");
  task->SetAttribute("G", 1);
  root->InsertEndChild(task);
  if (document.SaveFile(input_path) != tinyxml2::XML_SUCCESS) {return false;}

  robosoft_core::Task parsed;
  if (!parsed.loadFromFile(input_path)) {return false;}
  const auto farm_entry = parsed.farms.find("FRM9");
  const auto partfield_entry = parsed.partfields.find("PFD9");
  const auto message = parsed.toMessage();
  const robosoft_core::Task reconstructed(message);
  if (farm_entry == parsed.farms.end() ||
    farm_entry->second.farm_country != "FI" ||
    partfield_entry == parsed.partfields.end() ||
    partfield_entry->second.partfield_code != "FIELD-009" ||
    partfield_entry->second.partfield_designator != "Mapped field" ||
    partfield_entry->second.area != 12345 ||
    partfield_entry->second.customer_id_ref != "CTR9" ||
    partfield_entry->second.farm_id_ref != "FRM9" ||
    message.task_id != "TSK9" || message.farm_id != "FRM9" ||
    message.partfield_id != "PFD9" || reconstructed.task_id != "TSK9" ||
    reconstructed.current_farm_id != "FRM9" ||
    reconstructed.current_partfield_id != "PFD9")
  {
    return false;
  }
  if (!parsed.saveToFile(output_path)) {return false;}

  tinyxml2::XMLDocument output;
  if (output.LoadFile(output_path) != tinyxml2::XML_SUCCESS) {return false;}
  const auto * output_root = output.FirstChildElement("ISO11783_TaskData");
  const auto * output_farm = output_root->FirstChildElement("FRM");
  const auto * output_partfield = output_root->FirstChildElement("PFD");
  const auto * output_task = output_root->FirstChildElement("TSK");
  return output_root->FirstChildElement("DVC") == nullptr &&
         output_farm && output_partfield && output_task &&
         output_farm->Attribute("D") == nullptr &&
         std::string(output_farm->Attribute("H")) == "FI" &&
         std::string(output_partfield->Attribute("A")) == "PFD9" &&
         std::string(output_partfield->Attribute("B")) == "FIELD-009" &&
         std::string(output_partfield->Attribute("C")) == "Mapped field" &&
         output_partfield->Unsigned64Attribute("D") == 12345 &&
         std::string(output_partfield->Attribute("E")) == "CTR9" &&
         std::string(output_partfield->Attribute("F")) == "FRM9" &&
         std::string(output_task->Attribute("A")) == "TSK9";
}

bool testAltitudeRangeRejected(const char * output_path)
{
  robosoft_interfaces::msg::TaskData message;
  message.task_name = "Invalid altitude";
  message.partfield_name = "Range field";
  robosoft_interfaces::msg::GuidanceLine line;
  line.guidance_group_id = "GGP1";
  line.guidance_pattern_id = "GPN1";
  robosoft_interfaces::msg::RoutePoint first;
  first.latitude = 60.0;
  first.longitude = 25.0;
  first.altitude = 3000000.0F;
  auto second = first;
  second.latitude += 0.001;
  line.points = {first, second};
  message.guidance_lines.push_back(line);
  const robosoft_core::Task task(message);
  return !task.saveToFile(output_path);
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
  const char * altitude_input_path = "/tmp/robosoft_altitude_input.xml";
  const char * altitude_output_path = "/tmp/robosoft_altitude_output.xml";
  const char * metadata_input_path = "/tmp/robosoft_metadata_input.xml";
  const char * metadata_output_path = "/tmp/robosoft_metadata_output.xml";
  const char * invalid_altitude_path = "/tmp/robosoft_invalid_altitude.xml";
  const bool xml_ok = testFormat(
      robosoft_core::Task::ExtensionFormat::XML_ELEMENTS, xml_path);
  const bool json_ok = testFormat(
      robosoft_core::Task::ExtensionFormat::DESIGNATOR_JSON, json_path);
  const bool standard_ok = testStandardGuidanceInput(standard_path);
  const bool altitude_ok = testPointAltitudeSemantics(
    altitude_input_path, altitude_output_path);
  const bool metadata_ok = testFarmPartfieldAndDeviceMappings(
    metadata_input_path, metadata_output_path);
  const bool altitude_range_ok = testAltitudeRangeRejected(
    invalid_altitude_path);
  std::remove(xml_path);
  std::remove(json_path);
  std::remove(standard_path);
  std::remove(altitude_input_path);
  std::remove(altitude_output_path);
  std::remove(metadata_input_path);
  std::remove(metadata_output_path);
  std::remove(invalid_altitude_path);
  if (!xml_ok || !json_ok || !standard_ok || !altitude_ok || !metadata_ok ||
    !altitude_range_ok)
  {
    std::cerr << "Extension round-trip failed: xml=" << xml_ok
              << " json=" << json_ok << " standard=" << standard_ok
              << " altitude=" << altitude_ok
              << " metadata=" << metadata_ok
              << " altitude_range=" << altitude_range_ok << '\n';
    return 1;
  }
  return 0;
}
