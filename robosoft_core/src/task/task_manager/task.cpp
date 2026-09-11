// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "task.hpp"
#include <tinyxml2.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <regex>

namespace robosoft_core
{

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace
{

std::string jsonEscape(const std::string & value)
{
  std::string result;
  for (const char c : value) {
    if (c == '"' || c == '\\') {result.push_back('\\');}
    result.push_back(c);
  }
  return result;
}

double jsonNumber(
  const std::string & json, const std::string & key,
  double fallback)
{
  const std::regex expression(
    "\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  std::smatch match;
  return std::regex_search(json, match, expression) ?
         std::stod(match[1].str()) :
         fallback;
}

std::string jsonString(
  const std::string & json, const std::string & key,
  const std::string & fallback = "")
{
  const std::regex expression("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch match;
  return std::regex_search(json, match, expression) ? match[1].str() :
         fallback;
}

std::string lineExtensionJson(
  const robosoft_interfaces::msg::GuidanceLine & line)
{
  std::ostringstream json;
  json << "{\"label\":\"" << jsonEscape(line.designator)
       << "\",\"group\":\"" << jsonEscape(line.guidance_group_id)
       << "\",\"pattern\":\"" << jsonEscape(line.guidance_pattern_id)
       << "\",\"movement\":" << static_cast<int>(line.movement_type)
       << ",\"sequence\":" << line.sequence
       << ",\"direction\":" << static_cast<int>(line.driving_direction)
       << "}";
  return json.str();
}

std::string pointExtensionJson(
  const robosoft_interfaces::msg::RoutePoint & point)
{
  std::ostringstream json;
  json << "{\"label\":\"" << jsonEscape(point.designator)
       << "\",\"yaw\":" << point.yaw << ",\"speed\":" << point.speed
       << ",\"implement\":" << static_cast<int>(point.implement_state)
       << "}";
  return json.str();
}

std::string clusterExtensionJson(
  const robosoft_interfaces::msg::LidarCluster & cluster)
{
  std::ostringstream json;
  json << "{\"type\":\"robosoft_lidar_cluster\""
       << ",\"xx\":" << cluster.covariance_xx
       << ",\"xy\":" << cluster.covariance_xy
       << ",\"yx\":" << cluster.covariance_yx
       << ",\"yy\":" << cluster.covariance_yy << "}";
  return json.str();
}

}  // namespace

// ============================================================================
// Constructor & Destructor
// ============================================================================

Task::Task()
: version_major("4"),
  version_minor("0"),
  management_software_manufacturer("RoboSoft"),
  management_software_version("1.0"),
  task_controller_manufacturer("RoboSoft"),
  task_controller_version("1.0"),
  current_farm_id(""),
  current_partfield_id(""),
  task_name(""),
  task_id("TSK1"),
  task_designator(""),
  creation_timestamp(0),
  task_status(robosoft_interfaces::msg::TaskData::STATUS_PLANNED) {}

Task::Task(const robosoft_interfaces::msg::TaskData & msg)
{
  fromMessage(msg);
}

// ============================================================================
// File I/O
// ============================================================================

bool Task::loadFromFile(const std::string & filepath)
{
  XMLDocument doc;
  if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to load XML file: " << filepath << std::endl;
    return false;
  }

  XMLElement * root = doc.RootElement();
  if (!root) {
    std::cerr << "No root element found" << std::endl;
    return false;
  }

  const std::string root_name = root->Name();
  if (root_name != "ISO11783_TaskData") {
    std::cerr << "Invalid ISO11783 task file format, got: " << root_name << std::endl;
    return false;
  }

  if (const char * value = root->Attribute("VersionMajor")) {version_major = value;}
  if (const char * value = root->Attribute("VersionMinor")) {version_minor = value;}
  if (const char * value = root->Attribute("ManagementSoftwareManufacturer")) {
    management_software_manufacturer = value;
  }
  if (const char * value = root->Attribute("ManagementSoftwareVersion")) {
    management_software_version = value;
  }
  if (const char * value = root->Attribute("TaskControllerManufacturer")) {
    task_controller_manufacturer = value;
  }
  if (const char * value = root->Attribute("TaskControllerVersion")) {
    task_controller_version = value;
  }

  // ISO reference objects are root children.
  for (XMLElement * child = root->FirstChildElement(); child;
    child = child->NextSiblingElement())
  {
    const std::string tag = child->Name();
    if (tag == "FRM") {parseFarm(child);}
    else if (tag == "CTR") {parseCustomer(child);}
    else if (tag == "PDT") {parseProduct(child);}
    else if (tag == "DVC") {parseDevice(child);}
    else if (tag == "PFD") {parsePartfield(child);}
    else if (tag == "OTQ") {parseOperationTechnique(child);}
    else if (tag == "CPC") {parseCulturalPractice(child);}
  }

  XMLElement * task_elem = root->FirstChildElement("TSK");
  if (!task_elem) {
    std::cerr << "No TSK element found" << std::endl;
    return false;
  }

  return parseRoot(task_elem);
}

bool Task::saveToFile(const std::string & filepath) const
{
  XMLDocument doc;

  XMLElement * root = doc.NewElement("ISO11783_TaskData");
  root->SetAttribute("VersionMajor", version_major.c_str());
  root->SetAttribute("VersionMinor", version_minor.c_str());
  root->SetAttribute(
    "ManagementSoftwareManufacturer",
    management_software_manufacturer.c_str());
  root->SetAttribute(
    "ManagementSoftwareVersion", management_software_version.c_str());
  root->SetAttribute(
    "TaskControllerManufacturer", task_controller_manufacturer.c_str());
  root->SetAttribute(
    "TaskControllerVersion", task_controller_version.c_str());
  root->SetAttribute("DataTransferOrigin", 2);  // MICS
  doc.InsertFirstChild(root);

  buildFarms(root);
  buildCustomers(root);
  buildProducts(root);
  buildDevices(root);
  buildPartfields(root);
  buildOperationTechniques(root);
  buildCulturalPractices(root);

  XMLElement * task_elem = doc.NewElement("TSK");
  root->InsertEndChild(task_elem);

  buildRoot(task_elem);

  if (doc.SaveFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to save XML file: " << filepath << std::endl;
    return false;
  }

  return true;
}

// ============================================================================
// Message Conversion
// ============================================================================

robosoft_interfaces::msg::TaskData Task::toMessage() const
{
  robosoft_interfaces::msg::TaskData msg;
  msg.task_name = task_name;
  msg.farm_name = current_farm_id;
  const auto partfield = partfields.find(current_partfield_id);
  msg.partfield_name = partfield == partfields.end() ?
    current_partfield_id : partfield->second.partfield_name;
  msg.version = version_major + "." + version_minor;
  msg.timestamp = creation_timestamp;
  msg.guidance_lines = guidance_allocations;
  msg.lidar_clusters = lidar_clusters;
  msg.active_guidance_index = -1;
  for (size_t i = 0; i < guidance_allocations.size(); ++i) {
    if (guidance_allocations[i].is_active) {
      msg.active_guidance_index = static_cast<int32_t>(i);
      break;
    }
  }
  if (msg.active_guidance_index < 0 && !guidance_allocations.empty()) {
    msg.active_guidance_index = 0;
  }
  msg.status = task_status;
  return msg;
}

void Task::fromMessage(const robosoft_interfaces::msg::TaskData & msg)
{
  task_name = msg.task_name;
  current_farm_id = msg.farm_name;
  current_partfield_id = msg.partfield_name;
  version_major = "4";
  version_minor = "0";
  creation_timestamp = msg.timestamp;
  guidance_allocations = msg.guidance_lines;
  lidar_clusters = msg.lidar_clusters;
  if (partfields.find(current_partfield_id) == partfields.end()) {
    Partfield partfield;
    partfield.partfield_id =
      current_partfield_id.empty() ? "PFD1" : current_partfield_id;
    partfield.partfield_name = msg.partfield_name;
    current_partfield_id = partfield.partfield_id;
    partfields[partfield.partfield_id] = partfield;
  }
  task_status = msg.status <= robosoft_interfaces::msg::TaskData::STATUS_CANCELED ?
    msg.status : robosoft_interfaces::msg::TaskData::STATUS_NOT_SET;
}

// ============================================================================
// XML Parsing - Root & Metadata
// ============================================================================

bool Task::parseRoot(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);

  if (const char * value = elem->Attribute("A")) {task_id = value;}
  if (const char * value = elem->Attribute("B")) {
    task_designator = value;
    task_name = value;
  }
  if (const char * value = elem->Attribute("D")) {current_farm_id = value;}
  if (const char * value = elem->Attribute("E")) {current_partfield_id = value;}
  int parsed_status = static_cast<int>(task_status);
  if (elem->QueryIntAttribute("G", &parsed_status) == tinyxml2::XML_SUCCESS &&
    parsed_status >= robosoft_interfaces::msg::TaskData::STATUS_NOT_SET &&
    parsed_status <= robosoft_interfaces::msg::TaskData::STATUS_CANCELED)
  {
    task_status = static_cast<uint8_t>(parsed_status);
  }

  bool allocation_valid = true;
  // GAN allocates a root-level PFD/GGP guidance group to this task.
  XMLElement * child = elem->FirstChildElement();
  while (child) {
    std::string tag_name = child->Name();

    if (tag_name == "GAN") {
      allocation_valid = parseGuidanceAllocation(child) && allocation_valid;
    }

    child = child->NextSiblingElement();
  }

  return allocation_valid;
}

std::string Task::getAttribute(void * xml_elem, const std::string & attr_name)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  const char * value = elem->Attribute(attr_name.c_str());
  return value ? std::string(value) : "";
}

double Task::getAttributeDouble(void * xml_elem, const std::string & attr_name)
{
  std::string value = getAttribute(xml_elem, attr_name);
  if (value.empty()) {return 0.0;}
  // Replace comma with period for locale compatibility
  size_t pos = value.find(',');
  if (pos != std::string::npos) {
    value[pos] = '.';
  }
  return std::stod(value);
}

int64_t Task::getAttributeInt64(void * xml_elem, const std::string & attr_name)
{
  std::string value = getAttribute(xml_elem, attr_name);
  return value.empty() ? 0 : std::stoll(value);
}

// ============================================================================
// XML Parsing - Farms, Customers, Products, Devices
// ============================================================================

bool Task::parseFarm(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  Farm farm;

  if (const char * id = elem->Attribute("A")) {
    farm.farm_id = id;
  }
  if (const char * name = elem->Attribute("B")) {
    farm.farm_name = name;
  }
  if (const char * code = elem->Attribute("D")) {
    farm.country_code = code;
  }

  current_farm_id = farm.farm_id;
  farms[farm.farm_id] = farm;
  return true;
}

bool Task::parseCustomer(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  Customer customer;

  if (const char * id = elem->Attribute("A")) {
    customer.customer_id = id;
  }
  if (const char * name = elem->Attribute("B")) {
    customer.customer_name = name;
  }

  customers[customer.customer_id] = customer;
  return true;
}

bool Task::parseProduct(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  Product product;

  if (const char * id = elem->Attribute("A")) {
    product.product_id = id;
  }
  if (const char * name = elem->Attribute("B")) {
    product.product_name = name;
  }

  products[product.product_id] = product;
  return true;
}

bool Task::parseDevice(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  Device device;

  if (const char * id = elem->Attribute("A")) {
    device.device_id = id;
  }
  if (const char * name = elem->Attribute("B")) {
    device.device_name = name;
  }

  devices[device.device_id] = device;
  return true;
}

// ============================================================================
// XML Parsing - Partfield
// ============================================================================

bool Task::parsePartfield(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  Partfield partfield;

  if (const char * id = elem->Attribute("A")) {
    partfield.partfield_id = id;
  }
  if (const char * name = elem->Attribute("B")) {
    partfield.partfield_name = name;
  }

  current_partfield_id = partfield.partfield_id;

  // Parse child polygons
  XMLElement * child = elem->FirstChildElement();
  while (child) {
    std::string tag = child->Name();
    if (tag == "PLN") {
      parsePolygon(child, partfield.boundary_polygon);
    } else if (tag == "GGP") {
      parseGuidanceGroup(child);
    } else if (tag == "PNT" && getAttribute(child, "A") == "5") {
      robosoft_interfaces::msg::LidarCluster cluster;
      cluster.latitude = getAttributeDouble(child, "C");
      cluster.longitude = getAttributeDouble(child, "D");
      const std::string designator = getAttribute(child, "B");
      // A=5 point obstacles are landmarks even without optional covariance
      // metadata. Their C/D coordinates are sufficient for map visualization
      // and lidar association, just as in the simulator's TASK map.
      if (!designator.empty() && designator.front() == '{' &&
        jsonString(designator, "type") == "robosoft_lidar_cluster")
      {
        cluster.covariance_xx = jsonNumber(designator, "xx", 0.0);
        cluster.covariance_xy = jsonNumber(designator, "xy", 0.0);
        cluster.covariance_yx = jsonNumber(designator, "yx", 0.0);
        cluster.covariance_yy = jsonNumber(designator, "yy", 0.0);
      } else {
        const XMLElement * extension =
          child->FirstChildElement("RoboSoftExtension");
        const char * extension_type = extension ?
          extension->Attribute("type") : nullptr;
        if (extension_type && std::string(extension_type) == "lidar_cluster")
        {
          cluster.covariance_xx = extension->DoubleAttribute("xx");
          cluster.covariance_xy = extension->DoubleAttribute("xy");
          cluster.covariance_yx = extension->DoubleAttribute("yx");
          cluster.covariance_yy = extension->DoubleAttribute("yy");
        }
      }
      lidar_clusters.push_back(cluster);
    }
    child = child->NextSiblingElement();
  }

  partfields[partfield.partfield_id] = partfield;
  return true;
}

// ============================================================================
// XML Parsing - Guidance
// ============================================================================

bool Task::parseGuidanceGroup(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  GuidanceGroup group;
  group.guidance_group_id = getAttribute(elem, "A");
  group.guidance_group_designator = getAttribute(elem, "B");
  if (group.guidance_group_id.empty()) {return false;}

  for (XMLElement * pattern = elem->FirstChildElement("GPN"); pattern;
    pattern = pattern->NextSiblingElement("GPN"))
  {
    if (!parseGuidancePattern(pattern)) {continue;}
    const std::string pattern_id = getAttribute(pattern, "A");
    auto found = guidance_patterns.find(pattern_id);
    if (found == guidance_patterns.end()) {continue;}
    found->second.guidance_group_id = group.guidance_group_id;
    found->second.line.guidance_group_id = group.guidance_group_id;
    group.guidance_pattern_ids.push_back(pattern_id);
  }
  guidance_groups[group.guidance_group_id] = group;
  return !group.guidance_pattern_ids.empty();
}

bool Task::parseGuidancePattern(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  GuidancePattern pattern;
  pattern.guidance_pattern_id = getAttribute(elem, "A");
  pattern.guidance_pattern_designator = getAttribute(elem, "B");
  if (pattern.guidance_pattern_id.empty()) {return false;}

  XMLElement * line_string = elem->FirstChildElement("LSG");
  if (!line_string) {return false;}
  auto & line = pattern.line;
  line.guidance_pattern_id = pattern.guidance_pattern_id;
  line.name = pattern.guidance_pattern_designator;
  line.designator = getAttribute(line_string, "B");
  if (line.designator.empty()) {line.designator = pattern.guidance_pattern_designator;}
  line.target_speed = 1.0F;
  line.line_type = robosoft_interfaces::msg::GuidanceLine::LINE_HEADLAND;

  const std::string designator = line.designator;
  if (!designator.empty() && designator.front() == '{') {
    line.designator = jsonString(designator, "label");
    line.guidance_group_id = jsonString(designator, "group");
    line.guidance_pattern_id = jsonString(
      designator, "pattern", pattern.guidance_pattern_id);
    line.movement_type = static_cast<uint8_t>(
      jsonNumber(designator, "movement", 0));
    line.sequence = static_cast<uint32_t>(jsonNumber(designator, "sequence", 0));
    line.driving_direction = static_cast<int8_t>(
      jsonNumber(designator, "direction", 0));
  }
  if (auto * extension = line_string->FirstChildElement("RoboSoftExtension")) {
    line.guidance_group_id = getAttribute(extension, "group");
    const auto extension_pattern = getAttribute(extension, "pattern");
    if (!extension_pattern.empty()) {line.guidance_pattern_id = extension_pattern;}
    line.movement_type = static_cast<uint8_t>(
      extension->UnsignedAttribute("movement", 0));
    line.sequence = extension->UnsignedAttribute("sequence", 0);
    line.driving_direction = static_cast<int8_t>(extension->IntAttribute("direction", 0));
  }
  if (!parseLineString(line_string, line.points)) {return false;}
  if (!line.points.empty()) {line.target_speed = line.points.front().speed;}
  guidance_patterns[pattern.guidance_pattern_id] = pattern;
  return true;
}

bool Task::parseGuidanceAllocation(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  const std::string group_id = getAttribute(elem, "A");
  const auto group = guidance_groups.find(group_id);
  if (group == guidance_groups.end()) {return false;}
  bool allocated = false;
  for (const auto & pattern_id : group->second.guidance_pattern_ids) {
    const auto pattern = guidance_patterns.find(pattern_id);
    if (pattern == guidance_patterns.end()) {continue;}
    auto line = pattern->second.line;
    line.guidance_group_id = group_id;
    line.guidance_pattern_id = pattern_id;
    line.is_active = guidance_allocations.empty();
    guidance_allocations.push_back(line);
    allocated = true;
  }
  return allocated;
}

// ============================================================================
// XML Parsing - Geometry (Polygon, LineString, Point)
// ============================================================================

bool Task::parsePolygon(void * xml_elem, std::vector<geometry_msgs::msg::Point32> & points)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);

  XMLElement * exterior = elem->FirstChildElement("EXT");
  if (!exterior) {
    return false;
  }

  XMLElement * line_string = exterior->FirstChildElement("LSG");
  if (!line_string) {
    return false;
  }

  XMLElement * point_elem = line_string->FirstChildElement("PNT");
  while (point_elem) {
    geometry_msgs::msg::Point32 point;
    point.y = getAttributeDouble(point_elem, "C");  // north/latitude
    point.x = getAttributeDouble(point_elem, "D");  // east/longitude
    points.push_back(point);
    point_elem = point_elem->NextSiblingElement("PNT");
  }

  return !points.empty();
}

bool Task::parseLineString(
  void * xml_elem,
  std::vector<robosoft_interfaces::msg::RoutePoint> & points)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);

  XMLElement * point_elem = elem->FirstChildElement("PNT");
  while (point_elem) {
    robosoft_interfaces::msg::RoutePoint point;
    if (parsePoint(point_elem, point)) {
      points.push_back(point);
    }
    point_elem = point_elem->NextSiblingElement("PNT");
  }

  return !points.empty();
}

bool Task::parsePoint(void * xml_elem, robosoft_interfaces::msg::RoutePoint & point)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);

  // ISO 11783-10: A=type, B=designator, C=north, D=east, E=up.
  if (!elem->Attribute("C") || !elem->Attribute("D")) {return false;}
  point.designator = getAttribute(elem, "B");
  point.latitude = getAttributeDouble(elem, "C");
  point.longitude = getAttributeDouble(elem, "D");
  point.altitude = getAttributeDouble(elem, "E");
  point.speed = 1.0;  // Default speed
  point.yaw = 0.0;
  point.implement_state =
    robosoft_interfaces::msg::RoutePoint::IMPLEMENT_TRANSPORT;

  if (!point.designator.empty() && point.designator.front() == '{') {
    const auto json = point.designator;
    point.designator = jsonString(json, "label");
    point.yaw = static_cast<float>(jsonNumber(json, "yaw", 0.0));
    point.speed = static_cast<float>(jsonNumber(json, "speed", 1.0));
    point.implement_state = static_cast<uint8_t>(
      jsonNumber(json, "implement", 0.0));
  }
  if (const auto * extension =
    elem->FirstChildElement("RoboSoftExtension"))
  {
    point.yaw = extension->FloatAttribute("yaw", point.yaw);
    point.speed = extension->FloatAttribute("speed", point.speed);
    point.implement_state = static_cast<uint8_t>(
      extension->UnsignedAttribute("implement", point.implement_state));
  }

  return  point.latitude != 0.0 || point.longitude != 0.0;
}

bool Task::parseExterior(void * xml_elem, std::vector<geometry_msgs::msg::Point32> & points)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);

  XMLElement * line_string = elem->FirstChildElement("LSG");
  if (!line_string) {
    return false;
  }

  XMLElement * point_elem = line_string->FirstChildElement("PNT");
  while (point_elem) {
    geometry_msgs::msg::Point32 point;
    point.y = getAttributeDouble(point_elem, "C");  // north/latitude
    point.x = getAttributeDouble(point_elem, "D");  // east/longitude
    points.push_back(point);
    point_elem = point_elem->NextSiblingElement("PNT");
  }

  return !points.empty();
}

// ============================================================================
// XML Parsing - Operation Techniques & Cultural Practices
// ============================================================================

bool Task::parseOperationTechnique(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  OperationTechnique op_tech;

  if (const char * id = elem->Attribute("A")) {
    op_tech.operation_technique_id = std::string(id);
    if (op_tech.operation_technique_id.substr(0, 3) == "OTQ") {
      op_tech.operation_technique_id = op_tech.operation_technique_id.substr(3);
    }
  }
  if (const char * name = elem->Attribute("B")) {
    op_tech.operation_technique_designator = name;
  }

  operation_techniques[op_tech.operation_technique_id] = op_tech;
  return true;
}

bool Task::parseCulturalPractice(void * xml_elem)
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  CulturalPractice cultural_practice;

  if (const char * id = elem->Attribute("A")) {
    cultural_practice.cultural_practice_id = id;
  }
  if (const char * name = elem->Attribute("B")) {
    cultural_practice.cultural_practice_designator = name;
  }

  cultural_practices[cultural_practice.cultural_practice_id] = cultural_practice;
  return true;
}

// ============================================================================
// XML Building - Root & Metadata
// ============================================================================

void Task::buildRoot(void * xml_elem) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);

  elem->SetAttribute("A", task_id.empty() ? "TSK1" : task_id.c_str());
  const auto & designator = task_designator.empty() ? task_name : task_designator;
  if (!designator.empty()) {elem->SetAttribute("B", designator.c_str());}
  if (!current_farm_id.empty()) {elem->SetAttribute("D", current_farm_id.c_str());}
  if (!current_partfield_id.empty()) {
    elem->SetAttribute("E", current_partfield_id.c_str());
  }
  elem->SetAttribute("G", task_status);
  buildGuidanceAllocations(elem);
}

// ============================================================================
// XML Building - Farms, Customers, Products, Devices
// ============================================================================

void Task::buildFarms(void * xml_elem) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  for (const auto & farm_pair : farms) {
    const Farm & farm = farm_pair.second;
    XMLElement * farm_elem = doc->NewElement("FRM");
    farm_elem->SetAttribute("A", farm.farm_id.c_str());
    farm_elem->SetAttribute("B", farm.farm_name.c_str());
    elem->InsertEndChild(farm_elem);
  }
}

void Task::buildCustomers(void * xml_elem) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  for (const auto & customer_pair : customers) {
    const Customer & customer = customer_pair.second;
    XMLElement * customer_elem = doc->NewElement("CTR");
    customer_elem->SetAttribute("A", customer.customer_id.c_str());
    customer_elem->SetAttribute("B", customer.customer_name.c_str());
    elem->InsertEndChild(customer_elem);
  }
}

void Task::buildProducts(void * xml_elem) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  for (const auto & product_pair : products) {
    const Product & product = product_pair.second;
    XMLElement * product_elem = doc->NewElement("PDT");
    product_elem->SetAttribute("A", product.product_id.c_str());
    product_elem->SetAttribute("B", product.product_name.c_str());
    elem->InsertEndChild(product_elem);
  }
}

void Task::buildDevices(void * xml_elem) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  for (const auto & device_pair : devices) {
    const Device & device = device_pair.second;
    XMLElement * device_elem = doc->NewElement("DVC");
    device_elem->SetAttribute("A", device.device_id.c_str());
    device_elem->SetAttribute("B", device.device_name.c_str());
    elem->InsertEndChild(device_elem);
  }
}

// ============================================================================
// XML Building - Partfield
// ============================================================================

void Task::buildPartfields(void * xml_elem) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  for (const auto & pf_pair : partfields) {
    const Partfield & pf = pf_pair.second;
    XMLElement * pf_elem = doc->NewElement("PFD");
    pf_elem->SetAttribute("A", pf.partfield_id.c_str());
    pf_elem->SetAttribute("B", pf.partfield_name.c_str());
    elem->InsertEndChild(pf_elem);

    for (const auto & cluster : lidar_clusters) {
      XMLElement * point = doc->NewElement("PNT");
      point->SetAttribute("A", 5);
      point->SetAttribute("C", cluster.latitude);
      point->SetAttribute("D", cluster.longitude);
      if (extension_format_ == ExtensionFormat::DESIGNATOR_JSON) {
        point->SetAttribute("B", clusterExtensionJson(cluster).c_str());
      } else {
        point->SetAttribute("B", "RoboSoft lidar cluster");
        XMLElement * extension = doc->NewElement("RoboSoftExtension");
        extension->SetAttribute("type", "lidar_cluster");
        extension->SetAttribute("xx", cluster.covariance_xx);
        extension->SetAttribute("xy", cluster.covariance_xy);
        extension->SetAttribute("yx", cluster.covariance_yx);
        extension->SetAttribute("yy", cluster.covariance_yy);
        point->InsertEndChild(extension);
      }
      pf_elem->InsertEndChild(point);
    }

    if (pf.partfield_id != current_partfield_id) {continue;}
    std::map<std::string, std::vector<const robosoft_interfaces::msg::GuidanceLine *>>
    grouped_lines;
    for (const auto & line : guidance_allocations) {
      const std::string group_id = line.guidance_group_id.empty() ?
        "GGP1" : line.guidance_group_id;
      grouped_lines[group_id].push_back(&line);
    }
    std::size_t generated_pattern_index = 1;
    for (const auto & [group_id, lines] : grouped_lines) {
      XMLElement * group_elem = doc->NewElement("GGP");
      group_elem->SetAttribute("A", group_id.c_str());
      for (const auto * line : lines) {
        const std::string pattern_id = line->guidance_pattern_id.empty() ?
          "GPN" + std::to_string(generated_pattern_index) :
          line->guidance_pattern_id;
        ++generated_pattern_index;
        XMLElement * pattern_elem = doc->NewElement("GPN");
        pattern_elem->SetAttribute("A", pattern_id.c_str());
        if (!line->designator.empty()) {
          pattern_elem->SetAttribute("B", line->designator.c_str());
        }
        pattern_elem->SetAttribute("C", 3);  // ISO GuidancePatternType: curve
        buildGuidanceLine(pattern_elem, *line);
        group_elem->InsertEndChild(pattern_elem);
      }
      pf_elem->InsertEndChild(group_elem);
    }
  }
}

// ============================================================================
// XML Building - Guidance
// ============================================================================

void Task::buildGuidanceAllocations(void * xml_elem) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  if (guidance_allocations.empty()) {
    return;
  }

  std::vector<std::string> allocated_groups;
  for (const auto & line : guidance_allocations) {
    const std::string group_id = line.guidance_group_id.empty() ?
      "GGP1" : line.guidance_group_id;
    if (std::find(allocated_groups.begin(), allocated_groups.end(), group_id) !=
      allocated_groups.end())
    {
      continue;
    }
    XMLElement * allocation = doc->NewElement("GAN");
    allocation->SetAttribute("A", group_id.c_str());
    elem->InsertEndChild(allocation);
    allocated_groups.push_back(group_id);
  }
}

// ============================================================================
// XML Building - Geometry (Polygon, LineString, Point)
// ============================================================================

void Task::buildPolygon(
  void * xml_elem,
  const std::vector<geometry_msgs::msg::Point32> & points) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  XMLElement * exterior = doc->NewElement("EXT");
  XMLElement * line_string = doc->NewElement("LSG");

  for (const auto & point : points) {
    XMLElement * point_elem = doc->NewElement("PNT");
    point_elem->SetAttribute("A", 2);  // ISO PointType: other
    point_elem->SetAttribute("C", point.y);
    point_elem->SetAttribute("D", point.x);
    line_string->InsertEndChild(point_elem);
  }

  exterior->InsertEndChild(line_string);
  elem->InsertEndChild(exterior);
}

void Task::buildLineString(
  void * xml_elem,
  const std::vector<robosoft_interfaces::msg::RoutePoint> & points) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  XMLElement * exterior = doc->NewElement("EXT");
  XMLElement * line_string = doc->NewElement("LSG");

  for (const auto & point : points) {
    XMLElement * point_elem = doc->NewElement("PNT");
    point_elem->SetAttribute("A", 2);  // ISO PointType: other
    point_elem->SetAttribute("C", point.latitude);
    point_elem->SetAttribute("D", point.longitude);
    point_elem->SetAttribute("E", point.altitude);
    line_string->InsertEndChild(point_elem);
  }

  exterior->InsertEndChild(line_string);
  elem->InsertEndChild(exterior);
}

void Task::buildGuidanceLine(
  void * xml_elem,
  const robosoft_interfaces::msg::GuidanceLine & line) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();
  XMLElement * line_string = doc->NewElement("LSG");
  line_string->SetAttribute("A", 5);  // ISO 11783 GuidancePattern

  if (extension_format_ == ExtensionFormat::DESIGNATOR_JSON) {
    line_string->SetAttribute("B", lineExtensionJson(line).c_str());
  } else {
    if (!line.designator.empty()) {
      line_string->SetAttribute("B", line.designator.c_str());
    }
    XMLElement * extension = doc->NewElement("RoboSoftExtension");
    extension->SetAttribute("group", line.guidance_group_id.c_str());
    extension->SetAttribute("pattern", line.guidance_pattern_id.c_str());
    extension->SetAttribute("movement", line.movement_type);
    extension->SetAttribute("sequence", line.sequence);
    extension->SetAttribute("direction", line.driving_direction);
    line_string->InsertEndChild(extension);
  }

  for (std::size_t index = 0; index < line.points.size(); ++index) {
    const auto & point = line.points[index];
    XMLElement * point_elem = doc->NewElement("PNT");
    const int point_type =
      index == 0 ? 6 : (index + 1 == line.points.size() ? 7 : 9);
    point_elem->SetAttribute("A", point_type);
    if (extension_format_ == ExtensionFormat::DESIGNATOR_JSON) {
      point_elem->SetAttribute("B", pointExtensionJson(point).c_str());
    } else {
      if (!point.designator.empty()) {
        point_elem->SetAttribute("B", point.designator.c_str());
      }
      XMLElement * extension = doc->NewElement("RoboSoftExtension");
      extension->SetAttribute("yaw", point.yaw);
      extension->SetAttribute("speed", point.speed);
      extension->SetAttribute("implement", point.implement_state);
      point_elem->InsertEndChild(extension);
    }
    point_elem->SetAttribute("C", point.latitude);
    point_elem->SetAttribute("D", point.longitude);
    point_elem->SetAttribute("E", point.altitude);
    line_string->InsertEndChild(point_elem);
  }

  elem->InsertEndChild(line_string);
}

void Task::buildPoint(void * xml_elem, const robosoft_interfaces::msg::RoutePoint & point) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);

  elem->SetAttribute("A", 2);  // ISO PointType: other
  if (!point.designator.empty()) {elem->SetAttribute("B", point.designator.c_str());}
  elem->SetAttribute("C", point.latitude);
  elem->SetAttribute("D", point.longitude);
  elem->SetAttribute("E", point.altitude);
}

void Task::buildExterior(
  void * xml_elem,
  const std::vector<geometry_msgs::msg::Point32> & points) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  XMLElement * line_string = doc->NewElement("LSG");

  for (const auto & point : points) {
    XMLElement * point_elem = doc->NewElement("PNT");
    point_elem->SetAttribute("A", 2);  // ISO PointType: other
    point_elem->SetAttribute("C", point.y);
    point_elem->SetAttribute("D", point.x);
    line_string->InsertEndChild(point_elem);
  }

  elem->InsertEndChild(line_string);
}

// ============================================================================
// XML Building - Operation Techniques & Cultural Practices
// ============================================================================

void Task::buildOperationTechniques(void * xml_elem) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  for (const auto & op_tech_pair : operation_techniques) {
    const OperationTechnique & op_tech = op_tech_pair.second;
    XMLElement * op_tech_elem = doc->NewElement("OTQ");
    op_tech_elem->SetAttribute("A", ("OTQ" + op_tech.operation_technique_id).c_str());
    op_tech_elem->SetAttribute("B", op_tech.operation_technique_designator.c_str());
    elem->InsertEndChild(op_tech_elem);
  }
}

void Task::buildCulturalPractices(void * xml_elem) const
{
  XMLElement * elem = static_cast<XMLElement *>(xml_elem);
  XMLDocument * doc = elem->GetDocument();

  for (const auto & cp_pair : cultural_practices) {
    const CulturalPractice & cp = cp_pair.second;
    XMLElement * cp_elem = doc->NewElement("CPC");
    cp_elem->SetAttribute("A", cp.cultural_practice_id.c_str());
    cp_elem->SetAttribute("B", cp.cultural_practice_designator.c_str());
    elem->InsertEndChild(cp_elem);
  }
}

}  // namespace robosoft_core
