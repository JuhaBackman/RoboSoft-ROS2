// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <robosoft_interfaces/msg/task_data.hpp>
#include <robosoft_interfaces/msg/guidance_line.hpp>
#include <robosoft_interfaces/msg/route_point.hpp>

namespace robosoft_core
{

// Forward declarations
struct Farm;
struct Customer;
struct Product;
struct Device;
struct Partfield;
struct GuidanceGroup;
struct GuidancePattern;
struct OperationTechnique;
struct CulturalPractice;

/**
 * @brief Task class for full ISO 11783 XML serialization/deserialization
 *
 * Handles the ISO 11783-10 TASKDATA structures used by RoboSoft. Runtime
 * messages may contain RoboSoft route metadata that the standard schema does
 * not define. On disk that metadata is written either as explicit extension
 * XML elements or as JSON in an ISO designator, selected by ExtensionFormat.
 *
 * Obsolete bracket-encoded designator extensions are intentionally not parsed.
 *
 * Standard ISO 11783 element and attribute names are used wherever the
 * represented field has a standard counterpart. RoboSoft-only route fields
 * follow the configured extension representation.
 */
class Task {
public:
  /// Selects how non-standard route metadata is represented in Task XML.
  enum class ExtensionFormat { XML_ELEMENTS, DESIGNATOR_JSON };

  Task();
  explicit Task(const robosoft_interfaces::msg::TaskData & msg);
  virtual ~Task() = default;

  /// Load a complete TaskData XML document, replacing the current contents.
  bool loadFromFile(const std::string & filepath);
  /// Serialize the current task using the configured extension format.
  bool saveToFile(const std::string & filepath) const;

  // Conversion boundary between the in-memory ISO model and ROS messages.
  robosoft_interfaces::msg::TaskData toMessage() const;
  void fromMessage(const robosoft_interfaces::msg::TaskData & msg);
  void setExtensionFormat(ExtensionFormat format) {extension_format_ = format;}

  // ===== ISO 11783 Full Structure =====

  // Root metadata
  std::string version_major;
  std::string version_minor;
  std::string management_software_manufacturer;
  std::string management_software_version;
  std::string task_controller_manufacturer;
  std::string task_controller_version;

  // Farms
  std::map<std::string, Farm> farms;
  std::string current_farm_id;

  // Customers
  std::map<std::string, Customer> customers;

  // Products (seeds, fertilizers, etc.)
  std::map<std::string, Product> products;

  // Devices
  std::map<std::string, Device> devices;

  // Partfields (field/plot definitions)
  std::map<std::string, Partfield> partfields;
  std::string current_partfield_id;

  // Guidance definitions
  std::map<std::string, GuidanceGroup> guidance_groups;
  std::map<std::string, GuidancePattern> guidance_patterns;
  std::vector<robosoft_interfaces::msg::GuidanceLine> guidance_allocations;
  std::vector<robosoft_interfaces::msg::LidarCluster> lidar_clusters;

  // Operation techniques
  std::map<std::string, OperationTechnique> operation_techniques;

  // Cultural practices
  std::map<std::string, CulturalPractice> cultural_practices;

  // Task properties
  std::string task_name;
  std::string task_id;
  std::string task_designator;
  uint64_t creation_timestamp;
  uint8_t task_status;

private:
  // Extension selection affects serialization and accepted modern metadata,
  // but never enables the obsolete bracket-designator parser.
  ExtensionFormat extension_format_{ExtensionFormat::XML_ELEMENTS};
  // XML parsing - Root level
  bool parseRoot(void * xml_elem);

  // XML parsing - Farms & Customers
  bool parseFarm(void * xml_elem);
  bool parseCustomer(void * xml_elem);
  bool parseProduct(void * xml_elem);
  bool parseDevice(void * xml_elem);

  // XML parsing - Partfield
  bool parsePartfield(void * xml_elem);

  // XML parsing - Guidance
  bool parseGuidanceGroup(void * xml_elem);
  bool parseGuidancePattern(void * xml_elem);
  bool parseGuidanceAllocation(void * xml_elem);

  // XML parsing - Geometry
  bool parsePolygon(void * xml_elem, std::vector<geometry_msgs::msg::Point32> & points);
  bool parseLineString(void * xml_elem, std::vector<robosoft_interfaces::msg::RoutePoint> & points);
  bool parsePoint(void * xml_elem, robosoft_interfaces::msg::RoutePoint & point);
  bool parseExterior(void * xml_elem, std::vector<geometry_msgs::msg::Point32> & points);

  // XML parsing - Operation techniques
  bool parseOperationTechnique(void * xml_elem);
  bool parseCulturalPractice(void * xml_elem);

  // XML building - Root level
  void buildRoot(void * xml_elem) const;

  // XML building - Farms & Customers
  void buildFarms(void * xml_elem) const;
  void buildCustomers(void * xml_elem) const;
  void buildProducts(void * xml_elem) const;
  void buildDevices(void * xml_elem) const;

  // XML building - Partfield
  void buildPartfields(void * xml_elem) const;

  // XML building - Guidance
  void buildGuidanceAllocations(void * xml_elem) const;

  // XML building - Geometry
  void buildPolygon(void * xml_elem, const std::vector<geometry_msgs::msg::Point32> & points) const;
  void buildLineString(
    void * xml_elem,
    const std::vector<robosoft_interfaces::msg::RoutePoint> & points) const;
  void buildGuidanceLine(
    void * xml_elem,
    const robosoft_interfaces::msg::GuidanceLine & line) const;
  void buildPoint(void * xml_elem, const robosoft_interfaces::msg::RoutePoint & point) const;
  void buildExterior(
    void * xml_elem,
    const std::vector<geometry_msgs::msg::Point32> & points) const;

  // XML building - Operation techniques
  void buildOperationTechniques(void * xml_elem) const;
  void buildCulturalPractices(void * xml_elem) const;

  // Utility parsing helpers
  std::string getAttribute(void * xml_elem, const std::string & attr_name);
  double getAttributeDouble(void * xml_elem, const std::string & attr_name);
  int64_t getAttributeInt64(void * xml_elem, const std::string & attr_name);
};

// ===== ISO 11783 Data Structures =====

struct Farm
{
  std::string farm_id;
  std::string farm_name;
  std::string farm_designator;
  std::string country_code;
};

struct Customer
{
  std::string customer_id;
  std::string customer_name;
  std::string customer_designator;
};

struct Product
{
  std::string product_id;
  std::string product_name;
  std::string product_designator;
  std::string product_type;
};

struct Device
{
  std::string device_id;
  std::string device_name;
  std::string device_designator;
  std::string device_serial_number;
};

struct Partfield
{
  std::string partfield_id;
  std::string partfield_name;
  std::string partfield_designator;
  std::string farm_id_ref;
  double area;  // Square meters
  std::vector<geometry_msgs::msg::Point32> boundary_polygon;
};

struct GuidanceGroup
{
  std::string guidance_group_id;
  std::string guidance_group_designator;
  std::vector<std::string> guidance_pattern_ids;
};

struct GuidancePattern
{
  std::string guidance_pattern_id;
  std::string guidance_pattern_designator;
  std::string guidance_group_id;
  robosoft_interfaces::msg::GuidanceLine line;
};

struct OperationTechnique
{
  std::string operation_technique_id;
  std::string operation_technique_designator;
};

struct CulturalPractice
{
  std::string cultural_practice_id;
  std::string cultural_practice_designator;
};

}  // namespace robosoft_core
