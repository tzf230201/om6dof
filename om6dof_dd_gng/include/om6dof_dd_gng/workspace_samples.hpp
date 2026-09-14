#pragma once

#include <array>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include "om6dof_dd_gng/reachability_graph.hpp"

namespace om6dof_dd_gng::reachability
{
struct WorkspaceSample {Point3 requested_position; std::vector<double> joints;};

inline std::vector<std::string> workspaceCsvFields(std::string line)
{
  if (!line.empty() && line.back() == '\r') {line.pop_back();}
  if (line.find('"') != std::string::npos) {
    throw std::runtime_error("workspace CSV must use the scanner's unquoted numeric format");
  }
  std::vector<std::string> out;
  std::stringstream input(line);
  std::string field;
  while (std::getline(input, field, ',')) {out.push_back(field);}
  if (!line.empty() && line.back() == ',') {out.emplace_back();}
  return out;
}

inline double workspaceNumber(const std::string & text)
{
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::runtime_error("workspace CSV contains an invalid numeric value");
  }
  return value;
}

inline std::vector<WorkspaceSample> readWorkspaceSamples(std::istream & input)
{
  std::string line;
  if (!std::getline(input, line)) {throw std::runtime_error("empty workspace CSV");}
  const auto header = workspaceCsvFields(line);
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (!columns.emplace(header[i], i).second) {throw std::runtime_error("duplicate workspace column");}
  }
  for (const std::string key : {"x_mm", "y_mm", "z_mm", "position_found",
      "q1_rad", "q2_rad", "q3_rad", "q4_rad", "q5_rad", "q6_rad"})
  {
    if (!columns.count(key)) {throw std::runtime_error("missing workspace column: " + key);}
  }
  std::vector<WorkspaceSample> samples;
  std::size_t row = 1;
  while (std::getline(input, line)) {
    ++row;
    if (line.empty() || line == "\r") {continue;}
    const auto fields = workspaceCsvFields(line);
    if (fields.size() != header.size()) {
      throw std::runtime_error("workspace column count mismatch at row " + std::to_string(row));
    }
    const auto & found = fields[columns.at("position_found")];
    if (found == "0") {continue;}
    if (found != "1") {throw std::runtime_error("invalid position_found at row " + std::to_string(row));}
    WorkspaceSample sample;
    sample.requested_position = {
      workspaceNumber(fields[columns.at("x_mm")]) * 0.001,
      workspaceNumber(fields[columns.at("y_mm")]) * 0.001,
      workspaceNumber(fields[columns.at("z_mm")]) * 0.001};
    for (int j = 1; j <= 6; ++j) {
      sample.joints.push_back(workspaceNumber(fields[columns.at("q" + std::to_string(j) + "_rad")]));
    }
    samples.push_back(std::move(sample));
  }
  if (samples.empty()) {throw std::runtime_error("workspace CSV has no position-found witnesses");}
  return samples;
}

// Spatial buckets only accelerate candidate discovery. Ranking uses joint-space
// distance, retaining distinct IK configurations even at coincident XYZ.
inline std::vector<Edge> workspaceCandidateEdges(
  const std::vector<Node> & nodes, const std::vector<double> & ranges,
  double cartesian_limit, double joint_limit, std::size_t neighbors)
{
  if (!std::isfinite(cartesian_limit) || cartesian_limit <= 0.0 ||
    !std::isfinite(joint_limit) || joint_limit <= 0.0 || neighbors == 0)
  {throw std::invalid_argument("invalid workspace edge limits");}
  using Cell = std::array<int, 3>;
  auto cell = [cartesian_limit](const Point3 & p) -> Cell {
    return {static_cast<int>(std::floor(p.x / cartesian_limit)),
      static_cast<int>(std::floor(p.y / cartesian_limit)),
      static_cast<int>(std::floor(p.z / cartesian_limit))};
  };
  std::map<Cell, std::vector<std::size_t>> buckets;
  for (std::size_t i = 0; i < nodes.size(); ++i) {buckets[cell(nodes[i].position)].push_back(i);}
  std::map<std::pair<std::size_t, std::size_t>, double> unique;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const auto c = cell(nodes[i].position);
    std::vector<std::pair<double, std::size_t>> ranked;
    for (int x = -1; x <= 1; ++x) {for (int y = -1; y <= 1; ++y) {for (int z = -1; z <= 1; ++z) {
      auto it = buckets.find({c[0] + x, c[1] + y, c[2] + z});
      if (it == buckets.end()) {continue;}
      for (auto j : it->second) {
        if (i == j || distance(nodes[i].position, nodes[j].position) > cartesian_limit) {continue;}
        const double d = normalizedJointDistance(nodes[i].joints, nodes[j].joints, ranges);
        if (d > 1.0e-9 && d <= joint_limit) {ranked.emplace_back(d, j);}
      }
    }}}
    const auto count = std::min(neighbors, ranked.size());
    std::partial_sort(ranked.begin(), ranked.begin() + count, ranked.end());
    for (std::size_t k = 0; k < count; ++k) {
      const auto j = ranked[k].second;
      unique[std::minmax(i, j)] = jointPathCost(nodes[i].joints, nodes[j].joints);
    }
  }
  std::vector<Edge> edges;
  for (const auto & entry : unique) {edges.push_back({entry.first.first, entry.first.second, entry.second});}
  return edges;
}
}  // namespace om6dof_dd_gng::reachability
