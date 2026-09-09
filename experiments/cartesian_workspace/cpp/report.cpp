// Offline sampled-workspace report. Reads CSV only; no ROS or hardware access.
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "viewer3d.hpp"

namespace fs = std::filesystem;
namespace {
constexpr std::array<const char*, 5> statuses{{
    "pose_found", "pose_near_singular", "orientation_unresolved",
    "collision_candidates_only", "position_unresolved"}};
constexpr std::array<const char*, 5> colors{{
    "#15976b", "#397ac6", "#e5a21a", "#9658ad", "#d15b59"}};
constexpr std::array<const char*, 5> labels{{
    "Pose found", "Pose near singularity", "Position found; orientation unresolved",
    "Colliding candidates only", "Position unresolved"}};
constexpr std::array<const char*, 5> shapes{{
    "circle", "triangle", "square", "diamond", "cross"}};
constexpr std::array<const char*, 5> shape_labels{{
    "circle", "triangle", "square", "diamond", "X"}};

std::string number(double v, int digits = 2) {
  if (!std::isfinite(v)) return "n/a";
  std::ostringstream out;
  out << std::fixed << std::setprecision(digits) << v;
  return out.str();
}

std::string escape(const std::string& value) {
  std::string out;
  for (char c : value) {
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

// Escape metadata for a JSON script-data block, including HTML delimiters.
// The viewer reads this string with textContent; metadata is never executable HTML.
std::string json_string(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    if (c == '"') out << "\\\"";
    else if (c == '\\') out << "\\\\";
    else if (c == '/') out << "\\/";
    else if (c < 0x20 || c == '<' || c == '>' || c == '&')
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
          << static_cast<unsigned int>(c) << std::dec;
    else out << static_cast<char>(c);
  }
  out << '"';
  return out.str();
}

void json_number(std::ostream& out, double value) {
  if (std::isfinite(value)) out << std::setprecision(17) << value;
  else out << "null";
}

std::vector<std::string> csv_fields(const std::string& line) {
  std::vector<std::string> result;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        field += '"';
        ++i;
      } else quoted = !quoted;
    } else if (c == ',' && !quoted) {
      result.push_back(field);
      field.clear();
    } else if (c != '\r') field += c;
  }
  if (quoted) throw std::runtime_error("CSV contains an unterminated quoted field");
  result.push_back(field);
  return result;
}

double numeric(const std::string& s, bool required = false) {
  if (s.empty() || s == "None" || s == "null") {
    if (required) throw std::runtime_error("Required CSV number is empty");
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::size_t used = 0;
  double v = std::stod(s, &used);
  if (used != s.size() || (required && !std::isfinite(v)))
    throw std::runtime_error("Invalid CSV number: " + s);
  return v;
}

bool boolean(const std::string& s) {
  if (s == "True" || s == "true" || s == "1") return true;
  if (s == "False" || s == "false" || s == "0") return false;
  throw std::runtime_error("Invalid CSV boolean: " + s);
}

struct Point {
  std::array<double, 3> xyz{};
  bool position = false, pose = false;
  int status = 4;
  double rcond = NAN, margin = NAN, position_error = NAN, orientation_error = NAN;
  std::array<double, 6> q{{NAN, NAN, NAN, NAN, NAN, NAN}};
  std::array<double, 3> rpy{{NAN, NAN, NAN}};
};

std::vector<Point> read_points(const fs::path& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot read " + path.string());
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("CSV is empty");
  const auto headers = csv_fields(line);
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < headers.size(); ++i) columns[headers[i]] = i;
  for (const auto* column : {"x_mm", "y_mm", "z_mm", "position_found", "pose_found",
                            "status", "rcond", "joint_margin_deg", "position_error_mm",
                            "orientation_error_deg"}) {
    if (!columns.count(column)) throw std::runtime_error("CSV missing column: " + std::string(column));
  }
  std::vector<Point> points;
  std::size_t line_num = 1;
  while (std::getline(in, line)) {
    ++line_num;
    if (line.empty() || line == "\r") continue;
    const auto fields = csv_fields(line);
    if (fields.size() != headers.size())
      throw std::runtime_error("CSV column count mismatch at line " + std::to_string(line_num));
    const auto get = [&](const char* name) -> const std::string& { return fields.at(columns.at(name)); };
    Point p;
    p.xyz = {numeric(get("x_mm"), true), numeric(get("y_mm"), true), numeric(get("z_mm"), true)};
    p.position = boolean(get("position_found"));
    p.pose = boolean(get("pose_found"));
    auto it = std::find_if(statuses.begin(), statuses.end(), [&](const char* s) { return get("status") == s; });
    if (it == statuses.end()) throw std::runtime_error("Unknown point status: " + get("status"));
    p.status = static_cast<int>(it - statuses.begin());
    if (p.pose != (p.status == 0 || p.status == 1) || p.position != (p.status <= 2))
      throw std::runtime_error("Inconsistent reachability/status at CSV line " + std::to_string(line_num));
    p.rcond = numeric(get("rcond"));
    p.margin = numeric(get("joint_margin_deg"));
    p.position_error = numeric(get("position_error_mm"));
    p.orientation_error = numeric(get("orientation_error_deg"));
    for (std::size_t joint = 0; joint < p.q.size(); ++joint) {
      const auto name = "q" + std::to_string(joint + 1) + "_rad";
      if (columns.count(name)) p.q[joint] = numeric(get(name.c_str()));
    }
    const std::array<const char*, 3> rpy_columns{{
        "target_roll_deg", "target_pitch_deg", "target_yaw_deg"}};
    for (std::size_t axis = 0; axis < p.rpy.size(); ++axis)
      if (columns.count(rpy_columns[axis])) p.rpy[axis] = numeric(get(rpy_columns[axis]));
    points.push_back(p);
  }
  if (points.empty()) throw std::runtime_error("CSV has no data points");
  return points;
}

struct Counts {
  std::array<std::size_t, 5> counts{};
  std::size_t total = 0, position = 0, pose = 0, small_margin = 0;
  void add(const Point& p) {
    ++total;
    ++counts[p.status];
    position += p.position;
    pose += p.pose;
    small_margin += p.position && std::isfinite(p.margin) && p.margin < 1.0;
  }
  double rate(std::size_t n) const { return total ? 100.0 * n / total : NAN; }
};

struct Slice {
  int axis;
  double fixed;
  std::vector<const Point*> points;
  Counts counts;
  std::string filename;
};

std::string axis_name(int axis) { return std::string(1, "XYZ"[axis]); }

std::string slice_name(int axis, double fixed) {
  std::string value = number(std::abs(fixed), 3);
  while (value.back() == '0') value.pop_back();
  if (value.back() == '.') value.pop_back();
  std::replace(value.begin(), value.end(), '.', '_');
  return axis_name(axis) + "_" + (fixed < 0 ? "minus_" : "plus_") + value + "mm.svg";
}

std::ofstream output_file(const fs::path& path) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("Cannot write " + path.string());
  out.exceptions(std::ios::badbit | std::ios::failbit);
  return out;
}

std::string metadata_text(const fs::path& output) {
  std::ifstream in(output / "summary.json");
  if (!in) return "summary.json is unavailable; check the experiment parameters from the scanner.";
  std::ostringstream contents;
  contents << in.rdbuf();
  return contents.str();
}

void begin_svg(std::ostream& out, double width, double height, const std::string& title) {
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << width << ' ' << height
      << "\" role=\"img\"><title>" << escape(title) << "</title>"
      << "<rect width=\"100%\" height=\"100%\" fill=\"#f7f8fa\"/>"
      << "<style>text{font-family:Arial,sans-serif;fill:#243244}.grid{stroke:#d7dde5;stroke-width:.6}</style>\n";
}

// Keep plotted samples and every legend in sync, including when viewed without color.
void draw_marker(std::ostream& out, std::size_t status, double x, double y, double size,
                 const char* context, const std::string& title = "") {
  const double r = size / 2;
  out << "<g class=\"marker\" data-status=\"" << statuses[status]
      << "\" data-shape=\"" << shapes[status] << "\" data-context=\"" << context << "\">";
  if (!title.empty()) out << "<title>" << escape(title) << "</title>";
  if (status == 0) {
    out << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"" << r
        << "\" fill=\"" << colors[status] << "\"/>";
  } else if (status == 2) {
    out << "<rect x=\"" << x - r << "\" y=\"" << y - r << "\" width=\"" << size
        << "\" height=\"" << size << "\" fill=\"" << colors[status] << "\"/>";
  } else if (status == 1 || status == 3) {
    out << "<polygon points=\"" << x << ',' << y - r << ' ' << x + r << ',';
    if (status == 1) out << y + r << ' ' << x - r << ',' << y + r;
    else out << y << ' ' << x << ',' << y + r << ' ' << x - r << ',' << y;
    out << "\" fill=\"" << colors[status] << "\"/>";
  } else {
    out << "<path d=\"M " << x - r << ' ' << y - r << " L " << x + r << ' ' << y + r
        << " M " << x + r << ' ' << y - r << " L " << x - r << ' ' << y + r
        << "\" fill=\"none\" stroke=\"" << colors[status] << "\" stroke-width=\""
        << std::max(1.4, size * .18) << "\" stroke-linecap=\"round\"/>";
  }
  out << "</g>";
}

void legend(std::ostream& out, double x, double y, double fontsize = 13) {
  for (std::size_t i = 0; i < statuses.size(); ++i) {
    draw_marker(out, i, x + 6, y + 22 * i - 4, 12, "legend");
    out << "<text x=\"" << x + 20 << "\" y=\"" << y + 22 * i << "\" font-size=\"" << fontsize
        << "\">" << labels[i] << " (" << shape_labels[i] << ")</text>\n";
  }
}

void draw_slice(std::ostream& out, const Slice& slice, double x, double y, double size,
                double extent, double spacing, bool compact) {
  const int horizontal = slice.axis == 0 ? 1 : 0;
  const int vertical = slice.axis == 2 ? 1 : 2;
  const double inset = compact ? 39 : 56;
  const double plot = size - inset - 20;
  const double left = x + inset, top = y + 58;
  const auto px = [&](double v) { return left + (v + extent) / (2 * extent) * plot; };
  const auto py = [&](double v) { return top + (extent - v) / (2 * extent) * plot; };
  out << "<g><text x=\"" << x + size / 2 << "\" y=\"" << y + 21 << "\" text-anchor=\"middle\" font-size=\"" << (compact ? 15 : 21) << "\">"
      << axis_name(slice.axis) << " = " << number(slice.fixed, 0) << " mm (constant)</text>\n"
      << "<text x=\"" << x + size / 2 << "\" y=\"" << y + 40 << "\" text-anchor=\"middle\" font-size=\"11\">n=" << slice.counts.total
      << "; position " << number(slice.counts.rate(slice.counts.position), 1) << "%; pose " << number(slice.counts.rate(slice.counts.pose), 1) << "%</text>\n"
      << "<rect x=\"" << left << "\" y=\"" << top << "\" width=\"" << plot << "\" height=\"" << plot << "\" fill=\"white\" stroke=\"#a5afbd\"/>\n";
  // Coarser axis ticks keep compact sheets legible without dropping sample points.
  const double tick = compact ? std::max(100.0, spacing * 2) : std::max(50.0, spacing);
  for (double v = std::ceil(-extent / tick) * tick; v <= extent; v += tick) {
    out << "<path class=\"grid\" d=\"M " << px(v) << ' ' << top << " v " << plot << " M " << left << ' ' << py(v) << " h " << plot << "\"/>"
        << "<text x=\"" << px(v) << "\" y=\"" << top + plot + 15 << "\" text-anchor=\"middle\" font-size=\"9\">" << number(v, 0) << "</text>"
        << "<text x=\"" << left - 5 << "\" y=\"" << py(v) + 3 << "\" text-anchor=\"end\" font-size=\"9\">" << number(v, 0) << "</text>\n";
  }
  out << "<path d=\"M " << px(0) << ' ' << top << " v " << plot << " M " << left << ' ' << py(0) << " h " << plot << "\" fill=\"none\" stroke=\"#8a95a3\" stroke-width=\"1\"/>\n";
  const double cell = std::max(1.5, spacing / (2 * extent) * plot * .76);
  for (const Point* p : slice.points) {
    std::ostringstream title;
    title << "X=" << number(p->xyz[0]) << "; Y=" << number(p->xyz[1]) << "; Z=" << number(p->xyz[2]) << " mm; " << labels[p->status]
        << "; position error=" << number(p->position_error) << " mm; orientation error=" << number(p->orientation_error) << " deg; rcond=" << number(p->rcond, 5)
        << "; margin=" << number(p->margin) << " deg";
    draw_marker(out, p->status, px(p->xyz[horizontal]), py(p->xyz[vertical]), cell, "point", title.str());
    out << '\n';
  }
  if (slice.points.empty()) out << "<text x=\"" << left + plot / 2 << "\" y=\"" << top + plot / 2 << "\" text-anchor=\"middle\" font-size=\"13\">No samples on this plane</text>\n";
  out << "<text x=\"" << left + plot / 2 << "\" y=\"" << top + plot + 31 << "\" text-anchor=\"middle\" font-size=\"12\">" << axis_name(horizontal) << " (mm)</text>"
      << "<text transform=\"translate(" << x + 12 << ',' << top + plot / 2 << ") rotate(-90)\" text-anchor=\"middle\" font-size=\"12\">" << axis_name(vertical) << " (mm)</text></g>\n";
}

void write_slices(const fs::path& output, const std::vector<Slice>& slices,
                  double extent, double spacing) {
  fs::create_directories(output / "slices");
  auto csv = output_file(output / "slices.csv");
  csv << "constant_axis,constant_mm,total,position_found,pose_found,position_percent,pose_percent,pose_near_singular,orientation_unresolved,collision_candidates_only,position_unresolved,witness_margin_below_1deg,svg\n";
  for (const auto& slice : slices) {
    auto out = output_file(output / "slices" / slice.filename);
    begin_svg(out, 750, 870, "OM6DOF " + axis_name(slice.axis) + " = " + number(slice.fixed) + " mm");
    draw_slice(out, slice, 5, 5, 730, extent, spacing, false);
    legend(out, 65, 772);
    out << "<text x=\"420\" y=\"783\" font-size=\"12\">Model samples; no robot test.</text>"
        << "<text x=\"420\" y=\"805\" font-size=\"12\">White: untested / outside grid.</text>"
        << "<text x=\"420\" y=\"827\" font-size=\"12\">Red does not prove impossibility.</text></svg>\n";
    const Counts& c = slice.counts;
    csv << axis_name(slice.axis) << ',' << number(slice.fixed, 6) << ',' << c.total << ',' << c.position << ',' << c.pose << ','
        << number(c.rate(c.position), 5) << ',' << number(c.rate(c.pose), 5) << ',' << c.counts[1] << ',' << c.counts[2] << ',' << c.counts[3] << ',' << c.counts[4] << ',' << c.small_margin << ",slices/" << slice.filename << '\n';
  }
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<const Slice*> planes;
    for (const auto& slice : slices) if (slice.axis == axis) planes.push_back(&slice);
    const int columns = 4;
    const int rows = static_cast<int>((planes.size() + columns - 1) / columns);
    auto out = output_file(output / ("slices_" + axis_name(axis) + ".svg"));
    begin_svg(out, 1200, 100 + rows * 345 + 150, "OM6DOF all constant-" + axis_name(axis) + " planes");
    out << "<text x=\"28\" y=\"30\" font-size=\"23\">OM6DOF: constant " << axis_name(axis) << ", " << number(spacing, 0) << " mm increments</text>"
        << "<text x=\"28\" y=\"55\" font-size=\"14\">All planes from negative to positive; shared scale. White = no tested sample.</text>"
        << "<text x=\"28\" y=\"76\" font-size=\"14\">Red/purple do not prove a mechanical dead zone; yellow = position found, orientation unresolved.</text>\n";
    for (std::size_t i = 0; i < planes.size(); ++i)
      draw_slice(out, *planes[i], (i % columns) * 300, 95 + (i / columns) * 345, 295, extent, spacing, true);
    legend(out, 35, 115 + rows * 345);
    out << "</svg>\n";
  }
}

double quantile(std::vector<double> values, double fraction) {
  if (values.empty()) return NAN;
  std::sort(values.begin(), values.end());
  const double index = fraction * (values.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(index));
  const auto hi = static_cast<std::size_t>(std::ceil(index));
  return values[lo] + (values[hi] - values[lo]) * (index - lo);
}

void write_analysis(const fs::path& output, const std::vector<Point>& points,
                    const std::vector<Slice>& slices, const Counts& totals, double spacing) {
  auto out = output_file(output / "analysis.md");
  out << "# OM6DOF Cartesian workspace analysis\n\n"
      << "This offline experiment contains **" << totals.total << " discrete points**. The report shows every constant-X, constant-Y, and constant-Z plane at multiples of **" << number(spacing, 0)
      << " mm**, from negative to positive coordinates within the sample range. On each plane, the other two coordinates vary; the report is not limited to the X=Y=Z=0 slices.\n\n"
      << "Solver parameters, target orientation, tolerances, model radius, seeds, and computation time are recorded in [summary.json](summary.json). "
      << "Joint values and the orientation actually tested at each point are in [points.csv](points.csv). "
      << "Open the [interactive 3D viewer](viewer3d.html), [all constant-coordinate planes](index.html), or [per-plane statistics](slices.csv).\n\n"
      << "## Main results\n\n"
      << "- Position found without requiring a specific orientation: **" << totals.position << '/' << totals.total << " (" << number(totals.rate(totals.position), 1) << "%)**.\n"
      << "- Both position and the tested orientation found: **" << totals.pose << '/' << totals.total << " (" << number(totals.rate(totals.pose), 1) << "%)**.\n"
      << "- Among the poses found, " << totals.counts[1] << " points are classified as near singularity. This describes the best candidate found; it does not prove that every configuration at that point is singular.\n"
      << "- " << totals.small_margin << " selected position candidates have less than 1 degree of margin to an effective joint limit. This indicates sensitivity of those candidates, not that every solution lies near a limit.\n\n"
      << "| Category | Points | Percentage of all points |\n|---|---:|---:|\n";
  for (std::size_t i = 0; i < statuses.size(); ++i)
    out << "| " << labels[i] << " | " << totals.counts[i] << " | " << number(totals.rate(totals.counts[i]), 2) << "% |\n";
  out << "\nThese percentages count only tested grid points; they do not measure continuous workspace volume or the physical robot's probability of success.\n\n"
      << "## Interpretation for mechanical design\n\n"
      << "1. **Green** provides a witness joint configuration satisfying the position, tested orientation, joint limits, and model self-collision check.\n"
      << "2. **Blue triangles** mean a pose was found, but the best candidate's Jacobian is poorly conditioned. Small Cartesian motions may require large joint changes. The rcond metric depends on length normalization; compare only experiments with identical settings.\n"
      << "3. **Yellow squares** separate orientation constraints from position reachability. They help evaluate gripper direction, wrist range, and tasks allowing a free orientation. They do not prove the requested orientation is impossible.\n"
      << "4. **Purple** means the search found only colliding position candidates under the approximate collision model; it does not prove that no collision-free configuration exists.\n"
      << "5. **Red** means the position search did not succeed. Possible causes include geometry, joint limits, too few seeds, or convergence to a local minimum. Do not treat it as proof of a mechanical dead zone.\n\n"
      << "Investigate suspected problem regions with a finer local grid, more seeds/iterations, several orientations, and mesh collision checks. "
      << "Comparing zero joint margin with the operational margin can separate software constraints from the URDF range, but does not change or establish the physical hardware limits.\n\n"
      << "## Selected-candidate statistics\n\n"
      << "Margins are measured against the effective joint limits used by the scanner. Position-only results use the candidate with the smallest position error; full-pose results use the candidate with the best rcond.\n\n"
      << "| Metric | Samples | Minimum | Median | Maximum |\n|---|---:|---:|---:|---:|\n";
  std::vector<double> conditions, margins, pose_errors, angle_errors;
  std::vector<const Point*> witnesses;
  for (const auto& p : points) {
    if (p.pose && std::isfinite(p.rcond)) conditions.push_back(p.rcond);
    if (p.position && std::isfinite(p.margin)) { margins.push_back(p.margin); witnesses.push_back(&p); }
    if (p.pose && std::isfinite(p.position_error)) pose_errors.push_back(p.position_error);
    if (p.pose && std::isfinite(p.orientation_error)) angle_errors.push_back(p.orientation_error);
  }
  const auto stat = [&](const char* label, const std::vector<double>& v) {
    out << "| " << label << " | " << v.size() << " | " << number(quantile(v, 0), 5) << " | " << number(quantile(v, .5), 5) << " | " << number(quantile(v, 1), 5) << " |\n";
  };
  stat("Pose rcond (dimensionless)", conditions);
  stat("Position-candidate joint margin (deg)", margins);
  stat("Position error of found poses (mm)", pose_errors);
  stat("Orientation error of found poses (deg)", angle_errors);
  std::stable_sort(witnesses.begin(), witnesses.end(), [](const Point* a, const Point* b) { return a->margin < b->margin; });
  out << "\nExample candidates with the smallest joint margins (all joint values are available in points.csv):\n\n"
      << "| X (mm) | Y (mm) | Z (mm) | Margin (deg) | Status |\n|---:|---:|---:|---:|---|\n";
  for (std::size_t i = 0; i < std::min<std::size_t>(8, witnesses.size()); ++i) {
    const Point& p = *witnesses[i];
    out << "| " << number(p.xyz[0], 0) << " | " << number(p.xyz[1], 0) << " | " << number(p.xyz[2], 0) << " | " << number(p.margin, 4) << " | " << labels[p.status] << " |\n";
  }
  out << "\n## All constant-coordinate planes\n\n"
      << "Each point appears once in each axis group when the grid aligns with the slice increment. Do not add the X+Y+Z group counts and interpret them as unique points. "
      << "Planes with no samples are marked n/a, not counted as failures.\n\n";
  for (int axis = 0; axis < 3; ++axis) {
    out << "### Constant " << axis_name(axis) << "\n\n"
        << "![All constant-" << axis_name(axis) << " planes](slices_" << axis_name(axis) << ".svg)\n\n"
        << "| " << axis_name(axis) << " (mm) | Tested | Position found | Pose found | Near singularity | Orientation unresolved | Colliding only | Position unresolved |\n"
        << "|---:|---:|---:|---:|---:|---:|---:|---:|\n";
    for (const auto& slice : slices) {
      if (slice.axis != axis) continue;
      const auto& c = slice.counts;
      out << "| [" << number(slice.fixed, 0) << "](slices/" << slice.filename << ") | " << c.total << " | " << c.position << " (" << number(c.rate(c.position), 1) << "%) | " << c.pose
          << " (" << number(c.rate(c.pose), 1) << "%) | " << c.counts[1] << " | " << c.counts[2] << " | " << c.counts[3] << " | " << c.counts[4] << " |\n";
    }
    out << '\n';
  }
  out << "## Limitations\n\n"
      << "- No ROS commands, servo connections, or robot motion. All results describe model kinematics, not measured physical accuracy.\n"
      << "- Each point tests one orientation defined by the experiment settings, not every rotation in SO(3). In radial mode, yaw follows the point's azimuth.\n"
      << "- The bounding sphere is a conservative model sampling domain, not a guarantee that every position inside its radius is reachable. Regions below the base are also sampled if inside the sphere; a table/floor is not modeled.\n"
      << "- Collision checks use approximate model links at the endpoint, not precise mechanical meshes, environmental obstacles, or a verified path from a starting configuration.\n"
      << "- Payload, flexibility, backlash, calibration, motor torque, gravity, velocity, and acceleration are not evaluated.\n"
      << "- Colored markers mark sample centers; they do not fill or classify the area/volume between samples. Unsampled positions are not assessed. Green circles, yellow squares, blue triangles, purple diamonds, and red X markers distinguish the categories.\n"
      << "- A confirmed mechanical dead zone requires additional evidence; numerical IK failure alone is insufficient.\n";
  out << "\n## Experiment parameters (summary.json snapshot)\n\n"
      << "Check `orientation`, `rpy_deg`, reference frame, position/orientation tolerances, and joint margin before comparing maps. "
      << "The reporter does not change scanner parameters or data.\n\n```json\n" << metadata_text(output) << "\n```\n";
}

void write_html(const fs::path& output, const std::vector<Slice>& slices, const Counts& total, double spacing) {
  auto out = output_file(output / "index.html");
  out << "<!doctype html><html lang=\"en\"><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
      << "<title>OM6DOF workspace — all constant-coordinate planes</title>"
      << "<style>body{font:16px/1.5 system-ui,sans-serif;max-width:1240px;margin:32px auto;padding:0 20px;color:#243244;background:#f7f8fa}a{color:#1760a5}"
      << "nav,.planes{display:flex;gap:8px;flex-wrap:wrap}.planes a{padding:5px 9px;border:1px solid #c8d2df;border-radius:5px;background:white}"
      << "img{width:100%;height:auto;background:white;border:1px solid #d7dde5;border-radius:8px}section{margin-top:36px}.notice{padding:16px;background:#fff1ce;border-radius:8px}"
      << "li{margin:5px 0}.legend{display:flex;gap:16px;flex-wrap:wrap}.legend-symbol{width:18px;height:18px;margin-right:6px;vertical-align:-3px}"
      << ".viewer-link{display:inline-block;background:#1760a5;color:white;padding:12px 20px;border-radius:8px;text-decoration:none;font-weight:600}</style>"
      << "<body><h1>OM6DOF: Cartesian workspace map</h1>"
      << "<p>" << total.total << " offline points; constant-coordinate planes every " << number(spacing, 0) << " mm along all three axes, including negative and positive coordinates.</p>"
      << "<p>Position found: " << total.position << " (" << number(total.rate(total.position), 1) << "%); position and tested orientation found: " << total.pose << " (" << number(total.rate(total.pose), 1) << "%).</p>"
      << "<p><a class=\"viewer-link\" href=\"viewer3d.html\">Open interactive 3D viewer</a></p>"
      << "<p class=\"notice\">Red does not prove a mechanical dead zone. These are discrete model IK results, not physical hardware or path tests. White = untested points. All panels share the same coordinate scale.</p>"
      << "<nav><a href=\"analysis.md\">Full analysis</a> · <a href=\"summary.json\">Experiment parameters</a> · <a href=\"points.csv\">Point/joint data</a> · <a href=\"slices.csv\">Per-plane data</a> · "
      << "<a href=\"#X\">Constant X</a> · <a href=\"#Y\">Constant Y</a> · <a href=\"#Z\">Constant Z</a></nav><p class=\"legend\">";
  for (std::size_t i = 0; i < statuses.size(); ++i) {
    out << "<span><svg xmlns=\"http://www.w3.org/2000/svg\" class=\"legend-symbol\" viewBox=\"0 0 18 18\" aria-hidden=\"true\">";
    draw_marker(out, i, 9, 9, 12, "legend");
    out << "</svg>" << labels[i] << " (" << shape_labels[i] << ")</span>";
  }
  out << "</p><p>In each panel, one axis stays constant while the other two are tested. Click a plane value to open a larger map; hover over points in individual SVGs for errors and joint margins.</p>"
      << "<details><summary>Orientation, frame, tolerances, joint margin, and experiment parameters</summary>"
      << "<p>Exact values from summary.json. Each point tests one orientation; radial mode changes yaw with the point's azimuth. Margin statistics describe only the selected joint candidate.</p>"
      << "<pre style=\"overflow:auto;background:white;padding:16px\">" << escape(metadata_text(output)) << "</pre></details>";
  for (int axis = 0; axis < 3; ++axis) {
    out << "<section id=\"" << axis_name(axis) << "\"><h2>Constant " << axis_name(axis) << "</h2><p class=\"planes\">";
    for (const auto& slice : slices) if (slice.axis == axis)
      out << "<a href=\"slices/" << slice.filename << "\">" << axis_name(axis) << "=" << number(slice.fixed, 0) << " mm</a>";
    out << "</p><a href=\"slices_" << axis_name(axis) << ".svg\"><img src=\"slices_" << axis_name(axis) << ".svg\" alt=\"All constant-" << axis_name(axis) << " planes\" loading=\"lazy\"></a></section>";
  }
  out << "<footer><p>The 2D report is static; the 3D viewer uses local JavaScript with embedded data. No internet access, ROS nodes, or hardware connections are required. The model and tolerances apply only to this experiment snapshot. See analysis.md for limitations.</p></footer></body></html>\n";
}

void write_viewer3d(const fs::path& output, const std::vector<Point>& points) {
  auto out = output_file(output / "viewer3d.html");
  out << kViewer3dBeforeData << "{\"points\":[";
  bool first = true;
  for (const auto& point : points) {
    if (!first) out << ',';
    first = false;
    // x/y/z (mm), status index, rcond, margin (deg), position/orientation
    // errors (mm/deg), six witness joint angles (rad), target RPY (deg).
    out << '[';
    for (const double coordinate : point.xyz) {
      json_number(out, coordinate);
      out << ',';
    }
    out << point.status;
    for (const double value : {point.rcond, point.margin, point.position_error, point.orientation_error}) {
      out << ',';
      json_number(out, value);
    }
    for (const double joint : point.q) {
      out << ',';
      json_number(out, joint);
    }
    for (const double angle : point.rpy) {
      out << ',';
      json_number(out, angle);
    }
    out << ']';
  }
  out << "],\"summaryText\":" << json_string(metadata_text(output)) << '}' << kViewer3dAfterData;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    fs::path input;
    double step = 50;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h") {
        std::cout << "Usage: workspace_report --input DIR [--slice-step-mm 50]\n"
                  << "Reads points.csv; writes slices/, slices_X/Y/Z.svg, slices.csv, analysis.md, index.html, viewer3d.html.\n"
                  << "Offline only. Input CSV is never modified.\n";
        return 0;
      }
      if (arg == "--input" && i + 1 < argc) input = argv[++i];
      else if (arg == "--slice-step-mm" && i + 1 < argc) step = numeric(argv[++i], true);
      else throw std::runtime_error("Unknown/incomplete argument: " + arg);
    }
    if (input.empty()) throw std::runtime_error("--input DIR is required");
    if (!std::isfinite(step) || step <= 0) throw std::runtime_error("Slice step must be finite and positive");
    const auto points = read_points(input / "points.csv");
    Counts totals;
    double maximum = 0;
    std::array<double, 3> lower = points.front().xyz, upper = lower;
    for (const auto& point : points) {
      totals.add(point);
      for (int a = 0; a < 3; ++a) {
        lower[a] = std::min(lower[a], point.xyz[a]);
        upper[a] = std::max(upper[a], point.xyz[a]);
        maximum = std::max(maximum, std::abs(point.xyz[a]));
      }
    }
    std::vector<Slice> slices;
    for (int a = 0; a < 3; ++a) {
      const double first = std::ceil(lower[a] / step - 1e-8);
      const double last = std::floor(upper[a] / step + 1e-8);
      if (last - first > 10000 || std::abs(first) > 1e8 || std::abs(last) > 1e8)
        throw std::runtime_error("Too many slice planes; use a larger step");
      for (int index = static_cast<int>(first); index <= static_cast<int>(last); ++index) {
        Slice slice{a, index * step, {}, {}, slice_name(a, index * step)};
        for (const auto& point : points) if (std::abs(point.xyz[a] - slice.fixed) <= 1e-6) {
          slice.points.push_back(&point);
          slice.counts.add(point);
        }
        slices.push_back(std::move(slice));
      }
    }
    if (slices.empty()) throw std::runtime_error("No requested slice planes intersect the sample extent");
    write_slices(input, slices, std::max(step, maximum + step / 2), step);
    write_analysis(input, points, slices, totals, step);
    write_html(input, slices, totals, step);
    write_viewer3d(input, points);
    std::cout << "Report complete: " << points.size() << " points, " << slices.size() << " constant-coordinate planes.\n"
              << "Open " << (input / "index.html") << "\nAnalysis: " << (input / "analysis.md") << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "workspace_report: " << error.what() << '\n';
    return 1;
  }
}
