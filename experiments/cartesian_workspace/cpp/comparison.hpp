// Included inside report.cpp's anonymous namespace; shares validated CSV helpers.
using Tree = boost::property_tree::ptree;

Tree scan_metadata(const fs::path& path, const std::vector<Point>& points) {
  Tree meta;
  boost::property_tree::read_json((path / "summary.json").string(), meta);
  Counts counts;
  for (const auto& p : points) counts.add(p);
  if (meta.get<std::size_t>("points") != counts.total ||
      meta.get<std::size_t>("position_found") != counts.position ||
      meta.get<std::size_t>("pose_found") != counts.pose)
    throw std::runtime_error("Summary totals disagree with CSV: " + path.string());
  for (std::size_t s = 0; s < statuses.size(); ++s)
    if (meta.get<std::size_t>(std::string("counts.") + statuses[s], 0) != counts.counts[s])
      throw std::runtime_error("Summary categories disagree with CSV: " + path.string());
  return meta;
}

struct Overlap {
  std::size_t both = 0, v1_only = 0, v2_only = 0, neither = 0;
  void add(bool a, bool b) {
    if (a && b) ++both;
    else if (a) ++v1_only;
    else if (b) ++v2_only;
    else ++neither;
  }
};

void overlap_json(std::ostream& out, const Overlap& v) {
  out << "{\"both\":" << v.both << ",\"v1_only\":" << v.v1_only
      << ",\"v2_only\":" << v.v2_only << ",\"neither\":" << v.neither << '}';
}

void write_comparison(const fs::path& v1, const fs::path& v2, const fs::path& output) {
  const auto a = read_points(v1 / "points.csv"), b = read_points(v2 / "points.csv");
  const auto ma = scan_metadata(v1, a), mb = scan_metadata(v2, b);
  // Joint limits and kinematic bounds may differ: those belong to the models.
  for (const char* key : {"engine", "radius_mm", "arguments"})
    if (ma.get_child(key) != mb.get_child(key))
      throw std::runtime_error("Unmatched scan setting: " + std::string(key));
  for (const char* key : {"spacing_mm", "samples", "seeds", "iterations", "seed", "orientation",
                          "base_link", "tip_link", "joint_margin_rad", "collision_radius_mm",
                          "position_tolerance_mm", "orientation_tolerance_deg", "length_scale_mm",
                          "singular_threshold", "rpy_deg"})
    ma.get_child(std::string("arguments.") + key);  // Require complete provenance.
  std::map<std::array<double, 3>, const Point*> left, right;
  for (const auto& p : a) if (!left.emplace(p.xyz, &p).second)
    throw std::runtime_error("Duplicate XYZ in v1");
  for (const auto& p : b) if (!right.emplace(p.xyz, &p).second)
    throw std::runtime_error("Duplicate XYZ in v2");
  if (left.size() != right.size()) throw std::runtime_error("Different point grids");
  Overlap positions, poses;
  std::array<std::array<std::size_t, 5>, 5> transitions{};
  Counts ca, cb;
  for (const auto& [xyz, p] : left) {
    const auto it = right.find(xyz);
    if (it == right.end()) throw std::runtime_error("Missing matching XYZ in v2");
    const auto* q = it->second;
    for (std::size_t i = 0; i < 3; ++i)
      if (!std::isfinite(p->rpy[i]) || !std::isfinite(q->rpy[i]) || std::abs(p->rpy[i] - q->rpy[i]) > 1e-8)
        throw std::runtime_error("Missing or different target orientation at matching XYZ");
    positions.add(p->position, q->position); poses.add(p->pose, q->pose);
    ++transitions[p->status][q->status]; ca.add(*p); cb.add(*q);
  }
  // Refuse overwrite, including a scan input directory; validate before writing.
  if (fs::exists(output)) throw std::runtime_error("Output already exists; choose a new comparison directory");
  fs::create_directories(output);
  for (const auto& entry : {std::make_pair(v1, "v1"), std::make_pair(v2, "v2")}) {
    const auto dest = output / entry.second;
    fs::create_directories(dest);
    for (const char* file : {"index.html", "viewer3d.html", "analysis.md", "points.csv", "slices.csv",
                            "summary.json", "slices_X.svg", "slices_Y.svg", "slices_Z.svg"})
      if (fs::exists(entry.first / file)) fs::copy_file(entry.first / file, dest / file);
    if (fs::is_directory(entry.first / "slices"))
      fs::copy(entry.first / "slices", dest / "slices", fs::copy_options::recursive);
  }
  std::ostringstream json;
  json << "{\"points\":" << a.size() << ",\"positions\":"; overlap_json(json, positions);
  json << ",\"poses\":"; overlap_json(json, poses);
  json << ",\"transition_status_order\":[";
  for (std::size_t i=0;i<5;++i) { if(i)json << ',';json << json_string(statuses[i]); }
  json << "],\"transitions\":[";
  for (std::size_t i=0;i<5;++i) {
    if(i)json << ',';
    json << '[';
    for(std::size_t j=0;j<5;++j){if(j)json << ',';json << transitions[i][j];}json << ']';
  }
  json << "],\"v1SummaryText\":" << json_string(metadata_text(v1))
       << ",\"v2SummaryText\":" << json_string(metadata_text(v2)) << '}';
  output_file(output / "comparison.json") << json.str() << '\n';
  auto html = output_file(output / "comparison.html");
  html << kComparisonBeforeData << "{\"statistics\":" << json.str() << ",\"points\":[";
  bool first = true;
  for (const auto& [xyz, p] : left) {
    const auto* q = right.at(xyz);
    if (!first) html << ',';
    first = false;
    html << '[';
    for(double v: xyz){json_number(html,v);html << ',';}
    html << p->status << ',' << q->status;
    for(double v:{p->rcond,q->rcond,p->position_error,q->position_error,p->orientation_error,q->orientation_error}) {
      html << ',';json_number(html,v);
    }
    html << ']';
  }
  html << "]}" << kComparisonAfterData;
  auto md = output_file(output / "comparison.md");
  md << "# OM6DOF v1 / v2 matched workspace comparison\n\n"
     << a.size() << " identical XYZ samples, " << ma.get<double>("arguments.spacing_mm")
     << " mm grid, " << ma.get<double>("radius_mm") << " mm sphere, "
     << ma.get<std::string>("arguments.orientation") << " orientation. Both models were rerun; "
     << "this is not a comparison against the historical public snapshot.\n\n"
     << "| Result | v1 | v2 | Difference (percentage points) |\n|---|---:|---:|---:|\n";
  for (const auto& e : {std::make_pair("Position found", std::make_pair(ca.position,cb.position)),
                       std::make_pair("Pose found (including near-singular)", std::make_pair(ca.pose,cb.pose))})
    md << "| " << e.first << " | " << e.second.first << " (" << number(ca.rate(e.second.first)) << "%) | "
       << e.second.second << " (" << number(cb.rate(e.second.second)) << "%) | "
       << number(cb.rate(e.second.second)-ca.rate(e.second.first)) << " |\n";
  md << "\n| Metric | Both found | v1 only | v2 only | Neither found |\n|---|---:|---:|---:|---:|\n";
  for (const auto& e : {std::make_pair("Position", positions), std::make_pair("Pose", poses)})
    md << "| " << e.first << " | " << e.second.both << " | " << e.second.v1_only << " | "
       << e.second.v2_only << " | " << e.second.neither << " |\n";
  md << "\n## Interpretation and limits\n\n"
     << "- Unresolved is not proven unreachable: the finite multi-start IK search can miss solutions.\n"
     << "- Only one target orientation per XYZ; not all rotations in SO(3).\n"
     << "- Collision checks use chain capsules, not the URDF triangle meshes. The new link3 mesh, "
     << "D435 camera/bracket shape, mass and CoM are not evaluated by this experiment.\n"
     << "- This compares the URDF chain transforms and joint limits, not payload dynamics, "
     << "load capacity, positioning accuracy or safe paths. No table/floor is modelled.\n"
     << "- Near-singularity belongs to the selected witness configuration, not every solution at XYZ.\n"
     << "- Same seed and search budget do not imply the same numerical seed bank across different models.\n"
     << "- Counts are discrete sample coverage, not exact continuous workspace volumes.\n\n"
     << "Open [comparison.html](comparison.html), [v1 report](v1/index.html), or [v2 report](v2/index.html). "
     << "Settings and limits are preserved in [comparison.json](comparison.json).\n";
  auto slices = output_file(output / "comparison_slices.csv");
  slices << "axis,constant_mm,total,v1_position,v2_position,v1_pose,v2_pose,position_delta_pp,pose_delta_pp\n";
  for (std::size_t axis=0;axis<3;++axis) {
    std::map<double,std::pair<Counts,Counts>> planes;
    for (const auto& [xyz,p]:left) {planes[xyz[axis]].first.add(*p);planes[xyz[axis]].second.add(*right.at(xyz));}
    for(const auto& [level,c]:planes)
      slices << "XYZ"[axis] << ',' << level << ',' << c.first.total << ',' << c.first.position << ','
             << c.second.position << ',' << c.first.pose << ',' << c.second.pose << ','
             << number(c.second.rate(c.second.position)-c.first.rate(c.first.position),6) << ','
             << number(c.second.rate(c.second.pose)-c.first.rate(c.first.pose),6) << '\n';
  }
  std::cout << "Matched comparison: " << a.size() << " points. Open " << output / "comparison.html" << '\n';
}
