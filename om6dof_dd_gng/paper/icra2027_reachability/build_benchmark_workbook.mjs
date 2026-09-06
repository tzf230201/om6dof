import fs from "node:fs/promises";
import path from "node:path";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const root = path.resolve("reachability_paper_draft");
const dataDir = path.join(root, "data");
const analysisDir = path.join(dataDir, "analysis_matched_20260823");
const outputDir = path.join(root, "outputs", "01a02189-9587-75a3-b68a-c046bfd27650");
const outputPath = path.join(outputDir, "Reachability_Benchmark_50_Seed_Analysis.xlsx");

function parseCsv(text) {
  const rows = [];
  let row = [];
  let field = "";
  let quoted = false;
  for (let i = 0; i < text.length; i += 1) {
    const char = text[i];
    if (quoted) {
      if (char === '"' && text[i + 1] === '"') {
        field += '"';
        i += 1;
      } else if (char === '"') {
        quoted = false;
      } else {
        field += char;
      }
    } else if (char === '"') {
      quoted = true;
    } else if (char === ",") {
      row.push(field);
      field = "";
    } else if (char === "\n") {
      row.push(field.replace(/\r$/, ""));
      rows.push(row);
      row = [];
      field = "";
    } else {
      field += char;
    }
  }
  if (field.length || row.length) {
    row.push(field.replace(/\r$/, ""));
    rows.push(row);
  }
  return rows;
}

function typedRows(rows) {
  if (!rows.length) return [];
  return rows.map((row, rowIndex) => row.map((value) => {
    if (rowIndex === 0) return value;
    const trimmed = value.trim();
    if (trimmed === "") return null;
    if (trimmed === "True") return true;
    if (trimmed === "False") return false;
    if (/^-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$/.test(trimmed)) return Number(trimmed);
    return value;
  }));
}

function shortReason(value) {
  if (value === "path_ready_exact_validated_preview_only") return "validated after replan";
  if (value === "target_intersection_blocked_or_disconnected") return "blocked/disconnected";
  if (value === "target_intersection_exact_blocked_or_disconnected") return "exact-blocked/disconnected";
  return value;
}

async function loadCsv(filePath) {
  return typedRows(parseCsv(await fs.readFile(filePath, "utf8")));
}

function colLetter(index) {
  let value = index + 1;
  let result = "";
  while (value > 0) {
    const remainder = (value - 1) % 26;
    result = String.fromCharCode(65 + remainder) + result;
    value = Math.floor((value - 1) / 26);
  }
  return result;
}

function setTitle(sheet, title, subtitle, endColumn = "H") {
  sheet.mergeCells(`A1:${endColumn}1`);
  sheet.getRange("A1").values = [[title]];
  sheet.getRange("A1").format = {
    fill: "#17365D",
    font: { bold: true, color: "#FFFFFF", size: 18 },
    rowHeight: 30,
    verticalAlignment: "center",
  };
  sheet.mergeCells(`A2:${endColumn}2`);
  sheet.getRange("A2").values = [[subtitle]];
  sheet.getRange("A2").format = {
    fill: "#D9EAF7",
    font: { color: "#274E75", italic: true, size: 10 },
    rowHeight: 24,
    wrapText: true,
  };
}

function styleHeader(range) {
  range.format = {
    fill: "#2F75B5",
    font: { bold: true, color: "#FFFFFF" },
    wrapText: true,
    verticalAlignment: "center",
    borders: { preset: "outside", style: "thin", color: "#9FBAD0" },
  };
}

function styleDataBlock(range) {
  range.format.borders = {
    insideHorizontal: { style: "thin", color: "#D9E2F3" },
    bottom: { style: "thin", color: "#9FBAD0" },
  };
  range.format.verticalAlignment = "center";
}

const raw = await loadCsv(path.join(dataDir, "reachability_benchmark_50_matched_20260823.csv"));
const statsRows = await loadCsv(path.join(analysisDir, "paired_statistics.csv"));
const successRows = await loadCsv(path.join(analysisDir, "success_statistics.csv"));
const failureRows = await loadCsv(path.join(analysisDir, "failures.csv"));
const replanRows = await loadCsv(path.join(analysisDir, "exact_replans.csv"));
const differenceRows = await loadCsv(path.join(analysisDir, "paired_differences.csv"));
const validation = JSON.parse(await fs.readFile(path.join(analysisDir, "validation.json"), "utf8"));

const statsHeader = statsRows[0];
const statsObjects = statsRows.slice(1).map((row) => Object.fromEntries(statsHeader.map((key, i) => [key, row[i]])));
const successHeader = successRows[0];
const successObjects = successRows.slice(1).map((row) => Object.fromEntries(successHeader.map((key, i) => [key, row[i]])));
const byMetric = new Map(statsObjects.map((row) => [row.metric, row]));
const byScenario = new Map(successObjects.map((row) => [row.scenario, row]));
const failureHeader = failureRows[0];
const failureObjects = failureRows.slice(1).map((row) => Object.fromEntries(failureHeader.map((key, i) => [key, row[i]])));
const replanHeader = replanRows[0];
const replanObjects = replanRows.slice(1).map((row) => Object.fromEntries(replanHeader.map((key, i) => [key, row[i]])));

const workbook = Workbook.create();
const readme = workbook.worksheets.add("README");
const primary = workbook.worksheets.add("Primary Results");
const secondary = workbook.worksheets.add("Secondary Results");
const success = workbook.worksheets.add("Success and Failures");
const differences = workbook.worksheets.add("Paired Differences");
const rawSheet = workbook.worksheets.add("Raw Data");
const methods = workbook.worksheets.add("Methods");

for (const sheet of [readme, primary, secondary, success, differences, rawSheet, methods]) {
  sheet.showGridLines = false;
}

setTitle(readme, "Matched 50-Seed Reachability Benchmark", "GNG versus deterministic Halton/PRM • 800-node budget • preview-only exact collision validation", "H");
readme.getRange("A4:B9").values = [
  ["Validation check", "Workbook value"],
  ["Raw rows", null],
  ["GNG runs", null],
  ["Halton/PRM runs", null],
  ["Rows with process error", null],
  ["Matched target / obstacle", validation.matched_target_xyz && validation.matched_target_joints && validation.matched_obstacle_xyz],
];
readme.getRange("B5").formulas = [["=COUNTA('Raw Data'!A5:A104)"]];
readme.getRange("B6").formulas = [["=COUNTIF('Raw Data'!B5:B104,\"gng\")"]];
readme.getRange("B7").formulas = [["=COUNTIF('Raw Data'!B5:B104,\"halton_prm\")"]];
readme.getRange("B8").formulas = [["=COUNTIF('Raw Data'!AF5:AF104,\"<>\")"]];
styleHeader(readme.getRange("A4:B4"));
styleDataBlock(readme.getRange("A5:B9"));
readme.getRange("A11:H11").merge();
readme.getRange("A11").values = [["Evidence-backed findings"]];
readme.getRange("A11").format = { fill: "#E2F0D9", font: { bold: true, color: "#375623", size: 13 } };
readme.getRange("A12:H17").values = [
  ["• GNG strongly reduced component count: mean 1.70 versus 11.82; paired mean reduction 10.12 (95% bootstrap CI 9.22–11.02).", null, null, null, null, null, null, null],
  ["• GNG construction was slower: 1723.78 ms versus 1413.57 ms; paired difference −310.20 ms when positive is defined as favoring GNG.", null, null, null, null, null, null, null],
  ["• Clear-scene planning time did not differ after Holm correction: adjusted p = 0.566.", null, null, null, null, null, null, null],
  ["• Conditional on both methods succeeding, dynamic planning time did not differ after correction: adjusted p = 0.383 (n = 42 pairs).", null, null, null, null, null, null, null],
  ["• Dynamic success favored Halton/PRM: GNG 42/50 versus Halton/PRM 50/50; paired risk difference −16.0 pp (95% CI −26.0 to −6.0), exact McNemar Holm-adjusted p = 0.0234.", null, null, null, null, null, null, null],
  ["• Interpretation: this fixed-scene experiment exposes a connectivity–robustness trade-off; it does not establish generalization or physical execution safety.", null, null, null, null, null, null, null],
];
for (let row = 12; row <= 17; row += 1) readme.mergeCells(`A${row}:H${row}`);
readme.getRange("A12:H17").format = { wrapText: true, rowHeight: 31, verticalAlignment: "center" };
readme.getRange("A19:H22").values = [
  ["Reproducibility note", null, null, null, null, null, null, null],
  ["Seeds", "0–49, paired by seed-specific Halton offset", null, null, null, null, null, null],
  ["Matched task", `target q=${JSON.stringify(validation.target_joints)}; obstacle xyz=${JSON.stringify(validation.obstacle_xyz)}`, null, null, null, null, null, null],
  ["Measurement boundary", "Background hardware/perception processes remained active; ROS benchmark traffic used domains 20–119 and method order was counterbalanced.", null, null, null, null, null, null],
];
readme.mergeCells("A19:H19");
readme.getRange("A19").format = { fill: "#FFF2CC", font: { bold: true, color: "#7F6000" } };
for (let row = 20; row <= 22; row += 1) readme.mergeCells(`B${row}:H${row}`);
readme.getRange("A20:H22").format = { wrapText: true, rowHeight: 28 };
readme.getRange("A1:H22").format.font.name = "Aptos";
readme.getRange("A:A").format.columnWidth = 24;
readme.getRange("B:H").format.columnWidth = 16;

setTitle(primary, "Primary Paired Results", "Positive paired effects favor GNG; timing comparisons use Halton/PRM − GNG. Dynamic time is conditional on joint success.", "R");
const primaryMetrics = ["components", "build_time_ms", "clear_planning_time_ms", "dynamic_planning_time_ms"];
const primaryTable = [["Metric", "n", "GNG mean", "Halton mean", "Mean paired effect", "95% CI", "Rank-biserial r", "Holm p", "Population"]];
for (const metric of primaryMetrics) {
  const row = byMetric.get(metric);
  primaryTable.push([
    row.label,
    row.n_pairs,
    row.gng_mean,
    row.halton_mean,
    row.mean_paired_difference,
    `[${Number(row.mean_difference_ci_low).toFixed(2)}, ${Number(row.mean_difference_ci_high).toFixed(2)}]`,
    row.rank_biserial,
    row.p_holm_primary_family,
    row.analysis_population,
  ]);
}
const dynamic = byScenario.get("dynamic");
primaryTable.push([
  "Dynamic planning success",
  dynamic.n_pairs,
  dynamic.gng_success_rate,
  dynamic.halton_success_rate,
  dynamic.paired_risk_difference_pp / 100,
  `[${Number(dynamic.risk_difference_ci_low_pp).toFixed(1)}, ${Number(dynamic.risk_difference_ci_high_pp).toFixed(1)}] pp`,
  dynamic.matched_odds_ratio_haldane,
  dynamic.p_holm_primary_family,
  "all paired seeds; exact McNemar",
]);
primary.getRange(`A4:I${3 + primaryTable.length}`).values = primaryTable;
styleHeader(primary.getRange("A4:I4"));
styleDataBlock(primary.getRange(`A5:I${3 + primaryTable.length}`));
primary.getRange("B5:B9").format.numberFormat = "0";
primary.getRange("C5:E8").format.numberFormat = "0.00";
primary.getRange("G5:H9").format.numberFormat = "0.000";
primary.getRange("C9:E9").format.numberFormat = "0.0%";
primary.getRange("A4:I9").format.wrapText = true;
primary.freezePanes.freezeRows(4);
primary.getRange("A:A").format.columnWidth = 28;
primary.getRange("B:B").format.columnWidth = 8;
primary.getRange("C:E").format.columnWidth = 15;
primary.getRange("F:F").format.columnWidth = 20;
primary.getRange("G:H").format.columnWidth = 14;
primary.getRange("I:I").format.columnWidth = 26;

primary.getRange("K4:L6").values = [["Method", "Components"], ["GNG", null], ["Halton/PRM", null]];
primary.getRange("L5").formulas = [["=C5"]];
primary.getRange("L6").formulas = [["=D5"]];
primary.getRange("K9:L11").values = [["Method", "Dynamic success"], ["GNG", null], ["Halton/PRM", null]];
primary.getRange("L10").formulas = [["=C9"]];
primary.getRange("L11").formulas = [["=D9"]];
primary.getRange("L10:L11").format.numberFormat = "0%";
styleHeader(primary.getRange("K4:L4"));
styleHeader(primary.getRange("K9:L9"));
primary.getRange("A4:I4").format.rowHeight = 36;
primary.getRange("E:E").format.columnWidth = 18;
primary.getRange("F:F").format.columnWidth = 20;
primary.getRange("K:K").format.columnWidth = 16;
primary.getRange("L:L").format.columnWidth = 18;
const componentChart = primary.charts.add("bar", primary.getRange("K4:L6"));
componentChart.title = "GNG produces fewer connected components";
componentChart.hasLegend = false;
componentChart.yAxis = { numberFormatCode: "0.0" };
componentChart.setPosition("K13", "R26");
const successChart = primary.charts.add("bar", primary.getRange("K9:L11"));
successChart.title = "Dynamic success favors Halton/PRM";
successChart.hasLegend = false;
successChart.yAxis = { numberFormatCode: "0%", min: 0, max: 1 };
successChart.setPosition("S13", "Z26");

setTitle(secondary, "Secondary Paired Results", "Exploratory outcomes; raw p-values are not included in the five-test primary Holm family.", "L");
const secondaryObjects = statsObjects.filter((row) => !row.primary);
const secondaryTable = [["Metric", "n", "GNG mean", "GNG SD", "Halton mean", "Halton SD", "Mean paired effect", "95% CI", "Median effect", "Rank-biserial r", "Raw p", "Population"]];
for (const row of secondaryObjects) {
  secondaryTable.push([
    row.label,
    row.n_pairs,
    row.gng_mean,
    row.gng_sd,
    row.halton_mean,
    row.halton_sd,
    row.mean_paired_difference,
    `[${Number(row.mean_difference_ci_low).toFixed(2)}, ${Number(row.mean_difference_ci_high).toFixed(2)}]`,
    row.median_paired_difference,
    row.rank_biserial,
    row.p_raw,
    row.analysis_population,
  ]);
}
secondary.getRange(`A4:L${3 + secondaryTable.length}`).values = secondaryTable;
styleHeader(secondary.getRange("A4:L4"));
styleDataBlock(secondary.getRange(`A5:L${3 + secondaryTable.length}`));
secondary.getRange(`B5:B${3 + secondaryTable.length}`).format.numberFormat = "0";
secondary.getRange(`C5:G${3 + secondaryTable.length}`).format.numberFormat = "0.00";
secondary.getRange(`I5:K${3 + secondaryTable.length}`).format.numberFormat = "0.000";
secondary.getRange(`A4:L${3 + secondaryTable.length}`).format.wrapText = true;
secondary.freezePanes.freezeRows(4);
secondary.getRange("A:A").format.columnWidth = 30;
secondary.getRange("B:B").format.columnWidth = 8;
secondary.getRange("C:G").format.columnWidth = 14;
secondary.getRange("H:H").format.columnWidth = 20;
secondary.getRange("I:K").format.columnWidth = 14;
secondary.getRange("L:L").format.columnWidth = 27;

setTitle(success, "Success, Failure, and Exact Replanning", "Failures are retained; no seed was discarded or rerun. Exact replans demonstrate the reject-edge → graph-search loop.", "L");
const successTable = [["Scenario", "n", "GNG success", "Halton success", "GNG rate", "Halton rate", "GNG-only", "Halton-only", "Risk diff. (pp)", "95% CI (pp)", "McNemar p", "Holm p"]];
for (const row of successObjects) {
  successTable.push([
    row.scenario,
    row.n_pairs,
    row.gng_successes,
    row.halton_successes,
    row.gng_success_rate,
    row.halton_success_rate,
    row.gng_only_success,
    row.halton_only_success,
    row.paired_risk_difference_pp,
    `[${Number(row.risk_difference_ci_low_pp).toFixed(1)}, ${Number(row.risk_difference_ci_high_pp).toFixed(1)}]`,
    row.mcnemar_exact_p_raw,
    row.p_holm_primary_family,
  ]);
}
success.getRange("A4:L6").values = successTable;
styleHeader(success.getRange("A4:L4"));
styleDataBlock(success.getRange("A5:L6"));
success.getRange("B5:D6").format.numberFormat = "0";
success.getRange("E5:F6").format.numberFormat = "0.0%";
success.getRange("G5:I6").format.numberFormat = "0.0";
success.getRange("K5:L6").format.numberFormat = "0.000";
success.getRange("A4:L4").format.rowHeight = 34;
success.getRange("A9:F9").values = [["Failed cases", null, null, null, null, null]];
success.mergeCells("A9:F9");
success.getRange("A9").format = { fill: "#FCE4D6", font: { bold: true, color: "#C00000" } };
const failureTable = [["Seed", "Method", "Clear valid", "Dynamic valid", "Dynamic failure reason", "Exact replans"]];
for (const row of failureObjects) {
  failureTable.push([row.seed, row.method, row.clear_valid, row.dynamic_valid, shortReason(row.dynamic_reason), row.dynamic_exact_replans]);
}
success.getRange(`A10:F${9 + failureTable.length}`).values = failureTable;
styleHeader(success.getRange("A10:F10"));
styleDataBlock(success.getRange(`A11:F${9 + failureTable.length}`));
const replanStart = 12 + failureTable.length;
success.mergeCells(`A${replanStart}:F${replanStart}`);
success.getRange(`A${replanStart}`).values = [["Exact-replan cases"]];
success.getRange(`A${replanStart}`).format = { fill: "#E2F0D9", font: { bold: true, color: "#375623" } };
const replanTable = [["Seed", "Method", "Clear replans", "Dynamic replans", "Dynamic valid", "Outcome"]];
for (const row of replanObjects) {
  replanTable.push([row.seed, row.method, row.clear_exact_replans, row.dynamic_exact_replans, row.dynamic_valid, shortReason(row.dynamic_reason)]);
}
success.getRange(`A${replanStart + 1}:F${replanStart + replanTable.length}`).values = replanTable;
styleHeader(success.getRange(`A${replanStart + 1}:F${replanStart + 1}`));
styleDataBlock(success.getRange(`A${replanStart + 2}:F${replanStart + replanTable.length}`));
success.freezePanes.freezeRows(4);
success.getRange("A:A").format.columnWidth = 11;
success.getRange("B:D").format.columnWidth = 14;
success.getRange("E:F").format.columnWidth = 16;
success.getRange("G:I").format.columnWidth = 14;
success.getRange("J:J").format.columnWidth = 17;
success.getRange("K:L").format.columnWidth = 13;
success.getRange("E:E").format.columnWidth = 24;
success.getRange("F:F").format.columnWidth = 18;
success.getRange(`A10:F${replanStart + replanTable.length}`).format.wrapText = true;

setTitle(differences, "Seed-Level Paired Differences", "Positive values favor GNG under each metric's predefined direction; blanks denote pairs excluded by joint-success conditioning.", "L");
differences.getRange(`A4:${colLetter(differenceRows[0].length - 1)}${3 + differenceRows.length}`).values = differenceRows;
const diffEndCol = colLetter(differenceRows[0].length - 1);
styleHeader(differences.getRange(`A4:${diffEndCol}4`));
styleDataBlock(differences.getRange(`A5:${diffEndCol}${3 + differenceRows.length}`));
differences.getRange(`B5:${diffEndCol}${3 + differenceRows.length}`).format.numberFormat = "0.00";
differences.getRange(`B5:${diffEndCol}${3 + differenceRows.length}`).conditionalFormats.add("colorScale", {
  colors: ["#F8696B", "#FFEB84", "#63BE7B"],
  thresholds: ["min", { type: "percentile", value: 50 }, "max"],
});
differences.freezePanes.freezeRows(4);
differences.freezePanes.freezeColumns(1);
differences.getRange("A:A").format.columnWidth = 9;
differences.getRange(`B:${diffEndCol}`).format.columnWidth = 18;
differences.getRange(`A4:${diffEndCol}4`).format.wrapText = true;

setTitle(rawSheet, "Raw Matched Benchmark Data", "Unmodified 100-row CSV snapshot; one row per method × seed. Use filters to inspect failures, timings, and exact replans.", "P");
rawSheet.getRange(`A4:AF${3 + raw.length}`).values = raw;
styleHeader(rawSheet.getRange("A4:AF4"));
styleDataBlock(rawSheet.getRange(`A5:AF${3 + raw.length}`));
rawSheet.tables.add(`A4:AF${3 + raw.length}`, true, "RawBenchmarkTable");
rawSheet.freezePanes.freezeRows(4);
rawSheet.freezePanes.freezeColumns(3);
rawSheet.getRange("A:A").format.columnWidth = 10;
rawSheet.getRange("B:B").format.columnWidth = 14;
rawSheet.getRange("C:H").format.columnWidth = 13;
rawSheet.getRange("I:K").format.columnWidth = 40;
rawSheet.getRange("L:AF").format.columnWidth = 14;
rawSheet.getRange("M:M").format.columnWidth = 37;
rawSheet.getRange("W:W").format.columnWidth = 43;
rawSheet.getRange("H5:H103").format.numberFormat = "0.00";
rawSheet.getRange("P5:P103").format.numberFormat = "0.000";
rawSheet.getRange("T5:T103").format.numberFormat = "0.000";
rawSheet.getRange("Z5:Z103").format.numberFormat = "0.000";
rawSheet.getRange("AD5:AD103").format.numberFormat = "0.000";

setTitle(methods, "Methods and Statistical Definitions", "Use this sheet with Raw Data and the reproducible Python analysis script to audit every reported result.", "H");
const methodRows = [
  ["Item", "Definition / implementation"],
  ["Experimental unit", "One deterministic seed, paired across GNG and Halton/PRM."],
  ["Seed schedule", "Seeds 0–49. The same seed-specific Halton start offset supplies GNG training samples and the Halton/PRM nodes."],
  ["Counterbalancing", "Even seeds run GNG then Halton; odd seeds reverse the order to reduce monotonic host-load or thermal bias."],
  ["Matched task", `Start q=[0,0,0,0,0,0]; target q=${JSON.stringify(validation.target_joints)}; target xyz=${JSON.stringify(validation.target_xyz)}; obstacle xyz=${JSON.stringify(validation.obstacle_xyz)}.`],
  ["Primary family", "Connected components, build time, clear planning time, dynamic planning time conditional on joint success, and dynamic success."],
  ["Paired difference", "For lower-is-better outcomes: Halton/PRM − GNG. For higher-is-better outcomes: GNG − Halton/PRM. Positive values favor GNG."],
  ["Confidence intervals", "95% percentile paired bootstrap, 20,000 resamples of seed pairs; rank-biserial intervals use 10,000 paired resamples."],
  ["Continuous/count tests", "Two-sided Wilcoxon signed-rank with Pratt handling of zero differences. Rank-biserial correlation is the paired nonparametric effect size."],
  ["Success test", "Exact McNemar/binomial test on discordant pairs. Effect magnitude is paired risk difference with bootstrap CI; matched odds ratio uses Haldane 0.5 correction."],
  ["Multiplicity", "Holm adjustment across the five prespecified primary tests. Secondary p-values are exploratory and unadjusted."],
  ["Failure handling", "Failures are outcomes, not missing values. Dynamic timing is analyzed only for seeds where both methods succeed and is reported alongside success."],
  ["Execution safety", "The benchmark publishes only synthetic JointState, typed environment graph, and preview paths in isolated ROS domains. It does not create a controller action client."],
  ["External validity", "This fixed robot, target, and obstacle measures seed-level roadmap variability; it does not establish scene-level generalization or physical execution performance."],
  ["Host condition", "AGX hardware/perception processes remained active. This is disclosed because host contention can affect wall-clock measurements."],
];
methods.getRange(`A4:B${3 + methodRows.length}`).values = methodRows;
styleHeader(methods.getRange("A4:B4"));
styleDataBlock(methods.getRange(`A5:B${3 + methodRows.length}`));
methods.getRange(`A4:B${3 + methodRows.length}`).format.wrapText = true;
methods.getRange(`A5:B${3 + methodRows.length}`).format.rowHeight = 34;
methods.getRange("A:A").format.columnWidth = 23;
methods.getRange("B:B").format.columnWidth = 95;
methods.freezePanes.freezeRows(4);

for (const sheet of [readme, primary, secondary, success, differences, rawSheet, methods]) {
  const used = sheet.getUsedRange();
  if (used) used.format.font.name = "Aptos";
}

await fs.mkdir(outputDir, { recursive: true });
const xlsx = await SpreadsheetFile.exportXlsx(workbook);
await xlsx.save(outputPath);

const previewSpecs = [
  [readme, "A1:H22", "README"],
  [primary, "A1:Z26", "Primary_Results"],
  [secondary, `A1:L${3 + secondaryTable.length}`, "Secondary_Results"],
  [success, `A1:L${replanStart + replanTable.length}`, "Success_Failures"],
  [differences, "A1:L18", "Paired_Differences"],
  [rawSheet, "A1:P14", "Raw_Data"],
  [methods, `A1:H${3 + methodRows.length}`, "Methods"],
];
for (const [sheet, range, name] of previewSpecs) {
  const preview = await workbook.render({ sheetName: sheet.name, range, scale: 1, format: "png" });
  await fs.writeFile(path.join(outputDir, `${name}.png`), new Uint8Array(await preview.arrayBuffer()));
}

console.log((await workbook.inspect({
  kind: "workbook,sheet,table,drawing",
  maxChars: 8000,
  tableMaxRows: 8,
  tableMaxCols: 12,
})).ndjson);
console.log((await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
  options: { useRegex: true, maxResults: 200 },
  summary: "final formula error scan",
})).ndjson);
console.log(outputPath);
