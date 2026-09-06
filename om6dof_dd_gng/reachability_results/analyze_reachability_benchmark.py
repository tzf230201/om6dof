#!/usr/bin/env python3
"""Paired statistical analysis for the matched reachability benchmark."""

import argparse
import json
import math
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats


METHODS = ("gng", "halton_prm")
PRIMARY_METRICS = {
    "components",
    "build_time_ms",
    "clear_planning_time_ms",
    "dynamic_planning_time_ms",
}


def parse_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() == "true"


def percentile_ci(samples, confidence=0.95):
    alpha = (1.0 - confidence) / 2.0
    return tuple(float(x) for x in np.quantile(samples, [alpha, 1.0 - alpha]))


def bootstrap_stat_ci(values, statistic, rng, resamples=20000):
    values = np.asarray(values, dtype=float)
    indices = rng.integers(0, len(values), size=(resamples, len(values)))
    sampled = values[indices]
    if statistic == "mean":
        estimates = np.mean(sampled, axis=1)
    elif statistic == "median":
        estimates = np.median(sampled, axis=1)
    else:
        raise ValueError(statistic)
    return percentile_ci(estimates)


def rank_biserial(values):
    values = np.asarray(values, dtype=float)
    nonzero = values[values != 0.0]
    if len(nonzero) == 0:
        return 0.0
    ranks = stats.rankdata(np.abs(nonzero), method="average")
    positive = float(np.sum(ranks[nonzero > 0.0]))
    negative = float(np.sum(ranks[nonzero < 0.0]))
    return (positive - negative) / (positive + negative)


def bootstrap_rank_biserial_ci(values, rng, resamples=10000):
    values = np.asarray(values, dtype=float)
    estimates = np.empty(resamples, dtype=float)
    for index in range(resamples):
        sample = values[rng.integers(0, len(values), size=len(values))]
        estimates[index] = rank_biserial(sample)
    return percentile_ci(estimates)


def describe(values):
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "sd": float(np.std(values, ddof=1)) if len(values) > 1 else math.nan,
        "median": float(np.median(values)),
        "q1": float(np.quantile(values, 0.25)),
        "q3": float(np.quantile(values, 0.75)),
    }


def holm_adjust(p_values):
    p_values = np.asarray(p_values, dtype=float)
    order = np.argsort(p_values)
    adjusted = np.empty_like(p_values)
    running = 0.0
    count = len(p_values)
    for rank, original_index in enumerate(order):
        candidate = (count - rank) * p_values[original_index]
        running = max(running, candidate)
        adjusted[original_index] = min(1.0, running)
    return adjusted


def paired_metric(df, metric, direction, label, rng_seed, require_success=None):
    values = df.pivot(index="seed", columns="method", values=metric).sort_index()
    eligible = values.notna().all(axis=1)
    if require_success:
        valid = df.pivot(index="seed", columns="method", values=require_success).sort_index()
        eligible &= valid[METHODS[0]].astype(bool) & valid[METHODS[1]].astype(bool)
    values = values.loc[eligible]
    gng = values["gng"].astype(float).to_numpy()
    halton = values["halton_prm"].astype(float).to_numpy()
    if direction == "lower":
        difference = halton - gng
        definition = "Halton/PRM - GNG; positive favors GNG"
    elif direction == "higher":
        difference = gng - halton
        definition = "GNG - Halton/PRM; positive favors GNG"
    else:
        raise ValueError(direction)

    rng = np.random.default_rng(rng_seed)
    mean_ci = bootstrap_stat_ci(difference, "mean", rng)
    median_ci = bootstrap_stat_ci(difference, "median", rng)
    effect = rank_biserial(difference)
    effect_ci = bootstrap_rank_biserial_ci(difference, rng)
    if np.all(difference == 0.0):
        wilcoxon_stat, p_value = 0.0, 1.0
    else:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            result = stats.wilcoxon(
                difference,
                zero_method="pratt",
                correction=False,
                alternative="two-sided",
                mode="auto",
            )
        wilcoxon_stat, p_value = float(result.statistic), float(result.pvalue)
    gng_desc = describe(gng)
    halton_desc = describe(halton)
    return {
        "metric": metric,
        "label": label,
        "direction": direction,
        "difference_definition": definition,
        "analysis_population": (
            f"both methods {require_success}=true" if require_success else "all paired seeds"
        ),
        "n_pairs": len(difference),
        "gng_mean": gng_desc["mean"],
        "gng_sd": gng_desc["sd"],
        "gng_median": gng_desc["median"],
        "gng_q1": gng_desc["q1"],
        "gng_q3": gng_desc["q3"],
        "halton_mean": halton_desc["mean"],
        "halton_sd": halton_desc["sd"],
        "halton_median": halton_desc["median"],
        "halton_q1": halton_desc["q1"],
        "halton_q3": halton_desc["q3"],
        "mean_paired_difference": float(np.mean(difference)),
        "mean_difference_ci_low": mean_ci[0],
        "mean_difference_ci_high": mean_ci[1],
        "median_paired_difference": float(np.median(difference)),
        "median_difference_ci_low": median_ci[0],
        "median_difference_ci_high": median_ci[1],
        "rank_biserial": effect,
        "rank_biserial_ci_low": effect_ci[0],
        "rank_biserial_ci_high": effect_ci[1],
        "wilcoxon_statistic": wilcoxon_stat,
        "p_raw": p_value,
        "primary": metric in PRIMARY_METRICS,
    }


def success_result(df, scenario, rng_seed):
    metric = f"{scenario}_valid"
    values = df.pivot(index="seed", columns="method", values=metric).sort_index()
    gng = values["gng"].astype(bool).to_numpy()
    halton = values["halton_prm"].astype(bool).to_numpy()
    both_success = int(np.sum(gng & halton))
    gng_only = int(np.sum(gng & ~halton))
    halton_only = int(np.sum(~gng & halton))
    both_fail = int(np.sum(~gng & ~halton))
    discordant = gng_only + halton_only
    p_value = (
        float(stats.binomtest(gng_only, discordant, 0.5, alternative="two-sided").pvalue)
        if discordant
        else 1.0
    )
    difference = gng.astype(float) - halton.astype(float)
    rng = np.random.default_rng(rng_seed)
    risk_ci = bootstrap_stat_ci(difference * 100.0, "mean", rng)
    return {
        "scenario": scenario,
        "n_pairs": len(values),
        "gng_successes": int(np.sum(gng)),
        "halton_successes": int(np.sum(halton)),
        "gng_success_rate": float(np.mean(gng)),
        "halton_success_rate": float(np.mean(halton)),
        "both_success": both_success,
        "gng_only_success": gng_only,
        "halton_only_success": halton_only,
        "both_fail": both_fail,
        "paired_risk_difference_pp": float(np.mean(difference) * 100.0),
        "risk_difference_ci_low_pp": risk_ci[0],
        "risk_difference_ci_high_pp": risk_ci[1],
        "matched_odds_ratio_haldane": (gng_only + 0.5) / (halton_only + 0.5),
        "mcnemar_exact_p_raw": p_value,
        "primary": scenario == "dynamic",
    }


def validate(df):
    expected_seeds = set(range(50))
    method_counts = df["method"].value_counts().to_dict()
    seed_methods = df.groupby("seed")["method"].apply(set)
    target_values = df["target_xyz"].apply(json.loads)
    target_joint_values = df["target_joints"].apply(json.loads)
    obstacle_values = df["obstacle_xyz"].apply(json.loads)

    target_ref = np.asarray(target_values.iloc[0], dtype=float)
    joints_ref = np.asarray(target_joint_values.iloc[0], dtype=float)
    obstacle_ref = np.asarray(obstacle_values.iloc[0], dtype=float)
    checks = {
        "row_count": int(len(df)),
        "method_counts": {key: int(value) for key, value in method_counts.items()},
        "seed_count": int(df["seed"].nunique()),
        "seed_range": [int(df["seed"].min()), int(df["seed"].max())],
        "all_expected_seeds_present": set(df["seed"]) == expected_seeds,
        "each_seed_has_both_methods": bool(seed_methods.apply(lambda value: value == set(METHODS)).all()),
        "run_indices_complete": set(df["run_index"]) == set(range(100)),
        "errors": int(df["error"].fillna("").astype(str).str.len().gt(0).sum()),
        "nodes_all_800": bool((df["nodes"] == 800).all()),
        "matched_target_xyz": bool(
            all(np.allclose(np.asarray(value, dtype=float), target_ref) for value in target_values)
        ),
        "matched_target_joints": bool(
            all(np.allclose(np.asarray(value, dtype=float), joints_ref) for value in target_joint_values)
        ),
        "matched_obstacle_xyz": bool(
            all(np.allclose(np.asarray(value, dtype=float), obstacle_ref) for value in obstacle_values)
        ),
        "target_xyz": target_ref.tolist(),
        "target_joints": joints_ref.tolist(),
        "obstacle_xyz": obstacle_ref.tolist(),
    }
    required_true = [
        checks["row_count"] == 100,
        checks["method_counts"] == {"gng": 50, "halton_prm": 50},
        checks["seed_count"] == 50,
        checks["all_expected_seeds_present"],
        checks["each_seed_has_both_methods"],
        checks["run_indices_complete"],
        checks["errors"] == 0,
        checks["nodes_all_800"],
        checks["matched_target_xyz"],
        checks["matched_target_joints"],
        checks["matched_obstacle_xyz"],
    ]
    checks["valid"] = all(required_true)
    if not checks["valid"]:
        raise RuntimeError(f"benchmark validation failed: {checks}")
    return checks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv")
    parser.add_argument("output_dir")
    args = parser.parse_args()
    input_path = Path(args.input_csv).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(input_path)
    for column in [
        "clear_valid",
        "clear_exact_valid",
        "dynamic_valid",
        "dynamic_exact_valid",
    ]:
        df[column] = df[column].map(parse_bool)
    checks = validate(df)

    metric_specs = [
        ("components", "lower", "Connected components", None),
        ("edges", "higher", "Validated roadmap edges", None),
        ("build_time_ms", "lower", "Roadmap build time (ms)", None),
        ("clear_planning_time_ms", "lower", "Clear-scene planning time (ms)", "clear_valid"),
        ("clear_exact_checks", "lower", "Clear exact state checks", "clear_valid"),
        ("clear_exact_time_ms", "lower", "Clear exact validation time (ms)", "clear_valid"),
        ("clear_path_nodes", "lower", "Clear path nodes", "clear_valid"),
        ("dynamic_planning_time_ms", "lower", "Dynamic planning time (ms)", "dynamic_valid"),
        ("dynamic_exact_checks", "lower", "Dynamic exact state checks", "dynamic_valid"),
        ("dynamic_exact_time_ms", "lower", "Dynamic exact validation time (ms)", "dynamic_valid"),
        ("dynamic_path_nodes", "lower", "Dynamic path nodes", "dynamic_valid"),
    ]
    results = []
    for index, (metric, direction, label, require_success) in enumerate(metric_specs):
        results.append(
            paired_metric(
                df,
                metric,
                direction,
                label,
                rng_seed=20260823 + index * 101,
                require_success=require_success,
            )
        )
    results_df = pd.DataFrame(results)
    success_df = pd.DataFrame(
        [success_result(df, "clear", 20261823), success_result(df, "dynamic", 20262823)]
    )

    primary_refs = []
    primary_p_values = []
    for index, row in results_df.iterrows():
        if row["primary"]:
            primary_refs.append(("metric", index))
            primary_p_values.append(float(row["p_raw"]))
    for index, row in success_df.iterrows():
        if row["primary"]:
            primary_refs.append(("success", index))
            primary_p_values.append(float(row["mcnemar_exact_p_raw"]))
    adjusted = holm_adjust(primary_p_values)
    results_df["p_holm_primary_family"] = np.nan
    success_df["p_holm_primary_family"] = np.nan
    for reference, value in zip(primary_refs, adjusted):
        kind, index = reference
        if kind == "metric":
            results_df.loc[index, "p_holm_primary_family"] = value
        else:
            success_df.loc[index, "p_holm_primary_family"] = value

    failures = df.loc[
        (~df["clear_valid"]) | (~df["dynamic_valid"]),
        [
            "seed",
            "method",
            "clear_valid",
            "clear_reason",
            "dynamic_valid",
            "dynamic_reason",
            "clear_exact_replans",
            "dynamic_exact_replans",
        ],
    ].copy()
    replans = df.loc[
        (df["clear_exact_replans"] > 0) | (df["dynamic_exact_replans"] > 0),
        [
            "seed",
            "method",
            "clear_exact_replans",
            "dynamic_exact_replans",
            "clear_valid",
            "dynamic_valid",
            "dynamic_reason",
        ],
    ].copy()

    paired = df.pivot(index="seed", columns="method")
    paired_output = pd.DataFrame(index=sorted(df["seed"].unique()))
    paired_output.index.name = "seed"
    for metric, direction, _, require_success in metric_specs:
        gng = paired[metric]["gng"]
        halton = paired[metric]["halton_prm"]
        if direction == "lower":
            difference = halton - gng
        else:
            difference = gng - halton
        if require_success:
            valid = paired[require_success]["gng"].astype(bool) & paired[require_success][
                "halton_prm"
            ].astype(bool)
            difference = difference.where(valid)
        paired_output[f"{metric}_paired_difference"] = difference

    validation_path = output_dir / "validation.json"
    validation_path.write_text(json.dumps(checks, indent=2), encoding="utf-8")
    results_df.to_csv(output_dir / "paired_statistics.csv", index=False)
    success_df.to_csv(output_dir / "success_statistics.csv", index=False)
    failures.to_csv(output_dir / "failures.csv", index=False)
    replans.to_csv(output_dir / "exact_replans.csv", index=False)
    paired_output.to_csv(output_dir / "paired_differences.csv")

    primary_rows = results_df[results_df["primary"]].set_index("metric")
    dynamic_success = success_df.set_index("scenario").loc["dynamic"]
    report = f"""# Matched 50-seed reachability benchmark

## Protocol and validation

- 50 deterministic paired seeds per method (100 roadmap builds).
- Equal 800-node budget; seed-specific Halton offset shared by GNG and Halton/PRM.
- Method order counterbalanced within consecutive seed pairs.
- Identical start configuration, SRDF home target, and world-frame obstacle for both methods.
- Clear and dynamic queries are preview-only and exact-validated; no controller action is created.
- Raw rows: {checks['row_count']}; process errors: {checks['errors']}.

## Primary results

- Connected components: GNG mean {primary_rows.loc['components', 'gng_mean']:.2f} versus Halton/PRM {primary_rows.loc['components', 'halton_mean']:.2f}; mean paired reduction {primary_rows.loc['components', 'mean_paired_difference']:.2f} (95% paired-bootstrap CI {primary_rows.loc['components', 'mean_difference_ci_low']:.2f} to {primary_rows.loc['components', 'mean_difference_ci_high']:.2f}), rank-biserial r={primary_rows.loc['components', 'rank_biserial']:.3f}, Holm-adjusted p={primary_rows.loc['components', 'p_holm_primary_family']:.3g}.
- Build time: GNG mean {primary_rows.loc['build_time_ms', 'gng_mean']:.2f} ms versus Halton/PRM {primary_rows.loc['build_time_ms', 'halton_mean']:.2f} ms; paired difference (Halton-GNG) {primary_rows.loc['build_time_ms', 'mean_paired_difference']:.2f} ms (95% CI {primary_rows.loc['build_time_ms', 'mean_difference_ci_low']:.2f} to {primary_rows.loc['build_time_ms', 'mean_difference_ci_high']:.2f}), r={primary_rows.loc['build_time_ms', 'rank_biserial']:.3f}, Holm-adjusted p={primary_rows.loc['build_time_ms', 'p_holm_primary_family']:.3g}.
- Clear planning time: GNG mean {primary_rows.loc['clear_planning_time_ms', 'gng_mean']:.2f} ms versus Halton/PRM {primary_rows.loc['clear_planning_time_ms', 'halton_mean']:.2f} ms; paired difference {primary_rows.loc['clear_planning_time_ms', 'mean_paired_difference']:.2f} ms (95% CI {primary_rows.loc['clear_planning_time_ms', 'mean_difference_ci_low']:.2f} to {primary_rows.loc['clear_planning_time_ms', 'mean_difference_ci_high']:.2f}), r={primary_rows.loc['clear_planning_time_ms', 'rank_biserial']:.3f}, Holm-adjusted p={primary_rows.loc['clear_planning_time_ms', 'p_holm_primary_family']:.3g}.
- Dynamic planning time among seeds where both methods succeeded (n={int(primary_rows.loc['dynamic_planning_time_ms', 'n_pairs'])}): GNG mean {primary_rows.loc['dynamic_planning_time_ms', 'gng_mean']:.2f} ms versus Halton/PRM {primary_rows.loc['dynamic_planning_time_ms', 'halton_mean']:.2f} ms; paired difference {primary_rows.loc['dynamic_planning_time_ms', 'mean_paired_difference']:.2f} ms (95% CI {primary_rows.loc['dynamic_planning_time_ms', 'mean_difference_ci_low']:.2f} to {primary_rows.loc['dynamic_planning_time_ms', 'mean_difference_ci_high']:.2f}), r={primary_rows.loc['dynamic_planning_time_ms', 'rank_biserial']:.3f}, Holm-adjusted p={primary_rows.loc['dynamic_planning_time_ms', 'p_holm_primary_family']:.3g}.
- Dynamic success: GNG {int(dynamic_success['gng_successes'])}/50 versus Halton/PRM {int(dynamic_success['halton_successes'])}/50; paired risk difference {dynamic_success['paired_risk_difference_pp']:.1f} percentage points (95% CI {dynamic_success['risk_difference_ci_low_pp']:.1f} to {dynamic_success['risk_difference_ci_high_pp']:.1f}), exact McNemar Holm-adjusted p={dynamic_success['p_holm_primary_family']:.3g}.

## Interpretation boundary

The time comparison for the dynamic condition is conditional on both methods succeeding and must be reported together with the separate paired success analysis. Seed-level repetitions quantify roadmap-construction variability for this fixed robot, target, and obstacle; they do not establish generalization across object classes, workspace scenes, or physical executions. Background hardware/perception processes were active on the AGX during measurement, while benchmark ROS traffic remained isolated in domains 20-119. Method order was counterbalanced to reduce monotonic warm-up/thermal bias.
"""
    (output_dir / "analysis_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
