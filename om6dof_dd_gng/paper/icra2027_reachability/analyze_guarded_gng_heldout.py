#!/usr/bin/env python3
"""Reproducible paired analysis for the three-method held-out benchmark."""

import argparse
import json
import math
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats
from statsmodels.stats.contingency_tables import cochrans_q
from statsmodels.stats.proportion import proportion_confint


METHODS = ("gng", "guarded_gng", "halton_prm")
PAIRS = (
    ("guarded_gng", "gng"),
    ("guarded_gng", "halton_prm"),
    ("gng", "halton_prm"),
)
FAILURE_SEEDS = (6, 9, 16, 18, 35, 37, 44, 47)


def parse_bool(value):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    raise ValueError(f"expected explicit boolean, got {value!r}")


def describe(values):
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "sd": float(np.std(values, ddof=1)) if len(values) > 1 else math.nan,
        "median": float(np.median(values)),
        "q1": float(np.quantile(values, 0.25)),
        "q3": float(np.quantile(values, 0.75)),
    }


def percentile_ci(samples, confidence=0.95):
    alpha = (1.0 - confidence) / 2.0
    low, high = np.quantile(samples, [alpha, 1.0 - alpha])
    return float(low), float(high)


def bootstrap_ci(values, statistic, rng, resamples=20000):
    values = np.asarray(values, dtype=float)
    indices = rng.integers(0, len(values), size=(resamples, len(values)))
    samples = values[indices]
    if statistic == "mean":
        estimates = np.mean(samples, axis=1)
    elif statistic == "median":
        estimates = np.median(samples, axis=1)
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


def holm_adjust(p_values):
    p_values = np.asarray(p_values, dtype=float)
    order = np.argsort(p_values)
    adjusted = np.empty_like(p_values)
    running = 0.0
    count = len(p_values)
    for rank, index in enumerate(order):
        running = max(running, (count - rank) * p_values[index])
        adjusted[index] = min(1.0, running)
    return adjusted


def validate(df):
    expected_seeds = set(range(50, 100))
    method_counts = df["method"].value_counts().to_dict()
    seed_methods = df.groupby("seed")["method"].apply(set)
    targets = df["target_xyz"].apply(json.loads)
    target_joints = df["target_joints"].apply(json.loads)
    obstacles = df["obstacle_xyz"].apply(json.loads)
    target_ref = np.asarray(targets.iloc[0], dtype=float)
    joints_ref = np.asarray(target_joints.iloc[0], dtype=float)
    obstacle_ref = np.asarray(obstacles.iloc[0], dtype=float)

    positions = {method: [0, 0, 0] for method in METHODS}
    for _, group in df.sort_values("run_index").groupby("seed", sort=False):
        ordered = group.sort_values("run_index")["method"].tolist()
        for position, method in enumerate(ordered):
            positions[method][position] += 1

    checks = {
        "row_count": int(len(df)),
        "method_counts": {key: int(value) for key, value in method_counts.items()},
        "seed_count": int(df["seed"].nunique()),
        "seed_range": [int(df["seed"].min()), int(df["seed"].max())],
        "all_expected_seeds_present": set(df["seed"]) == expected_seeds,
        "each_seed_has_all_methods": bool(
            seed_methods.apply(lambda value: value == set(METHODS)).all()
        ),
        "run_indices_complete": set(df["run_index"]) == set(range(150)),
        "method_position_counts": positions,
        "errors": int(df["error"].fillna("").astype(str).str.len().gt(0).sum()),
        "nodes_all_800": bool((df["nodes"] == 800).all()),
        "matched_target_xyz": bool(
            all(np.allclose(np.asarray(value, dtype=float), target_ref) for value in targets)
        ),
        "matched_target_joints": bool(
            all(np.allclose(np.asarray(value, dtype=float), joints_ref) for value in target_joints)
        ),
        "matched_obstacle_xyz": bool(
            all(np.allclose(np.asarray(value, dtype=float), obstacle_ref) for value in obstacles)
        ),
        "clear_valid_matches_exact": bool(
            (df["clear_valid"] == df["clear_exact_valid"]).all()
        ),
        "dynamic_valid_matches_exact": bool(
            (df["dynamic_valid"] == df["dynamic_exact_valid"]).all()
        ),
        "target_xyz": target_ref.tolist(),
        "target_joints": joints_ref.tolist(),
        "obstacle_xyz": obstacle_ref.tolist(),
    }
    checks["valid"] = all(
        [
            checks["row_count"] == 150,
            checks["method_counts"] == {method: 50 for method in METHODS},
            checks["seed_count"] == 50,
            checks["all_expected_seeds_present"],
            checks["each_seed_has_all_methods"],
            checks["run_indices_complete"],
            checks["errors"] == 0,
            checks["nodes_all_800"],
            checks["matched_target_xyz"],
            checks["matched_target_joints"],
            checks["matched_obstacle_xyz"],
            checks["clear_valid_matches_exact"],
            checks["dynamic_valid_matches_exact"],
        ]
    )
    if not checks["valid"]:
        raise RuntimeError(f"held-out validation failed: {checks}")
    return checks


def method_summary(df):
    rows = []
    for method in METHODS:
        group = df[df["method"] == method].sort_values("seed")
        successes = int(group["dynamic_exact_valid"].sum())
        ci_low, ci_high = proportion_confint(successes, len(group), method="wilson")
        row = {
            "method": method,
            "n": len(group),
            "clear_successes": int(group["clear_valid"].sum()),
            "dynamic_successes": successes,
            "dynamic_success_rate": successes / len(group),
            "dynamic_success_wilson_ci_low": float(ci_low),
            "dynamic_success_wilson_ci_high": float(ci_high),
        }
        for metric in (
            "components",
            "edges",
            "build_time_ms",
            "clear_planning_time_ms",
            "dynamic_planning_time_ms",
            "dynamic_exact_time_ms",
            "dynamic_exact_checks",
            "dynamic_exact_replans",
            "dynamic_path_nodes",
        ):
            values = group[metric]
            if metric.startswith("dynamic_"):
                values = values[group["dynamic_exact_valid"]]
            summary = describe(values)
            for statistic, value in summary.items():
                row[f"{metric}_{statistic}"] = value
        rows.append(row)
    return pd.DataFrame(rows)


def success_analysis(df):
    pivot = df.pivot(index="seed", columns="method", values="dynamic_exact_valid").sort_index()
    global_result = cochrans_q(pivot[list(METHODS)].astype(int).to_numpy())
    rows = []
    for pair_index, (method_a, method_b) in enumerate(PAIRS):
        a = pivot[method_a].astype(bool).to_numpy()
        b = pivot[method_b].astype(bool).to_numpy()
        a_only = int(np.sum(a & ~b))
        b_only = int(np.sum(~a & b))
        discordant = a_only + b_only
        p_value = (
            float(stats.binomtest(a_only, discordant, 0.5).pvalue) if discordant else 1.0
        )
        differences = (a.astype(float) - b.astype(float)) * 100.0
        rng = np.random.default_rng(2026082300 + pair_index)
        ci_low, ci_high = bootstrap_ci(differences, "mean", rng)
        rows.append(
            {
                "method_a": method_a,
                "method_b": method_b,
                "n_pairs": len(a),
                "a_successes": int(a.sum()),
                "b_successes": int(b.sum()),
                "both_success": int(np.sum(a & b)),
                "a_only_success": a_only,
                "b_only_success": b_only,
                "both_fail": int(np.sum(~a & ~b)),
                "paired_risk_difference_pp_a_minus_b": float(np.mean(differences)),
                "risk_difference_ci_low_pp": ci_low,
                "risk_difference_ci_high_pp": ci_high,
                "mcnemar_exact_p_raw": p_value,
                "confirmatory_primary": method_a == "guarded_gng",
            }
        )
    result = pd.DataFrame(rows)
    primary_mask = result["confirmatory_primary"].to_numpy(dtype=bool)
    result["mcnemar_p_holm_primary"] = np.nan
    result.loc[primary_mask, "mcnemar_p_holm_primary"] = holm_adjust(
        result.loc[primary_mask, "mcnemar_exact_p_raw"].to_numpy()
    )
    global_row = {
        "test": "Cochran Q: dynamic success across three methods",
        "n_complete_blocks": len(pivot),
        "statistic": float(global_result.statistic),
        "degrees_of_freedom": 2,
        "p_raw": float(global_result.pvalue),
    }
    return result, global_row


def pairwise_continuous(df):
    specs = (
        ("components", "lower", None),
        ("edges", "higher", None),
        ("build_time_ms", "lower", None),
        ("clear_planning_time_ms", "lower", "clear_valid"),
        ("dynamic_planning_time_ms", "lower", "dynamic_exact_valid"),
        ("dynamic_exact_time_ms", "lower", "dynamic_exact_valid"),
        ("dynamic_exact_checks", "lower", "dynamic_exact_valid"),
        ("dynamic_exact_replans", "lower", "dynamic_exact_valid"),
        ("dynamic_path_nodes", "lower", "dynamic_exact_valid"),
    )
    pivots = {
        column: df.pivot(index="seed", columns="method", values=column).sort_index()
        for column in {item for spec in specs for item in (spec[0], spec[2]) if item}
    }
    pairwise_rows = []
    global_rows = []
    for metric_index, (metric, direction, success_column) in enumerate(specs):
        values = pivots[metric]
        eligible = values.notna().all(axis=1)
        if success_column:
            success = pivots[success_column].astype(bool)
            eligible &= success[list(METHODS)].all(axis=1)
        complete = values.loc[eligible, list(METHODS)].astype(float)
        friedman = stats.friedmanchisquare(*(complete[method] for method in METHODS))
        global_rows.append(
            {
                "test": f"Friedman: {metric}",
                "n_complete_blocks": len(complete),
                "statistic": float(friedman.statistic),
                "degrees_of_freedom": 2,
                "p_raw": float(friedman.pvalue),
            }
        )
        metric_row_indices = []
        metric_p_values = []
        for pair_index, (method_a, method_b) in enumerate(PAIRS):
            pair_eligible = values[[method_a, method_b]].notna().all(axis=1)
            if success_column:
                success = pivots[success_column].astype(bool)
                pair_eligible &= success[method_a] & success[method_b]
            pair = values.loc[pair_eligible, [method_a, method_b]].astype(float)
            raw_difference = pair[method_a].to_numpy() - pair[method_b].to_numpy()
            oriented = -raw_difference if direction == "lower" else raw_difference
            rng = np.random.default_rng(2026082400 + metric_index * 17 + pair_index)
            mean_ci = bootstrap_ci(raw_difference, "mean", rng)
            median_ci = bootstrap_ci(raw_difference, "median", rng)
            if np.all(raw_difference == 0.0):
                wilcoxon_statistic, wilcoxon_p = 0.0, 1.0
                sign_p = 1.0
            else:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    wilcoxon = stats.wilcoxon(
                        raw_difference, zero_method="pratt", alternative="two-sided"
                    )
                wilcoxon_statistic = float(wilcoxon.statistic)
                wilcoxon_p = float(wilcoxon.pvalue)
                nonzero = raw_difference[raw_difference != 0.0]
                sign_p = float(
                    stats.binomtest(int(np.sum(nonzero > 0.0)), len(nonzero), 0.5).pvalue
                )
            a_summary = describe(pair[method_a])
            b_summary = describe(pair[method_b])
            row = {
                "metric": metric,
                "direction": direction,
                "method_a": method_a,
                "method_b": method_b,
                "analysis_population": (
                    f"both {success_column}=true" if success_column else "all paired seeds"
                ),
                "n_pairs": len(pair),
                "a_mean": a_summary["mean"],
                "a_sd": a_summary["sd"],
                "a_median": a_summary["median"],
                "b_mean": b_summary["mean"],
                "b_sd": b_summary["sd"],
                "b_median": b_summary["median"],
                "mean_difference_a_minus_b": float(np.mean(raw_difference)),
                "mean_difference_ci_low": mean_ci[0],
                "mean_difference_ci_high": mean_ci[1],
                "median_difference_a_minus_b": float(np.median(raw_difference)),
                "median_difference_ci_low": median_ci[0],
                "median_difference_ci_high": median_ci[1],
                "rank_biserial_positive_favors_a": rank_biserial(oriented),
                "wilcoxon_statistic": wilcoxon_statistic,
                "wilcoxon_p_raw": wilcoxon_p,
                "sign_test_p_sensitivity": sign_p,
            }
            pairwise_rows.append(row)
            metric_row_indices.append(len(pairwise_rows) - 1)
            metric_p_values.append(wilcoxon_p)
        adjusted = holm_adjust(metric_p_values)
        for row_index, value in zip(metric_row_indices, adjusted):
            pairwise_rows[row_index]["wilcoxon_p_holm_within_metric"] = float(value)
    return pd.DataFrame(pairwise_rows), pd.DataFrame(global_rows)


def development_ablation(base_csv, data_dir):
    base = pd.read_csv(base_csv)
    for column in ("dynamic_valid", "dynamic_exact_valid"):
        base[column] = base[column].map(parse_bool)
    if not (base["dynamic_valid"] == base["dynamic_exact_valid"]).all():
        raise RuntimeError("development base valid/exact-valid mismatch")
    rows = [
        {
            "guard_fraction": 0.0,
            "method": "gng",
            "n_development_failure_seeds": len(FAILURE_SEEDS),
            "dynamic_successes": int(
                base[(base["method"] == "gng") & base["seed"].isin(FAILURE_SEEDS)][
                    "dynamic_exact_valid"
                ].sum()
            ),
        }
    ]
    for fraction, suffix in ((0.10, "010"), (0.25, "025"), (0.50, "050"), (0.75, "075")):
        path = data_dir / f"guarded_gng_dev_fraction_{suffix}_20260823.csv"
        frame = pd.read_csv(path)
        for column in ("dynamic_valid", "dynamic_exact_valid"):
            frame[column] = frame[column].map(parse_bool)
        if not (frame["dynamic_valid"] == frame["dynamic_exact_valid"]).all():
            raise RuntimeError(f"development valid/exact-valid mismatch: {path}")
        rows.append(
            {
                "guard_fraction": fraction,
                "method": "guarded_gng",
                "n_development_failure_seeds": len(frame),
                "dynamic_successes": int(frame["dynamic_exact_valid"].sum()),
            }
        )
    result = pd.DataFrame(rows)
    result["dynamic_success_rate"] = (
        result["dynamic_successes"] / result["n_development_failure_seeds"]
    )
    result["role"] = "development-only hyperparameter selection; not confirmatory"
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("heldout_csv")
    parser.add_argument("development_base_csv")
    parser.add_argument("output_dir")
    args = parser.parse_args()
    heldout_path = Path(args.heldout_csv).expanduser().resolve()
    development_path = Path(args.development_base_csv).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(heldout_path)
    for column in ("clear_valid", "clear_exact_valid", "dynamic_valid", "dynamic_exact_valid"):
        df[column] = df[column].map(parse_bool)
    checks = validate(df)
    summary = method_summary(df)
    success, success_global = success_analysis(df)
    continuous, continuous_global = pairwise_continuous(df)
    global_tests = pd.concat(
        [pd.DataFrame([success_global]), continuous_global], ignore_index=True
    )
    development = development_ablation(development_path, heldout_path.parent)
    failures = df.loc[
        ~df["dynamic_exact_valid"],
        [
            "seed",
            "method",
            "dynamic_reason",
            "dynamic_blocked_nodes",
            "dynamic_blocked_edges",
            "dynamic_exact_checks",
            "dynamic_exact_replans",
        ],
    ].sort_values(["method", "seed"])
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
    ].sort_values(["method", "seed"])

    (output_dir / "validation.json").write_text(
        json.dumps(checks, indent=2), encoding="utf-8"
    )
    summary.to_csv(output_dir / "method_summary.csv", index=False)
    success.to_csv(output_dir / "pairwise_success.csv", index=False)
    continuous.to_csv(output_dir / "pairwise_continuous.csv", index=False)
    global_tests.to_csv(output_dir / "global_tests.csv", index=False)
    development.to_csv(output_dir / "development_ablation.csv", index=False)
    failures.to_csv(output_dir / "failures.csv", index=False)
    replans.to_csv(output_dir / "exact_replans.csv", index=False)

    by_method = summary.set_index("method")
    guarded_vs_gng = success.set_index(["method_a", "method_b"]).loc[
        ("guarded_gng", "gng")
    ]
    guarded_vs_halton = success.set_index(["method_a", "method_b"]).loc[
        ("guarded_gng", "halton_prm")
    ]
    report = f"""# Guarded-GNG held-out 50-offset analysis

## Validation

- 150 roadmap builds: 50 paired held-out sample-stream offsets (50--99) for each of pure GNG, guarded-GNG, and Halton/PRM.
- Equal 800-node output budget and identical anchored start, target, midpoint point obstacle, and offset-specific stream position. Candidate-information and compute budgets are not equal.
- Every permutation of the three method orders was cycled; position counts: {checks['method_position_counts']}.
- Process errors: {checks['errors']}; clear-scene successes: 150/150.

## Development-only selection

- On the eight original pure-GNG failure offsets, success increased from 0/8 at 0% guard to 1/8, 4/8, 7/8, and 8/8 at 10%, 25%, 50%, and 75% guard.
- The 75% value was fixed before the held-out run. These eight offsets are regression/development evidence only.

## Held-out results

- Dynamic exact-valid success: pure GNG {int(by_method.loc['gng', 'dynamic_successes'])}/50 ({by_method.loc['gng', 'dynamic_success_rate']:.0%}), guarded-GNG {int(by_method.loc['guarded_gng', 'dynamic_successes'])}/50 ({by_method.loc['guarded_gng', 'dynamic_success_rate']:.0%}), Halton/PRM {int(by_method.loc['halton_prm', 'dynamic_successes'])}/50 ({by_method.loc['halton_prm', 'dynamic_success_rate']:.0%}). Cochran Q p={success_global['p_raw']:.3g}.
- Guarded-GNG versus pure GNG: paired risk difference {guarded_vs_gng['paired_risk_difference_pp_a_minus_b']:.1f} pp (95% paired-bootstrap CI {guarded_vs_gng['risk_difference_ci_low_pp']:.1f} to {guarded_vs_gng['risk_difference_ci_high_pp']:.1f}); exact McNemar Holm p={guarded_vs_gng['mcnemar_p_holm_primary']:.3g}.
- Guarded-GNG versus Halton/PRM: paired risk difference {guarded_vs_halton['paired_risk_difference_pp_a_minus_b']:.1f} pp (95% CI {guarded_vs_halton['risk_difference_ci_low_pp']:.1f} to {guarded_vs_halton['risk_difference_ci_high_pp']:.1f}); exact McNemar Holm p={guarded_vs_halton['mcnemar_p_holm_primary']:.3g}.
- Mean connected components: pure GNG {by_method.loc['gng', 'components_mean']:.2f}, guarded-GNG {by_method.loc['guarded_gng', 'components_mean']:.2f}, Halton/PRM {by_method.loc['halton_prm', 'components_mean']:.2f}.
- Mean validated edges: pure GNG {by_method.loc['gng', 'edges_mean']:.1f}, guarded-GNG {by_method.loc['guarded_gng', 'edges_mean']:.1f}, Halton/PRM {by_method.loc['halton_prm', 'edges_mean']:.1f}.
- Mean build time: pure GNG {by_method.loc['gng', 'build_time_ms_mean']:.1f} ms, guarded-GNG {by_method.loc['guarded_gng', 'build_time_ms_mean']:.1f} ms, Halton/PRM {by_method.loc['halton_prm', 'build_time_ms_mean']:.1f} ms.
- Dynamic timing is conditional on success and therefore secondary: means were {by_method.loc['gng', 'dynamic_planning_time_ms_mean']:.1f}, {by_method.loc['guarded_gng', 'dynamic_planning_time_ms_mean']:.1f}, and {by_method.loc['halton_prm', 'dynamic_planning_time_ms_mean']:.1f} ms, respectively.

## Interpretation

Guard sampling reduced the fixed-query held-out pure-GNG failure count from four to one, but the paired improvement was not statistically significant at n=50 and guarded-GNG also lost pure GNG's low-component-count characteristic. Guarded-GNG matched Halton/PRM's 49/50 point success rate, with different failed offsets, but did not establish superiority. The result motivates a target-robustness--connectivity hypothesis and query-time multi-anchor or topology-repair methods. It does not justify claiming general reachable-area coverage or that guard sampling solves obstacle-robust reachability.
"""
    (output_dir / "analysis_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
