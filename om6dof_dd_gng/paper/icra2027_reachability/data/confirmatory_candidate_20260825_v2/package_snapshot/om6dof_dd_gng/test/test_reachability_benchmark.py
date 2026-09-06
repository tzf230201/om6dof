import importlib.util
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "reachability_benchmark.py"
if not SCRIPT.exists():
    SCRIPT = PACKAGE_ROOT / "reachability_benchmark.py"
SPEC = importlib.util.spec_from_file_location("reachability_benchmark", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def valid_result(method="guarded_gng"):
    guarded = method == "guarded_gng"
    return {
        "method": method,
        "requested_node_count": 800,
        "nodes": 800,
        "anchor_node_count": 2,
        "prototype_budget": 199 if guarded else 798,
        "prototype_node_count": 199 if guarded else 798,
        "requested_guard_node_count": 599 if guarded else 0,
        "guard_node_count": 599 if guarded else 0,
        "fill_sample_node_count": 0,
        "effective_halton_start_index": 395967,
        "effective_guard_fraction": 0.75 if guarded else 0.0,
        "clear": {"valid": True, "exact_valid": True},
        "dynamic": {"valid": False, "exact_valid": False},
    }


def validate(result, method="guarded_gng"):
    MODULE.validate_runtime_result(result, method, 800, 395967, 0.75)
    return result.get("error", "")


class RuntimeContractTest(unittest.TestCase):
    def test_accepts_consistent_echo(self):
        self.assertEqual(validate(valid_result()), "")

    def test_rejects_wrong_method_and_composition(self):
        result = valid_result()
        result["method"] = "gng"
        result["fill_sample_node_count"] = 1
        error = validate(result)
        self.assertIn("method='gng', expected 'guarded_gng'", error)
        self.assertIn("node composition sums to 801", error)

    def test_rejects_missing_scenario_payload(self):
        result = valid_result()
        result["clear"] = None
        self.assertIn(
            "clear result is missing valid/exact_valid fields", validate(result)
        )

    def test_rejects_validity_disagreement(self):
        result = valid_result()
        result["dynamic"] = {"valid": True, "exact_valid": False}
        self.assertIn(
            "dynamic valid=True differs from exact_valid=False", validate(result)
        )

    def test_accepts_pure_gng_remaining_budget(self):
        result = valid_result("gng")
        self.assertEqual(validate(result, "gng"), "")


if __name__ == "__main__":
    unittest.main()
