#!/usr/bin/env python3
"""Regression tests for the version-tolerant tegrastats parser."""

from __future__ import annotations

import io
import unittest

from parse_tegrastats import build_summary, parse_stream


class ParseTegrastatsTest(unittest.TestCase):
    def test_agx_orin_power_names_and_missing_emc_remain_explicit(self) -> None:
        source = (
            "09-10-2026 16:34:28 RAM 9177/62828MB (lfb 1571x4MB) "
            "SWAP 1774/31414MB (cached 7MB) "
            "CPU [64%@2201,off,78%@2201] GR3D_FREQ 0% "
            "cpu@60.875C soc0@53.781C "
            "VDD_GPU_SOC 3292mW/3292mW VDD_CPU_CV 11760mW/11760mW "
            "VIN_SYS_5V0 5818mW/5818mW\n"
        )
        rows, metadata = parse_stream(io.StringIO(source))
        summary = build_summary(rows, metadata, "fixture")

        self.assertEqual(metadata["parsed_sample_count"], 1)
        self.assertIn("VDD_GPU_SOC", summary["observed_power_rails"])
        self.assertIn("VIN_SYS_5V0", summary["observed_power_rails"])
        self.assertFalse(summary["field_presence"]["emc_load"])
        self.assertFalse(summary["field_presence"]["requested_power_rails"]["VDD_IN"])
        cpu1 = [
            row for row in rows
            if row.metric == "cpu_state" and row.component == "cpu1"
        ]
        self.assertEqual(cpu1[0].status, "off")
        self.assertIsNone(cpu1[0].value)

    def test_scalar_emc_gr3d_and_requested_power_rails(self) -> None:
        source = (
            "RAM 100/1000MB SWAP 0/0MB CPU [10%@729] "
            "EMC_FREQ 42%@2133 GR3D_FREQ 7%@306 "
            "VDD_IN 5000mW/4900mW VDD_CPU_GPU_CV 2100mW/2000mW "
            "VDD_SOC 900mW/850mW\n"
        )
        rows, metadata = parse_stream(io.StringIO(source))
        summary = build_summary(rows, metadata, "fixture")

        self.assertTrue(summary["field_presence"]["emc_frequency"])
        self.assertTrue(summary["field_presence"]["gr3d_frequency"])
        self.assertTrue(all(summary["field_presence"]["requested_power_rails"].values()))
        values = {(row.metric, row.component): row.value for row in rows}
        self.assertEqual(values[("emc_load", "memory_controller")], 42.0)
        self.assertEqual(values[("gr3d_frequency", "gpu")], 306.0)

    def test_per_gpc_frequency_and_unparsed_line_accounting(self) -> None:
        source = "GR3D_FREQ 12%@[306,612]\nnot a tegrastats line\n\n"
        rows, metadata = parse_stream(io.StringIO(source))

        self.assertEqual(metadata["parsed_sample_count"], 1)
        self.assertEqual(metadata["unparsed_nonempty_line_numbers"], [2])
        frequencies = {
            row.component: row.value for row in rows if row.metric == "gr3d_frequency"
        }
        self.assertEqual(frequencies, {"gpu0": 306.0, "gpu1": 612.0})


if __name__ == "__main__":
    unittest.main()
