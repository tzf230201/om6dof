# Copyright 2026 KUBOTA Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Offline contract for actionable SyncRead receive-failure telemetry."""

from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "dynamixel"
    / "dynamixel.cpp"
)


def test_receive_failures_report_elapsed_time_and_remaining_queue_depth():
    source = SOURCE.read_text(encoding="utf-8")

    assert "const auto rx_start = std::chrono::steady_clock::now();" in source
    assert "std::chrono::duration<double, std::milli>" in source
    assert "port_handler->getBytesAvailable()" in source
    assert "[Rx elapsed : %.3f ms]" in source
    assert "[Queued bytes after failure : %d]" in source


def test_instrumentation_does_not_replace_existing_log_throttle():
    source = SOURCE.read_text(encoding="utf-8")

    assert "now - last_log_time < std::chrono::seconds(5)" in source
    assert "the number of bytes\n        // already consumed" in source
