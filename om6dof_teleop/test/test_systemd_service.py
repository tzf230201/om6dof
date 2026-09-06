from pathlib import Path


SERVICE_FILE = (
    Path(__file__).resolve().parents[1]
    / "systemd"
    / "om6dof-hardware.service"
)


def _service_section():
    text = SERVICE_FILE.read_text(encoding="utf-8")
    return text.split("[Service]", 1)[1].split("[Install]", 1)[0]


def _service_directives():
    return [
        line.strip()
        for line in _service_section().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


def test_controller_manager_can_raise_only_its_update_thread_to_fifo():
    directives = _service_directives()

    assert "LimitRTPRIO=60" in directives
    assert "LimitMEMLOCK=infinity" in directives
    assert not any(
        line.startswith("CPUSchedulingPolicy=") for line in directives
    )


def test_packaged_service_pins_fastdds():
    assert (
        "Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
        in _service_directives()
    )


def test_packaged_service_leaves_cpu_affinity_host_specific():
    assert not any(
        line.startswith("CPUAffinity=") for line in _service_directives()
    )
