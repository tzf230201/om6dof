"""Key handling, prompt plumbing and reply parsing — no network calls."""

import json
import numpy as np
import pytest

from om6dof_pick_and_place_gemini import gemini_client as gc


def test_no_key_anywhere_means_disabled_not_broken():
    client = gc.GeminiClient(api_key="", key_env="OM6DOF_NO_SUCH_VAR",
                             key_file="/nonexistent/key")
    assert client.enabled is False
    assert "DISABLED" in client.describe()


def test_calling_without_a_key_raises_the_degradable_error():
    client = gc.GeminiClient(api_key="", key_env="OM6DOF_NO_SUCH_VAR",
                             key_file="/nonexistent/key")
    with pytest.raises(gc.GeminiUnavailable):
        client._post([{"text": "hi"}])


def test_the_key_never_appears_in_the_status_line():
    client = gc.GeminiClient(api_key="AIza-secret-value",
                             key_env="OM6DOF_NO_SUCH_VAR",
                             key_file="/nonexistent/key")
    assert client.enabled
    assert "secret" not in client.describe()


def test_key_resolution_order(tmp_path, monkeypatch):
    key_file = tmp_path / "key"
    key_file.write_text("from-file\n")
    monkeypatch.setenv("OM6DOF_TEST_KEY", "from-env")
    assert gc.resolve_api_key("explicit", "OM6DOF_TEST_KEY", str(key_file)) \
        == "explicit"
    assert gc.resolve_api_key("", "OM6DOF_TEST_KEY", str(key_file)) == "from-env"
    monkeypatch.delenv("OM6DOF_TEST_KEY")
    assert gc.resolve_api_key("", "OM6DOF_TEST_KEY", str(key_file)) == "from-file"


def test_a_placeholder_key_counts_as_no_key():
    assert gc.resolve_api_key("<paste your key here>", "OM6DOF_NO_SUCH_VAR",
                              "/nonexistent") == ""


def test_parse_json_payload_strips_a_code_fence():
    assert gc.parse_json_payload('```json\n{"label": "cup"}\n```') \
        == {"label": "cup"}


def test_parse_json_payload_digs_json_out_of_prose():
    assert gc.parse_json_payload('Sure! {"label": "box"} hope that helps') \
        == {"label": "box"}


def test_parse_json_payload_takes_the_first_of_a_list():
    assert gc.parse_json_payload('[{"label": "a"}, {"label": "b"}]') \
        == {"label": "a"}


def test_parse_json_payload_rejects_unusable_text():
    with pytest.raises(gc.GeminiError):
        gc.parse_json_payload("I could not see the object.")


def test_parse_json_payload_skips_a_json_template_in_the_reasoning():
    # Gemma narrates its reasoning first, including a template of the shape
    # it is about to answer in, even with responseMimeType: application/json
    # set. A greedy first-brace-to-last-brace regex would splice the
    # template's opening brace to the real answer's closing brace.
    text = (
        'The object is a small bottle of coffee.\n'
        'JSON format: `{"label": "<category>", "confidence": <float>}`\n'
        '{"label": "bottle", "confidence": 1.0, "reason": "coffee bottle"}')
    assert gc.parse_json_payload(text) == {
        "label": "bottle", "confidence": 1.0, "reason": "coffee bottle"}


def test_parse_json_payload_handles_nested_braces_in_reasoning_and_answer():
    text = ('Thinking: {"note": "a red thing", "size": {"w": 1, "h": 2}} '
            'anyway.\nFinal: {"label": "cup", "confidence": 0.8}')
    assert gc.parse_json_payload(text) == {"label": "cup", "confidence": 0.8}


def test_balanced_json_objects_ignores_unmatched_brackets():
    assert gc._balanced_json_objects('some } stray { brackets') == []
    assert gc._balanced_json_objects('{"a": 1} then [2, 3]') == \
        ['{"a": 1}', '[2, 3]']


def test_localization_converts_the_0_1000_grid_to_pixels():
    found = gc.localization_from_payload(
        {"found": True, "point": [500, 250]}, 640, 480)
    assert found.found
    assert found.pixel == pytest.approx((160.0, 240.0))


def test_localization_falls_back_to_the_box_centre():
    found = gc.localization_from_payload(
        {"found": True, "box_2d": [400, 200, 600, 300]}, 1000, 1000)
    assert found.pixel == pytest.approx((250.0, 500.0))


def test_localization_reports_a_miss_without_raising():
    assert gc.localization_from_payload({"found": False}, 640, 480).found is False


def test_localization_clamps_a_point_outside_the_image():
    found = gc.localization_from_payload(
        {"found": True, "point": [1200, -50]}, 640, 480)
    assert 0 <= found.pixel[0] <= 639 and 0 <= found.pixel[1] <= 479


def test_extract_text_explains_a_blocked_prompt():
    with pytest.raises(gc.GeminiError, match="blockReason=SAFETY"):
        gc.GeminiClient._extract_text(
            {"promptFeedback": {"blockReason": "SAFETY"}})


def test_classify_snaps_an_off_list_answer_to_the_fallback(monkeypatch):
    client = gc.GeminiClient(api_key="k", key_env="OM6DOF_NO_SUCH_VAR",
                             key_file="/nonexistent")
    monkeypatch.setattr(client, "_post",
                        lambda parts: json.dumps({"label": "banana",
                                                  "confidence": 0.4}))
    image = np.zeros((32, 32, 3), np.uint8)
    assert client.classify(image, ["cup", "box", "unknown"]).label == "unknown"


def test_classify_is_case_insensitive_about_the_label(monkeypatch):
    client = gc.GeminiClient(api_key="k", key_env="OM6DOF_NO_SUCH_VAR",
                             key_file="/nonexistent")
    monkeypatch.setattr(client, "_post",
                        lambda parts: '{"label": "CUP", "confidence": 0.9}')
    result = client.classify(np.zeros((32, 32, 3), np.uint8),
                             ["cup", "box", "unknown"])
    assert result.label == "cup" and result.confidence == pytest.approx(0.9)


def test_the_request_carries_the_prompt_and_one_jpeg(monkeypatch):
    client = gc.GeminiClient(api_key="k", key_env="OM6DOF_NO_SUCH_VAR",
                             key_file="/nonexistent")
    seen = {}

    def capture(parts):
        seen["parts"] = parts
        return '{"found": true, "point": [500, 500]}'

    monkeypatch.setattr(client, "_post", capture)
    client.locate(np.zeros((48, 64, 3), np.uint8), "the red cube")
    assert "the red cube" in seen["parts"][0]["text"]
    assert seen["parts"][1]["inline_data"]["mime_type"] == "image/jpeg"


def test_crop_around_a_pixel_stays_inside_the_image():
    image = np.zeros((480, 640, 3), np.uint8)
    crop = gc.crop_around(image, (5.0, 5.0), 90)
    assert crop.shape[0] <= 480 and crop.shape[1] <= 640 and crop.size > 0


def test_crop_around_prefers_a_known_bounding_box():
    image = np.zeros((480, 640, 3), np.uint8)
    crop = gc.crop_around(image, (320.0, 240.0), 90, bbox=(100, 100, 140, 160))
    assert crop.shape[1] == pytest.approx(41 + 32, abs=1)
