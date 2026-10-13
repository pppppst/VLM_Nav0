import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))


def message(x=1.0, y=1.0, z=0.0, frame="map"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=7, nanosec=42)),
        point=SimpleNamespace(x=x, y=y, z=z),
    )


def test_record_flushes_numbers_invalid_points_and_never_overwrites(tmp_path):
    from record_semantic_points import PointRecorder

    with PointRecorder(tmp_path, "map") as recorder:
        recorder.record(message())
        recorder.record(message(frame="odom"))
        recorder.record(message(x=math.nan))
        recorder.record(message(z=math.inf))
        rows = [json.loads(line) for line in recorder.path.read_text().splitlines()]
        assert [r["index"] for r in rows] == [1, 2, 3, 4]
        assert [r["valid"] for r in rows] == [True, False, False, False]
        assert rows[0]["stamp"] == {"sec": 7, "nanosec": 42}
        assert rows[1]["frame_id"] == "odom"
        assert rows[2]["point"]["x"] == "nan"
        first = recorder.path
    with PointRecorder(tmp_path, "map") as other:
        assert other.path != first
        assert first.read_text().count("\n") == 4


@pytest.fixture
def inputs(tmp_path):
    image = tmp_path / "map.pgm"
    Image.new("L", (40, 30), 200).save(image)
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text(yaml.safe_dump(dict(
        image="map.pgm", resolution=0.5, origin=[0, 0, 0],
        mode="trinary", negate=0, occupied_thresh=0.65, free_thresh=0.25,
    )))
    records = tmp_path / "clicked_points_fixture.jsonl"
    points = [(1, 1), (4, 1), (4, 3), (1, 3)]
    records.write_text("".join(json.dumps(dict(
        session="fixture", index=i, frame_id="map", valid=True,
        stamp={"sec": 7, "nanosec": i}, point=dict(x=x, y=y, z=0),
    )) + "\n" for i, (x, y) in enumerate(points, 1)))
    return map_yaml, records, [dict(id="table_01", category="table", name="靠窗长桌", point_indices=[1, 2, 3, 4])]


def test_projection_inverts_y_and_origin_yaw():
    from render_semantic_map import map_to_pixel

    # local (1, 2) rotated +90 degrees, then translated (10, 20).
    assert map_to_pixel([8, 21], dict(resolution=0.5, origin=[10, 20, math.pi / 2], height=30)) == pytest.approx((1.5, 25.5))


@pytest.mark.parametrize("polygon", [
    [[1, 1], [1, 1], [4, 3], [1, 3]],
    [[1, 1], [2, 1], [3, 1], [4, 1]],
    [[1, 1], [4, 3], [4, 1], [1, 3]],
    [[-1, 1], [4, 1], [4, 3], [1, 3]],
    [[1, 1], [4, 1], [math.nan, 3], [1, 3]],
])
def test_rejects_bad_polygon_without_reordering(polygon):
    from render_semantic_map import validate_polygon

    with pytest.raises(ValueError):
        validate_polygon(polygon, dict(resolution=0.5, origin=[0, 0, 0], width=40, height=30))


def test_build_and_render_preserves_source_and_backs_up_output(inputs, tmp_path):
    from render_semantic_map import build_semantic, render

    map_yaml, records, groups = inputs
    source = (tmp_path / "map.pgm").read_bytes()
    document = build_semantic(map_yaml, records, groups)
    assert document["source_robot"] == "go2"
    assert document["frame_id"] == "map"
    assert document["furniture"][0]["polygon"] == [[1, 1], [4, 1], [4, 3], [1, 3]]
    assert document["furniture"][0]["session"] == "fixture"
    semantic = tmp_path / "semantic_map.yaml"
    semantic.write_text(yaml.safe_dump(document, allow_unicode=True))
    output = tmp_path / "annotated_map.png"
    output.write_bytes(b"previous version")
    render(semantic, output)
    assert (tmp_path / "map.pgm").read_bytes() == source
    assert len(list(tmp_path.glob("annotated_map.png.*.bak"))) == 1
    assert next(tmp_path.glob("annotated_map.png.*.bak")).read_bytes() == b"previous version"
    with Image.open(output) as image:
        assert image.size == (240, 180)
        assert image.getpixel((230, 10))[:3] == (200, 200, 200)
        assert image.getpixel((30, 155))[:3] != (200, 200, 200)


@pytest.mark.parametrize("fault", ["missing", "invalid", "duplicate_id", "duplicate_index", "wrong_frame", "nonfinite"])
def test_group_validation(inputs, fault):
    from render_semantic_map import build_semantic

    map_yaml, records, groups = inputs
    if fault == "missing":
        groups[0]["point_indices"][-1] = 99
    elif fault == "duplicate_id":
        groups.append(dict(groups[0]))
    elif fault == "duplicate_index":
        groups[0]["point_indices"][-1] = 1
    else:
        rows = [json.loads(line) for line in records.read_text().splitlines()]
        if fault == "invalid":
            rows[0]["valid"] = False
        elif fault == "wrong_frame":
            rows[0]["frame_id"] = "odom"
        else:
            rows[0]["point"]["z"] = "inf"
        records.write_text("".join(json.dumps(r) + "\n" for r in rows))
    with pytest.raises(ValueError):
        build_semantic(map_yaml, records, groups)


def test_render_rejects_changed_map_and_modified_polygon(inputs, tmp_path):
    from render_semantic_map import build_semantic, render

    map_yaml, records, groups = inputs
    document = build_semantic(map_yaml, records, groups)
    semantic = tmp_path / "semantic_map.yaml"
    document["furniture"][0]["polygon"][0][0] += 0.1
    semantic.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="point_indices"):
        render(semantic, tmp_path / "out.png")
    document = build_semantic(map_yaml, records, groups)
    semantic.write_text(yaml.safe_dump(document))
    Image.new("L", (40, 30), 0).save(tmp_path / "map.pgm")
    with pytest.raises(ValueError, match="SHA256"):
        render(semantic, tmp_path / "out.png")


def test_render_refuses_source_output(inputs, tmp_path):
    from render_semantic_map import build_semantic, render

    map_yaml, records, groups = inputs
    semantic = tmp_path / "semantic_map.yaml"
    semantic.write_text(yaml.safe_dump(build_semantic(map_yaml, records, groups)))
    before = (tmp_path / "map.pgm").read_bytes()
    with pytest.raises(ValueError):
        render(semantic, tmp_path / "map.pgm")
    assert (tmp_path / "map.pgm").read_bytes() == before


def test_overlapping_labels_move_outside_with_leader():
    from render_semantic_map import label_boxes
    from PIL import ImageDraw, ImageFont

    polygon = [(80, 80), (200, 80), (200, 160), (80, 160)]
    image = Image.new("RGB", (400, 300))
    boxes = label_boxes([polygon, list(polygon)], ["table_01", "table_02"],
                        ImageDraw.Draw(image), ImageFont.truetype("DejaVuSans.ttf", 20), image.size)
    a, b = boxes[0][0], boxes[1][0]
    assert boxes[1][2] is True
    assert a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1]
