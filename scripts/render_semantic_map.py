#!/usr/bin/env python3
"""Validate click provenance and render furniture over the immutable occupancy image."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
import uuid

from PIL import Image, ImageDraw, ImageFont
import yaml


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def load_map(map_yaml):
    path = Path(map_yaml).resolve()
    data = yaml.safe_load(path.read_text())
    image = (path.parent / data["image"]).resolve()
    if not finite(data["resolution"]) or data["resolution"] <= 0:
        raise ValueError("地图 resolution 必须为正有限数")
    if len(data["origin"]) != 3 or not all(finite(v) for v in data["origin"]):
        raise ValueError("地图 origin 必须为三个有限数")
    with Image.open(image) as source:
        width, height = source.size
    return dict(yaml=str(path), image=str(image), yaml_sha256=sha256(path),
                image_sha256=sha256(image), resolution=data["resolution"], origin=data["origin"],
                width=width, height=height, mode=data.get("mode", "trinary"),
                negate=data["negate"], occupied_thresh=data["occupied_thresh"], free_thresh=data["free_thresh"])


def map_to_pixel(point, source):
    x, y = point[0] - source["origin"][0], point[1] - source["origin"][1]
    c, s = math.cos(source["origin"][2]), math.sin(source["origin"][2])
    # Origin is the lower-left cell edge; Pillow integer coordinates are pixel centers.
    return ((c * x + s * y) / source["resolution"] - 0.5,
            source["height"] - 0.5 - (-s * x + c * y) / source["resolution"])


def cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a, b, c, d):
    ab_c, ab_d, cd_a, cd_b = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
    if ab_c * ab_d < 0 and cd_a * cd_b < 0:
        return True
    return any(abs(value) <= 1e-10 and
               min(start[0], end[0]) - 1e-10 <= p[0] <= max(start[0], end[0]) + 1e-10 and
               min(start[1], end[1]) - 1e-10 <= p[1] <= max(start[1], end[1]) + 1e-10
               for value, start, end, p in ((ab_c, a, b, c), (ab_d, a, b, d), (cd_a, c, d, a), (cd_b, c, d, b)))


def validate_polygon(polygon, source):
    if len(polygon) != 4 or any(len(p) != 2 or not all(finite(v) for v in p) for p in polygon):
        raise ValueError("必须提供四个有限 x/y 角点")
    if len({tuple(p) for p in polygon}) != 4:
        raise ValueError("重复角点")
    a, b, c, d = polygon
    if segments_intersect(a, b, c, d) or segments_intersect(b, c, d, a):
        raise ValueError("角点顺序造成自交，不自动重排")
    area2 = sum(cross([0, 0], polygon[i], polygon[(i + 1) % 4]) for i in range(4))
    if abs(area2) <= 1e-10 or any(abs(cross(polygon[i-1], polygon[i], polygon[(i+1) % 4])) <= 1e-10 for i in range(4)):
        raise ValueError("零面积或退化角点")
    outside = [i + 1 for i, p in enumerate(polygon) if not (
        -0.5 - 1e-8 <= map_to_pixel(p, source)[0] <= source["width"] - 0.5 + 1e-8 and
        -0.5 - 1e-8 <= map_to_pixel(p, source)[1] <= source["height"] - 0.5 + 1e-8)]
    if outside:
        raise ValueError(f"第 {outside} 个角点在源地图边界外，请核对原始点号")


def build_semantic(map_yaml, record_path, groups):
    """Use explicit human groups (id, category, point_indices); preserve supplied order."""
    source = load_map(map_yaml)
    record_path = Path(record_path).resolve()
    rows = [json.loads(line) for line in record_path.read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError("没有选点记录")
    session = rows[0]["session"]
    if any(row["session"] != session or type(row["index"]) is not int or row["index"] != i
           for i, row in enumerate(rows, 1)):
        raise ValueError("原始记录 session 不一致或编号不连续")
    by_index = {row["index"]: row for row in rows}
    furniture, ids = [], set()
    for group in groups:
        identifier, category, indices = group["id"], group["category"], group["point_indices"]
        if not isinstance(identifier, str) or not identifier.strip() or identifier in ids:
            raise ValueError(f"家具 ID 为空或重复: {identifier}")
        if not isinstance(category, str) or not category.strip():
            raise ValueError(f"{identifier}: 必须指定类别")
        if len(indices) != 4 or any(type(i) is not int for i in indices) or len(set(indices)) != 4:
            raise ValueError(f"{identifier}: 必须提供四个不同原始点号")
        polygon = []
        for index in indices:
            row = by_index.get(index)
            if row is None or row["valid"] is not True or row["frame_id"] != "map" or not all(
                    finite(row["point"][axis]) for axis in ("x", "y", "z")):
                raise ValueError(f"{identifier}: 点号 {index} 不存在或无效")
            polygon.append([row["point"]["x"], row["point"]["y"]])
        try:
            validate_polygon(polygon, source)
        except ValueError as error:
            raise ValueError(f"{identifier} 点号 {indices}: {error}") from error
        item = dict(id=identifier, category=category, polygon=polygon, session=session, point_indices=indices)
        for key in ("name", "room_id"):
            if key in group:
                item[key] = group[key]
        furniture.append(item)
        ids.add(identifier)
    return dict(schema_version=1, source_robot="go2", frame_id="map", units="meters", source_map=source,
                point_records=[dict(session=session, file=str(record_path), sha256=sha256(record_path))], furniture=furniture)


def backup_existing(path):
    path = Path(path)
    if path.exists():
        backup = path.with_name(path.name + "." + uuid.uuid4().hex + ".bak")
        with path.open("rb") as original, backup.open("xb") as saved:
            shutil.copyfileobj(original, saved)
        print(f"保留旧版本: {backup}")


def point_inside(point, polygon):
    x, y = point
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (a[1] > y) != (b[1] > y) and x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
            inside = not inside
    return inside


def label_boxes(polygons, labels, draw, font, size):
    """Prefer an interior label; fall back to free map space with a leader line."""
    occupied, result = [], []
    for polygon, label in zip(polygons, labels):
        left, top, right, bottom = draw.textbbox((0, 0), label, font=font)
        w, h = right - left + 12, bottom - top + 10
        cx, cy = sum(p[0] for p in polygon) / 4, sum(p[1] for p in polygon) / 4
        positions = [(cx - w / 2, cy - h / 2)]
        for distance in range(32, max(size) + 32, 32):
            positions.extend((cx + dx * distance - w / 2, cy + dy * distance - h / 2)
                             for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)))
        chosen = None
        for x, y in positions:
            box = (x, y, x + w, y + h)
            if x < 0 or y < 0 or box[2] > size[0] or box[3] > size[1]:
                continue
            if any(x < b[2] + 4 and box[2] + 4 > b[0] and y < b[3] + 4 and box[3] + 4 > b[1] for b in occupied):
                continue
            corners = [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]
            interior = all(point_inside(p, polygon) for p in corners)
            # Bounding boxes conservatively keep exterior labels off other furniture.
            blocked = any(x < max(p[0] for p in poly) and x+w > min(p[0] for p in poly) and
                          y < max(p[1] for p in poly) and y+h > min(p[1] for p in poly)
                          for poly in polygons if poly is not polygon or not interior)
            if not blocked:
                chosen = (box, (cx, cy), not interior, (left, top))
                break
        if chosen is None:
            raise ValueError(f"无法为 {label} 放置清晰且不重叠的 ID；请检查家具数量/ID 长度")
        occupied.append(chosen[0])
        result.append(chosen)
    return result


def render(semantic_yaml, output):
    semantic_yaml, output = Path(semantic_yaml).resolve(), Path(output).resolve()
    document = yaml.safe_load(semantic_yaml.read_text())
    if any(document.get(k) != v for k, v in dict(schema_version=1, source_robot="go2", frame_id="map", units="meters").items()):
        raise ValueError("语义地图身份或 schema_version 不匹配")
    declared = dict(document["source_map"])
    for key in ("yaml", "image"):
        declared[key] = str((semantic_yaml.parent / declared[key]).resolve())
    source = load_map(declared["yaml"])
    if source != declared:
        raise ValueError("源地图 SHA256、图片关联或元数据不匹配")
    protected = [semantic_yaml, Path(source["yaml"]), Path(source["image"])]
    furniture = document["furniture"]
    if len({item["id"] for item in furniture}) != len(furniture):
        raise ValueError("重复家具 ID")
    sessions = set()
    for record in document["point_records"]:
        path = (semantic_yaml.parent / record["file"]).resolve()
        protected.append(path)
        if sha256(path) != record["sha256"]:
            raise ValueError("原始选点 SHA256 不匹配；先结束录点再生成语义层")
        if record["session"] in sessions:
            raise ValueError("重复选点 session")
        sessions.add(record["session"])
        groups = [f for f in furniture if f["session"] == record["session"]]
        checked = build_semantic(source["yaml"], path, groups)
        if checked["point_records"][0]["session"] != record["session"] or checked["furniture"] != groups:
            raise ValueError("polygon 或 session 与原始 point_indices 不一致")
    if any(f["session"] not in sessions for f in furniture):
        raise ValueError("家具缺少对应选点 session")
    if output.suffix.lower() != ".png" or any(output == p or (output.exists() and output.samefile(p)) for p in protected):
        raise ValueError("输出必须是独立 PNG，不能覆盖源地图、语义 YAML 或原始记录")
    # Nearest-neighbor enlargement preserves occupancy pixels and makes IDs readable.
    scale = 6
    with Image.open(source["image"]) as raw:
        image = raw.convert("RGBA").resize((source["width"] * scale, source["height"] * scale), Image.NEAREST)
    overlay = Image.new("RGBA", image.size)
    draw = ImageDraw.Draw(overlay)
    colors = [(30, 110, 230), (230, 95, 25), (25, 155, 80), (155, 65, 210), (205, 45, 105), (15, 155, 175)]
    polygons = []
    for i, item in enumerate(furniture):
        polygon = [tuple((v + 0.5) * scale - 0.5 for v in map_to_pixel(p, source)) for p in item["polygon"]]
        polygons.append(polygon)
        color = colors[i % len(colors)]
        draw.polygon(polygon, fill=color + (45,))
        draw.line(polygon + polygon[:1], fill=color + (255,), width=3, joint="curve")
    image = Image.alpha_composite(image, overlay)
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("DejaVuSans.ttf", 20)
    labels = [f["id"] for f in furniture]
    boxes = label_boxes(polygons, labels, draw, font, image.size)
    for i, (label, (box, center, leader, offset)) in enumerate(zip(labels, boxes)):
        color = colors[i % len(colors)]
        if leader:
            end = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
            draw.line((center, end), fill=color, width=2)
        draw.rounded_rectangle(box, radius=4, fill="white", outline=color, width=2)
        draw.text((box[0] + 6 - offset[0], box[1] + 5 - offset[1]), label, font=font, fill="black")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent, prefix=".semantic-render-") as temporary:
        pending = Path(temporary) / "annotated.png"
        image.convert("RGB").save(pending)
        backup_existing(output)
        pending.replace(output)
    print(f"已渲染 {len(furniture)} 件家具: {output}")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--semantic-yaml", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        render(args.semantic_yaml, args.output)
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as error:
        parser.exit(1, f"标注失败: {error}\n")


if __name__ == "__main__":
    main()
