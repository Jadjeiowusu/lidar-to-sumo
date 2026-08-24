#!/usr/bin/env python3
"""
01_extract_audit.py
Extract LiDAR object detections from every .db3 in a rosbag2 split folder
and produce a per-recording audit summary.

Usage:
    python 01_extract_audit.py /path/to/rosbag2_2026_06_25_split [--inspect] [--topic /ouster/raw_objects]

Outputs (in ./out/):
    detections_<recording>.parquet   one row per detection
    audit_summary.csv                per-recording counts, spans, rates

Assumes the standard rosbag2 SQLite schema (topics, messages) and
JSON-serialized object messages on the LiDAR topic. If your message
payload differs, run with --inspect first: it prints the schema and a
raw sample payload so you can adjust FIELD_MAP below.
"""
import argparse, json, sqlite3, sys
from pathlib import Path
import pandas as pd

# Map JSON keys in the message payload -> canonical column names.
# Adjust after running --inspect if your keys differ.
FIELD_MAP = {
    "id": "track_id", "track_id": "track_id",
    "x": "x", "y": "y", "z": "z",
    "vx": "vx", "vy": "vy",
    "heading": "heading",
    "class": "obj_class", "classification": "obj_class", "label": "obj_class",
    "num_points": "npts", "points": "npts", "point_count": "npts",
}
VEHICLE_CLASSES = {"VEHICLE", "LARGE_VEHICLE"}


def cdr_to_json(blob: bytes):
    """rosbag2 stores CDR-serialized messages; for std_msgs/String-style JSON
    payloads the JSON text sits inside the blob. Find the outermost JSON."""
    for opener, closer in (("{", "}"), ("[", "]")):
        i = blob.find(opener.encode())
        if i == -1:
            continue
        j = blob.rfind(closer.encode())
        if j > i:
            try:
                return json.loads(blob[i:j + 1].decode("utf-8", errors="ignore"))
            except json.JSONDecodeError:
                continue
    return None


def normalize_obj(obj: dict):
    out = {}
    for k, v in obj.items():
        ck = FIELD_MAP.get(k)
        if ck:
            out[ck] = v
    # allow nested position/velocity dicts (velocity keys are x/y in-payload)
    if isinstance(obj.get("position"), dict):
        for c in ("x", "y", "z"):
            if c in obj["position"]:
                out[c] = obj["position"][c]
    if isinstance(obj.get("velocity"), dict):
        for src, dst in (("x", "vx"), ("y", "vy")):
            if src in obj["velocity"]:
                out[dst] = obj["velocity"][src]
    return out


def inspect(db_path: Path, topic_like: str):
    con = sqlite3.connect(db_path)
    print(f"\n=== {db_path.name} ===")
    for name, in con.execute("SELECT name FROM sqlite_master WHERE type='table'"):
        print("table:", name)
    print("\ntopics:")
    for row in con.execute("SELECT id,name,type FROM topics"):
        print(" ", row)
    row = con.execute(
        "SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
        "WHERE t.name LIKE ? LIMIT 1", (f"%{topic_like}%",)).fetchone()
    if row:
        print("\nsample payload (first 600 bytes):\n", row[0][:600])
        parsed = cdr_to_json(row[0])
        print("\nparsed JSON keys:", list(parsed.keys()) if isinstance(parsed, dict)
              else (type(parsed), parsed[:1] if parsed else None))
    con.close()


def extract(db_path: Path, topic: str) -> pd.DataFrame:
    con = sqlite3.connect(db_path)
    cur = con.execute(
        "SELECT m.timestamp, m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
        "WHERE t.name = ? ORDER BY m.timestamp", (topic,))
    rows, n_frames, n_bad = [], 0, 0
    for ts, blob in cur:
        payload = cdr_to_json(blob)
        if payload is None:
            n_bad += 1
            continue
        n_frames += 1
        if isinstance(payload, dict) and "object_list" in payload:
            objs = []
            for fr in payload["object_list"]:
                if isinstance(fr, dict):
                    objs.extend(fr.get("objects") or [])
        elif isinstance(payload, list):
            objs = payload
        else:
            objs = payload.get("objects", payload.get("obstacles", [payload]))
        for o in objs:
            if not isinstance(o, dict):
                continue
            r = normalize_obj(o)
            if "track_id" not in r or "x" not in r:
                continue
            r["t_ns"] = ts
            rows.append(r)
    con.close()
    df = pd.DataFrame(rows)
    if not df.empty:
        for c in ("x", "y", "z", "vx", "vy", "npts"):
            if c in df:
                df[c] = pd.to_numeric(df[c], errors="coerce")
        df["speed"] = (df.get("vx", 0) ** 2 + df.get("vy", 0) ** 2) ** 0.5
        df["t_s"] = (df["t_ns"] - df["t_ns"].min()) / 1e9
    df.attrs.update(n_frames=n_frames, n_bad=n_bad)
    return df


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", type=Path)
    ap.add_argument("--topic", default="/ouster/raw_objects")
    ap.add_argument("--inspect", action="store_true")
    ap.add_argument("--out", type=Path, default=Path("out"))
    args = ap.parse_args()

    # rosbag2 layout: <folder>/<bag_name>/{metadata.yaml, <bag_name>_N.db3}
    # Group db3 files by their bag directory; one "recording" per bag dir.
    all_dbs = sorted(args.folder.rglob("*.db3"))
    if not all_dbs:
        sys.exit(f"No .db3 files found under {args.folder}")
    bags = {}
    for db in all_dbs:
        bags.setdefault(db.parent.name if db.parent != args.folder else db.stem,
                        []).append(db)
    if args.inspect:
        for name, dbs in bags.items():
            inspect(dbs[0], args.topic.strip("/").split("/")[-1])
        return

    args.out.mkdir(exist_ok=True)
    summary = []
    for name, dbs in bags.items():
        parts = [extract(db, args.topic) for db in sorted(dbs)]
        df = pd.concat([p for p in parts if not p.empty], ignore_index=True) \
            if any(not p.empty for p in parts) else pd.DataFrame()
        if not df.empty:
            df.attrs["n_frames"] = sum(p.attrs.get("n_frames", 0) for p in parts)
            df.attrs["n_bad"] = sum(p.attrs.get("n_bad", 0) for p in parts)
            df["t_s"] = (df["t_ns"] - df["t_ns"].min()) / 1e9
        if df.empty:
            print(f"[WARN] {name}: no detections parsed — run with --inspect")
            continue
        veh = df[df["obj_class"].isin(VEHICLE_CLASSES)] if "obj_class" in df else df
        real = veh[veh["npts"] > 0] if "npts" in veh else veh
        dur_min = df["t_s"].max() / 60
        meta = dbs[0].parent / "metadata.yaml"
        start_ns = None
        if meta.exists():
            for line in meta.read_text().splitlines():
                if "nanoseconds_since_epoch" in line:
                    try:
                        start_ns = int(line.split(":")[-1].strip())
                    except ValueError:
                        pass
                    break
        summary.append({
            "recording": name,
            "start_time": (pd.Timestamp(start_ns, unit="ns").isoformat()
                           if start_ns else ""),
            "duration_min": round(dur_min, 1),
            "frames": df.attrs["n_frames"],
            "unparsed_msgs": df.attrs["n_bad"],
            "detections_all": len(df),
            "detections_vehicle": len(veh),
            "detections_after_ghost_removal": len(real),
            "unique_track_ids": real["track_id"].nunique(),
            "tracks_per_min": round(real["track_id"].nunique() / dur_min, 1),
        })
        real.to_parquet(args.out / f"detections_{name}.parquet", index=False)
        print(f"[OK] {name}: {len(real):,} vehicle detections, "
              f"{real['track_id'].nunique():,} tracks, {dur_min:.1f} min")
    pd.DataFrame(summary).to_csv(args.out / "audit_summary.csv", index=False)
    print(f"\nWrote {args.out/'audit_summary.csv'}")


if __name__ == "__main__":
    main()