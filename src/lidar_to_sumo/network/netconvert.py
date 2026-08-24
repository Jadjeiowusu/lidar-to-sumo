"""SUMO network emission and compilation.

Writes the node and edge descriptions implied by the fitted geometry, invokes
``netconvert``, and determines the constant coordinate offset that
``netconvert`` introduces when it normalises the network origin.

That offset is the single most consequential value in the toolkit. If it is
wrong, every replayed vehicle lands in the wrong place, the replay runs
cleanly, and nothing warns you. It must be re-derived whenever the network is
regenerated.
"""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .axis_fitting import ApproachAxis, classify

__all__ = ["write_network_files", "compile_network", "derive_netconvert_offset"]


def _outward_endpoint(
    axis: ApproachAxis,
    junction: np.ndarray,
    labels: dict[str, str],
    axis_length_m: float,
) -> np.ndarray:
    """Place the far node on the approach's own side of the junction.

    The canonicalised axis direction usually points outward already, but a
    probe check is cheap and prevents an edge being drawn across the junction
    into the opposing approach.
    """
    endpoint = junction + axis.direction * axis_length_m

    probe = junction + axis.direction * 10.0
    probe_label = classify(np.array([probe[0]]), np.array([probe[1]]), labels)[0]
    if probe_label != axis.label:
        endpoint = junction - axis.direction * axis_length_m

    return endpoint


def write_network_files(
    axes: dict[str, ApproachAxis | None],
    junction: np.ndarray,
    approaches: list[dict],
    labels: dict[str, str],
    outdir: Path,
    *,
    axis_length_m: float = 90.0,
    prefix: str = "lidar",
) -> tuple[Path, Path]:
    """Emit SUMO ``.nod.xml`` and ``.edg.xml`` files from fitted geometry.

    Args:
        axes: Fitted approach axes.
        junction: Estimated junction centre, sensor frame.
        approaches: Approach configuration entries, each carrying ``label``,
            ``ingress_lanes``, ``egress_lanes`` and ``egress_label``.
        labels: Quadrant mapping, used for the endpoint side check.
        outdir: Destination directory.
        axis_length_m: Edge length drawn outward from the junction.
        prefix: Output filename stem.

    Returns:
        Paths to the written node and edge files.
    """
    outdir.mkdir(parents=True, exist_ok=True)

    nodes = [
        "<nodes>",
        f'  <node id="J" x="{junction[0]:.2f}" y="{junction[1]:.2f}" type="traffic_light"/>',
    ]
    edges = ["<edges>"]

    for spec in approaches:
        label = spec["label"]
        axis = axes.get(label)
        if axis is None:
            continue

        endpoint = _outward_endpoint(axis, junction, labels, axis_length_m)
        node_id = f"end_{label}"

        nodes.append(f'  <node id="{node_id}" x="{endpoint[0]:.2f}" y="{endpoint[1]:.2f}"/>')
        edges.append(
            f'  <edge id="{label}" from="{node_id}" to="J" '
            f'numLanes="{spec["ingress_lanes"]}" spreadType="center"/>'
        )
        edges.append(
            f'  <edge id="{spec["egress_label"]}" from="J" to="{node_id}" '
            f'numLanes="{spec["egress_lanes"]}" spreadType="center"/>'
        )

    nodes.append("</nodes>")
    edges.append("</edges>")

    nod_path = outdir / f"{prefix}.nod.xml"
    edg_path = outdir / f"{prefix}.edg.xml"
    nod_path.write_text("\n".join(nodes))
    edg_path.write_text("\n".join(edges))

    return nod_path, edg_path


def compile_network(nod_path: Path, edg_path: Path, net_path: Path) -> Path:
    """Compile node and edge files into a SUMO network with ``netconvert``.

    Raises:
        RuntimeError: If ``netconvert`` is not on PATH or exits non-zero.
    """
    if shutil.which("netconvert") is None:
        raise RuntimeError(
            "netconvert not found on PATH. Install SUMO 1.18+ and ensure its "
            "bin directory is on PATH."
        )

    result = subprocess.run(
        [
            "netconvert",
            "--node-files", str(nod_path),
            "--edge-files", str(edg_path),
            "-o", str(net_path),
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"netconvert failed:\n{result.stderr}")

    return net_path


def derive_netconvert_offset(net_path: Path, junction: np.ndarray) -> np.ndarray:
    """Determine the offset netconvert applied when normalising the origin.

    ``netconvert`` translates the network so its bounding box starts at the
    origin. The translation is constant for a given network and is recorded in
    the ``<location netOffset=...>`` element, which is read here and
    cross-checked against the compiled junction position.

    Args:
        net_path: Compiled ``.net.xml``.
        junction: Junction centre in the pre-compilation sensor frame.

    Returns:
        Offset such that ``p_sumo = p_lidar + offset``.

    Raises:
        RuntimeError: If the network declares no location element.
    """
    root = ET.parse(net_path).getroot()
    location = root.find("location")
    if location is None or "netOffset" not in location.attrib:
        raise RuntimeError(f"No <location netOffset=...> element in {net_path}")

    offset = np.array([float(v) for v in location.attrib["netOffset"].split(",")])

    junction_node = root.find(".//junction[@id='J']")
    if junction_node is not None:
        compiled = np.array([float(junction_node.get("x")), float(junction_node.get("y"))])
        residual = np.linalg.norm((junction + offset) - compiled)
        if residual > 1.0:
            raise RuntimeError(
                f"netconvert offset {offset} does not reconcile the junction centre "
                f"(residual {residual:.2f} m). Verify the network before replaying — "
                f"a wrong offset produces a silent, meaningless replay."
            )

    return offset