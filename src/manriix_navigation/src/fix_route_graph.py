#!/usr/bin/env python3
"""
Fix Route Graph GeoJSON
========================
Post-processor for Nav2 RViz Route Tool output.

Fixes:
  1. Empty MultiLineString edges -> LineString with node coordinates
  2. Adds missing reverse edges (makes fully bidirectional)
  3. Cleans format to match Nav2 depot_graph.geojson convention

Usage:
  python3 fix_route_graph.py <input.geojson> [output.geojson]

If output is omitted, overwrites the input file (backup saved as .bak).
"""

import json
import sys
import shutil
import os


def fix_route_graph(input_path, output_path=None):
    if output_path is None:
        output_path = input_path
        # Create backup
        bak = input_path + '.bak'
        shutil.copy2(input_path, bak)
        print(f"Backup saved: {bak}")

    with open(input_path) as f:
        data = json.load(f)

    # Separate nodes and edges
    nodes = []
    edges = []
    for feat in data['features']:
        gtype = feat['geometry'].get('type', '')
        if gtype == 'Point':
            nodes.append(feat)
        elif gtype in ('LineString', 'MultiLineString'):
            edges.append(feat)

    # Build node coordinate lookup
    node_coords = {}
    for n in nodes:
        nid = n['properties']['id']
        node_coords[nid] = n['geometry']['coordinates']

    print(f"Nodes: {len(nodes)}")
    print(f"Original edges: {len(edges)}")

    # Collect existing directed pairs
    existing_pairs = set()
    for e in edges:
        s = e['properties']['startid']
        t = e['properties']['endid']
        existing_pairs.add((s, t))

    # Find missing reverse edges
    missing = []
    for s, t in list(existing_pairs):
        if (t, s) not in existing_pairs:
            missing.append((t, s))

    if missing:
        print(f"Missing reverse edges: {len(missing)}")
        for t, s in missing:
            print(f"  Adding: {t} -> {s}")
    else:
        print("All edges already bidirectional")

    # Count empty edges
    empty_count = sum(
        1 for e in edges
        if e['geometry'].get('type') == 'MultiLineString'
        or not e['geometry'].get('coordinates')
    )
    if empty_count:
        print(f"Empty edge geometries to fix: {empty_count}")

    # Build clean nodes (depot_graph format)
    clean_nodes = []
    for n in nodes:
        clean_nodes.append({
            "type": "Feature",
            "properties": {"id": n['properties']['id']},
            "geometry": {
                "type": "Point",
                "coordinates": n['geometry']['coordinates']
            }
        })

    # Build clean bidirectional edges
    all_pairs = set(existing_pairs)
    for t, s in missing:
        all_pairs.add((t, s))

    # Edge IDs start at 10000 (depot convention)
    clean_edges = []
    edge_id = 10000
    # Sort for consistent output
    for s, t in sorted(all_pairs):
        if s not in node_coords or t not in node_coords:
            print(f"  WARNING: Edge {s}->{t} references unknown node, skipping")
            continue
        clean_edges.append({
            "type": "Feature",
            "properties": {"id": edge_id, "startid": s, "endid": t},
            "geometry": {
                "type": "LineString",
                "coordinates": [node_coords[s], node_coords[t]]
            }
        })
        edge_id += 1

    # Determine graph name from filename
    graph_name = os.path.splitext(os.path.basename(output_path))[0]

    # Assemble final GeoJSON (depot_graph format)
    output = {
        "type": "FeatureCollection",
        "name": graph_name,
        "crs": {"type": "name", "properties": {"name": "urn:ogc:def:crs:EPSG::3857"}},
        "features": clean_nodes + clean_edges
    }

    with open(output_path, 'w') as f:
        json.dump(output, f, indent=4)

    print(f"\nResult: {len(clean_nodes)} nodes, {len(clean_edges)} edges")
    print(f"Saved: {output_path}")

    # Final verification
    pairs_check = set()
    for e in clean_edges:
        pairs_check.add((e['properties']['startid'], e['properties']['endid']))
    all_bidi = True
    for s, t in list(pairs_check):
        if (t, s) not in pairs_check:
            print(f"  WARNING: {s}->{t} still missing reverse!")
            all_bidi = False
    if all_bidi:
        print("✓ All edges bidirectional")
        print("✓ All edges have LineString coordinates")
        print("✓ Format matches depot_graph.geojson")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 fix_route_graph.py <input.geojson> [output.geojson]")
        print("  If output omitted, overwrites input (backup saved as .bak)")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    fix_route_graph(input_file, output_file)