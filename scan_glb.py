# scan_glb.py
# pip install trimesh numpy

import trimesh
import numpy as np
from collections import defaultdict

GLB_PATH = "mekatronik.glb"

def aabb_world(scene, node, geom_name):
    T, _ = scene.graph[node]
    g = scene.geometry[geom_name]
    b = g.bounds
    mins, maxs = b[0], b[1]
    corners = np.array([
        [mins[0], mins[1], mins[2]],
        [mins[0], mins[1], maxs[2]],
        [mins[0], maxs[1], mins[2]],
        [mins[0], maxs[1], maxs[2]],
        [maxs[0], mins[1], mins[2]],
        [maxs[0], mins[1], maxs[2]],
        [maxs[0], maxs[1], mins[2]],
        [maxs[0], maxs[1], maxs[2]],
    ])
    corners_h = np.hstack([corners, np.ones((8, 1))])
    world = (T @ corners_h.T).T[:, :3]
    wmin = world.min(axis=0)
    wmax = world.max(axis=0)
    center = (wmin + wmax) / 2.0
    size = (wmax - wmin)
    return center, size

def main():
    scene = trimesh.load(GLB_PATH, force="scene")

    color_map = defaultdict(list)
    for node in scene.graph.nodes_geometry:
        T, geom_name = scene.graph[node]
        g = scene.geometry[geom_name]
        mat_name = getattr(getattr(g.visual, "material", None), "name", "NO_MAT")
        center, size = aabb_world(scene, node, geom_name)

        # piston benzeri parça: bir ekseni diğerlerinden belirgin büyük olsun
        major = int(np.argmax(size))
        ratio = (size[major] / (np.sort(size)[-2] + 1e-9)) if size.max() > 0 else 0

        color_map[mat_name].append({
            "node": node,
            "geom": geom_name,
            "center": center,
            "size": size,
            "major_axis": ["X","Y","Z"][major],
            "ratio": ratio
        })

    # Sadece renkli “boya” materyallerini ve piston gibi görünenleri yazdır
    wanted = [
        "Toz_Boya_(Kırmızı)",
        "Boya_-_Metalik_(Sarı)",
        "Boya_-_Metalik_(Yeşil)",
        "Boya_-_Metalik_(Mavi)",
    ]

    for mat in wanted:
        items = sorted(color_map.get(mat, []), key=lambda d: -d["ratio"])
        print("\n===", mat, "===")
        for it in items[:10]:
            c = it["center"]; s = it["size"]
            print(f"- node: {it['node']}")
            print(f"  center: [{c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f}]")
            print(f"  size  : [{s[0]:.2f}, {s[1]:.2f}, {s[2]:.2f}]  major={it['major_axis']}  ratio={it['ratio']:.2f}")

if __name__ == "__main__":
    main()
