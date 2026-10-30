"""Audit original assets, write a resolved COPY and conservative collision bounds."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from tb3_grasp.model import ROOT, origin, RobotModel


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def prepare(source):
    source = Path(source).expanduser().resolve()
    tree = ET.parse(source); robot = tree.getroot()
    vendor = ROOT/"vendor/turtlebot3_manipulation"
    assets = ROOT/"assets"; assets.mkdir(exist_ok=True)
    meshes, local_missing = {}, []
    for mesh in robot.findall(".//mesh"):
        name = mesh.get("filename")
        if name in meshes:
            mesh.set("filename",meshes[name]["resolved"]); continue
        if name.startswith("package://"):
            relative = name[len("package://"):]
            local = source.parent/relative
            # Use one pinned, immutable asset snapshot, including during concurrent
            # uploads into robot_models. Never modify that original directory.
            resolved = vendor/relative
        else:
            local = Path(name.removeprefix("file://"))
            if not local.is_absolute(): local = source.parent/local
            resolved = local
        if not local.is_file(): local_missing.append(name)
        if not resolved.is_file(): raise FileNotFoundError(f"Missing mesh (no replacement allowed): {name}")
        geom = trimesh.load(str(resolved),force="mesh")
        if geom.vertices.size == 0 or not np.isfinite(geom.vertices).all(): raise ValueError(f"Invalid mesh {resolved}")
        meshes[name] = {"resolved":str(resolved),"sha256":sha(resolved),"local_present":local.is_file(),
                        "local_matches_official":sha(local)==sha(resolved) if local.is_file() else None,
                        "local_size_bytes":local.stat().st_size if local.is_file() else None,
                        "official_size_bytes":resolved.stat().st_size,"triangles":len(geom.faces)}
        mesh.set("filename",str(resolved))
    resolved_urdf = assets/"resolved.urdf"
    tree.write(resolved_urdf,encoding="utf-8",xml_declaration=True)
    shapes = []
    for link in robot.findall("link"):
        for element in link.findall("collision"):
            geo = element.find("geometry"); shape = list(geo)[0]; kind = shape.tag
            if kind == "mesh":
                mesh = trimesh.load(shape.get("filename"),force="mesh")
                scale = np.fromstring(shape.get("scale","1 1 1"),sep=" ")
                if scale.shape != (3,) or np.any(scale <= 0): raise ValueError("Invalid mesh scale")
                v = mesh.vertices*scale; lo,hi = v.min(axis=0),v.max(axis=0)
                hull = trimesh.Trimesh(vertices=v,faces=mesh.faces,process=False).convex_hull
                convex = {"vertices":hull.vertices.tolist(),"faces":hull.faces.tolist()}
            elif kind == "box":
                hi = np.fromstring(shape.get("size"),sep=" ")/2; lo = -hi
            elif kind == "cylinder":
                hi = np.array([float(shape.get("radius"))]*2+[float(shape.get("length"))/2]); lo = -hi
            elif kind == "sphere":
                hi = np.full(3,float(shape.get("radius"))); lo = -hi
            else: raise ValueError(f"Unsupported collision geometry {kind}")
            shapes.append({"link":link.get("name"),"kind":kind,"origin":origin(element).tolist(),"center":((hi+lo)/2).tolist(),"half":((hi-lo)/2).tolist(),
                           "convex":convex if kind=="mesh" else None,
                           "radius":float(shape.get("radius")) if shape.get("radius") else None,
                           "length":float(shape.get("length")) if shape.get("length") else None})
    srdf=vendor/"turtlebot3_manipulation_moveit_config/config/turtlebot3_manipulation.srdf"
    structural_exemptions=[[e.get("link1"),e.get("link2")] for e in ET.parse(srdf).getroot().findall("disable_collisions") if e.get("reason")=="Adjacent"]
    bounds = {"resolved_urdf_sha256":sha(resolved_urdf),"method":"FCL enclosing mesh convex hulls + exact primitives; conservative OBB broad phase", "shapes":shapes,
              "structural_exemptions":structural_exemptions,"srdf_sha256":sha(srdf)}
    (assets/"collision_bounds.json").write_text(json.dumps(bounds,indent=2))
    report = {"original_urdf":str(source),"original_urdf_sha256":sha(source),"resolved_urdf_sha256":sha(resolved_urdf),
              "official_source":"https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/tree/humble",
              "official_commit":subprocess.check_output(["git","-C",str(vendor),"rev-parse","HEAD"],text=True).strip(),
              "missing_in_original_folder":local_missing,"mesh_count":len(meshes),"meshes":meshes,
              "asset_policy":"Pinned official mesh snapshot, original URDF kinematics/collision definitions unchanged; local mesh differences explicitly reported",
              "links":[{"name":x.get("name"),"visual_count":len(x.findall("visual")),"collision_count":len(x.findall("collision")),"inertial_present":x.find("inertial") is not None} for x in robot.findall("link")],
              "joints":[dict(x.attrib, parent=x.find("parent").attrib,child=x.find("child").attrib,
                             origin=getattr(x.find("origin"),"attrib",{}),axis=getattr(x.find("axis"),"attrib",{}),
                             limits=getattr(x.find("limit"),"attrib",{}),mimic=getattr(x.find("mimic"),"attrib",{})) for x in robot.findall("joint")]}
    m = RobotModel()
    report.update(arm_lower_rad=m.lower.tolist(),arm_upper_rad=m.upper.tolist(),shoulder_base_footprint_m=m.shoulder.tolist(),
                  collision_shapes=len(m.shapes),checked_self_collision_pairs=len(m.pairs),
                  gripper_opening_limits_m=list(m.gripper_limits),conservative_max_inner_gap_m=m.zero_gap+2*m.gripper_limits[1],
                  original_unchanged=sha(source)==report["original_urdf_sha256"])
    (ROOT/"results/asset_report.json").write_text(json.dumps(report,indent=2))
    print(json.dumps({k:report[k] for k in ("mesh_count","missing_in_original_folder","arm_lower_rad","arm_upper_rad","shoulder_base_footprint_m","checked_self_collision_pairs","conservative_max_inner_gap_m","original_unchanged")},indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(); p.add_argument("--urdf",default="~/robot_models/turtlebot3_manipulation.urdf")
    prepare(p.parse_args().urdf)
