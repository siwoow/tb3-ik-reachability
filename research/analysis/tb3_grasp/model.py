"""URDF FK, exact 4-DOF IK, and conservative collision screening.

Units: metres/radians. Reference frame: URDF root base_footprint.
Mesh bounds enclose the ORIGINAL collision mesh (no visual substitutions).
OBB screening may reject valid configurations; it never shrinks collision shapes.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import hashlib
import json
import math
import xml.etree.ElementTree as ET

import numpy as np

try:
    import fcl
except ImportError:
    fcl = None

ROOT = Path(__file__).resolve().parents[1]
ARM = ("joint1", "joint2", "joint3", "joint4")


def numbers(text, default=(0., 0., 0.)):
    return np.array(default if text is None else [float(x) for x in text.split()])


def transform(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = xyz
    return t


def origin(element):
    e = element.find("origin")
    return np.eye(4) if e is None else transform(numbers(e.get("xyz")), numbers(e.get("rpy")))


def wrap(angle):
    return (np.asarray(angle) + np.pi) % (2*np.pi) - np.pi


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    velocity: float | None
    effort: float | None
    mimic: tuple | None


@dataclass
class Shape:
    link: str
    origin: np.ndarray
    center: np.ndarray
    half: np.ndarray


def obb_overlap(ca, ra, ha, cb, rb, hb):
    """Vectorized full 15-axis separating-axis test, including parallel axes."""
    rot = np.einsum("nji,njk->nik", ra, rb)
    absolute = np.abs(rot) + 1.e-12
    t = np.einsum("nji,nj->ni", ra, cb-ca)
    separated = np.any(np.abs(t) > ha + np.einsum("nij,j->ni", absolute, hb), axis=1)
    separated |= np.any(np.abs(np.einsum("ni,nij->nj", t, rot)) > hb + np.einsum("nij,i->nj", absolute, ha), axis=1)
    for i in range(3):
        i1, i2 = (i+1) % 3, (i+2) % 3
        for j in range(3):
            j1, j2 = (j+1) % 3, (j+2) % 3
            lhs = np.abs(t[:, i2]*rot[:, i1, j] - t[:, i1]*rot[:, i2, j])
            rhs = ha[i1]*absolute[:, i2, j] + ha[i2]*absolute[:, i1, j] + hb[j1]*absolute[:, i, j2] + hb[j2]*absolute[:, i, j1]
            separated |= lhs > rhs
    return ~separated


class RobotModel:
    def __init__(self, urdf=None, bounds=None, srdf=None):
        self.urdf = Path(urdf or ROOT / "assets/resolved.urdf").resolve()
        self.sha256 = hashlib.sha256(self.urdf.read_bytes()).hexdigest()
        root = ET.parse(self.urdf).getroot()
        self.links = {e.get("name"): e for e in root.findall("link")}
        self.joints = {}
        for e in root.findall("joint"):
            lim, ax, mi = e.find("limit"), e.find("axis"), e.find("mimic")
            def limit(key):
                return None if lim is None or lim.get(key) is None else float(lim.get(key))
            kind = e.get("type")
            if kind not in ("fixed", "continuous", "revolute", "prismatic"):
                raise ValueError(f"Unsupported joint type: {kind}")
            j = Joint(e.get("name"), kind, e.find("parent").get("link"), e.find("child").get("link"),
                      origin(e), numbers(None if ax is None else ax.get("xyz"), (1, 0, 0)),
                      limit("lower") if kind in ("revolute", "prismatic") else -math.inf,
                      limit("upper") if kind in ("revolute", "prismatic") else math.inf,
                      limit("velocity"), limit("effort"),
                      None if mi is None else (mi.get("joint"), float(mi.get("multiplier", "1")), float(mi.get("offset", "0"))))
            if kind != "fixed" and (not np.isclose(np.linalg.norm(j.axis), 1) or j.lower is None or j.upper is None):
                raise ValueError(f"Invalid axis or missing limits: {j.name}")
            self.joints[j.name] = j
        children = {j.child for j in self.joints.values()}
        roots = set(self.links)-children
        if roots != {"base_footprint"}:
            raise ValueError(f"Expected single base_footprint root, got {roots}")
        self.order, pending, visited = [], list(self.joints.values()), {"base_footprint"}
        while pending:
            ready = [j for j in pending if j.parent in visited]
            if not ready:
                raise ValueError("Disconnected/cyclic URDF")
            for j in ready:
                self.order.append(j); visited.add(j.child)
            ready_names={j.name for j in ready}
            pending=[j for j in pending if j.name not in ready_names]
        self.lower = np.array([self.joints[n].lower for n in ARM])
        self.upper = np.array([self.joints[n].upper for n in ARM])
        self._validate_4dof()
        self.shoulder = self.fk(np.zeros((1, 4)))["link3"][0, :3, 3]
        a = self.joints["joint3"].origin[:3, 3]
        self.l1, self.alpha = float(np.linalg.norm(a)), math.atan2(a[2], a[0])
        self.l2 = float(self.joints["joint4"].origin[0, 3])
        self.tool = float(self.joints["end_effector_joint"].origin[0, 3])
        bp = Path(bounds or self.urdf.with_name("collision_bounds.json"))
        cached = json.loads(bp.read_text())
        if cached["resolved_urdf_sha256"] != self.sha256:
            raise ValueError("Collision bounds belong to a different URDF; rerun prepare.py")
        self.shapes = [Shape(x["link"], np.array(x["origin"]), np.array(x["center"]), np.array(x["half"])) for x in cached["shapes"]]
        self.fcl_shapes = []
        if fcl is not None:
            for x,s in zip(cached["shapes"],self.shapes):
                if x["kind"] == "mesh":
                    vertices = np.array(x["convex"]["vertices"],dtype=float)
                    faces = np.array(x["convex"]["faces"],dtype=np.int32)
                    encoded = np.column_stack((np.full(len(faces),3,dtype=np.int32),faces)).ravel()
                    self.fcl_shapes.append(fcl.Convex(vertices,len(faces),encoded))
                elif x["kind"] == "box": self.fcl_shapes.append(fcl.Box(*(s.half*2)))
                elif x["kind"] == "cylinder": self.fcl_shapes.append(fcl.Cylinder(x["radius"],x["length"]))
                elif x["kind"] == "sphere": self.fcl_shapes.append(fcl.Sphere(x["radius"]))
        self.collision_backend = "FCL convex hulls + exact primitives, OBB broad phase" if fcl is not None else "conservative OBB only (may reject more)"
        # Structural contact exemptions only. Do NOT use broad SRDF Never/User exemptions.
        exemptions = {frozenset((j.parent, j.child)) for j in self.order}
        exemptions |= {frozenset(pair) for pair in cached.get("structural_exemptions",[])}
        srdf = Path(srdf or ROOT/"vendor/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/config/turtlebot3_manipulation.srdf")
        if srdf.exists():
            if "srdf_sha256" in cached and hashlib.sha256(srdf.read_bytes()).hexdigest() != cached["srdf_sha256"]:
                raise ValueError("SRDF differs from cached structural exclusions; rerun prepare.py")
            exemptions |= {frozenset((e.get("link1"), e.get("link2"))) for e in ET.parse(srdf).getroot().findall("disable_collisions") if e.get("reason") == "Adjacent"}
        rigid = {"base_footprint": "base_footprint"}
        self.dynamic = set()
        for j in self.order:
            rigid[j.child] = rigid[j.parent] if j.kind == "fixed" else j.child
            if j.name in ARM or j.parent in self.dynamic or j.kind == "prismatic":
                self.dynamic.add(j.child)
        self.exemptions = exemptions
        self.pairs = [(i, j) for i, a in enumerate(self.shapes) for j, b in enumerate(self.shapes) if i < j
                      and (a.link in self.dynamic or b.link in self.dynamic)
                      and rigid[a.link] != rigid[b.link] and frozenset((a.link, b.link)) not in exemptions]
        left = next(s for s in self.shapes if s.link == "gripper_left_link")
        right = next(s for s in self.shapes if s.link == "gripper_right_link")
        # Conservative inner gap from full finger bounding boxes at zero opening.
        self.zero_gap = float(left.origin[1, 3]+left.center[1]-left.half[1]
                              -right.origin[1, 3]-right.center[1]-right.half[1]
                              +self.joints["gripper_left_joint"].origin[1, 3]
                              -self.joints["gripper_right_joint"].origin[1, 3])
        self.gripper_limits = (max(self.joints[n].lower for n in ("gripper_left_joint", "gripper_right_joint")),
                               min(self.joints[n].upper for n in ("gripper_left_joint", "gripper_right_joint")))

    def _validate_4dof(self):
        if set(j.name for j in self.joints.values() if j.kind == "revolute") != set(ARM):
            raise ValueError("Expected exactly four revolute arm joints")
        for name, parent, child, axis in zip(ARM, ("link1", "link2", "link3", "link4"), ("link2", "link3", "link4", "link5"), ((0,0,1),(0,1,0),(0,1,0),(0,1,0))):
            j = self.joints[name]
            if j.parent != parent or j.child != child or not np.allclose(j.axis, axis) or not np.allclose(j.origin[:3,:3], np.eye(3)):
                raise ValueError(f"Analytic IK incompatible with URDF joint {name}")
        z = self.joints["joint2"].origin[:3,3]
        a, b, c = [self.joints[n].origin[:3,3] for n in ("joint3", "joint4", "end_effector_joint")]
        if not (np.allclose(z[:2],0) and abs(a[1]) < 1.e-9 and a[0] > 0 and a[2] > 0
                and b[0] > 0 and np.allclose(b[1:],0) and c[0] > 0 and np.allclose(c[1:],0)):
            raise ValueError("Unexpected arm geometry; analytic IK must be rederived")
        j1 = self.joints["joint1"]
        for j in self.order:
            if j.child == j1.parent and not np.allclose(j.origin[:3,:3],np.eye(3)):
                raise ValueError("Rotated mount is not supported by this analytic solver")
        m = self.joints["gripper_right_joint"].mimic
        if m != ("gripper_left_joint", 1., 0.):
            raise ValueError("Expected equal-opening opposing-axis gripper mimic")

    def fk(self, q, opening=.019):
        q = np.atleast_2d(np.asarray(q, dtype=float))
        if q.shape[1] != 4 or not np.isfinite(q).all():
            raise ValueError("q must be finite N x 4")
        n = len(q)
        vals = {name: q[:, i] for i, name in enumerate(ARM)}
        vals["gripper_left_joint"] = np.broadcast_to(np.asarray(opening), (n,))
        poses = {"base_footprint": np.broadcast_to(np.eye(4), (n,4,4)).copy()}
        for j in self.order:
            x = vals.get(j.name, np.zeros(n))
            if j.mimic:
                name, scale, offset = j.mimic; x = vals[name]*scale+offset
            vals[j.name] = x
            motion = np.broadcast_to(np.eye(4), (n,4,4)).copy()
            if j.kind in ("revolute", "continuous"):
                a = j.axis; k = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
                motion[:,:3,:3] += np.sin(x)[:,None,None]*k + (1-np.cos(x))[:,None,None]*(k@k)
            elif j.kind == "prismatic":
                motion[:,:3,3] = x[:,None]*j.axis
            poses[j.child] = poses[j.parent] @ j.origin @ motion
        return poses

    def ik(self, points, pitch=math.pi/2, joint_margin=0.):
        """All 4 analytic branches: signed radial direction and elbow branch.

        Local tool +X is the approach axis. Ry(+pi/2) points it down.
        Roll is fixed, yaw is coupled to position; no fictitious 6-DOF target.
        Returns q[N,4 branches,4], valid[N,4]. FK residual is checked.
        """
        p = np.atleast_2d(np.asarray(points, float))
        if p.shape[1] != 3 or not np.isfinite(p).all() or not np.isfinite(pitch):
            raise ValueError("Expected finite XYZ points and pitch")
        d = p-self.shoulder
        rho = np.hypot(d[:,0], d[:,1]); yaw = np.arctan2(d[:,1],d[:,0])
        solutions, flags = [], []
        for sign in (1, -1):
            u = sign*rho-self.tool*math.cos(pitch)
            v = d[:,2]+self.tool*math.sin(pitch)
            cosine = (u*u+v*v-self.l1*self.l1-self.l2*self.l2)/(2*self.l1*self.l2)
            reachable = (np.abs(cosine) <= 1+1.e-12) & (rho > 1.e-8)
            for elbow in (1, -1):
                delta = elbow*np.arccos(np.clip(cosine,-1,1))
                theta = np.arctan2(v,u)-np.arctan2(self.l2*np.sin(delta),self.l1+self.l2*np.cos(delta))
                q2 = wrap(self.alpha-theta); q3 = wrap(-delta-self.alpha)
                q4 = wrap(pitch-q2-q3)
                q = np.column_stack((wrap(yaw+(0 if sign == 1 else np.pi)), q2, q3, q4))
                ok = reachable & np.all((q >= self.lower+joint_margin-1.e-10) & (q <= self.upper-joint_margin+1.e-10),axis=1)
                tip = self.fk(q)["end_effector_link"]
                ok &= np.linalg.norm(tip[:,:3,3]-p,axis=1) < 1.e-7
                solutions.append(q); flags.append(ok)
        return np.stack(solutions,axis=1), np.stack(flags,axis=1)

    def boxes(self, q, opening=.019):
        poses = self.fk(q,opening)
        result = []
        for s in self.shapes:
            t = poses[s.link] @ s.origin
            result.append((np.einsum("nij,j->ni",t[:,:3,:3],s.center)+t[:,:3,3], t[:,:3,:3], s.half))
        return result

    def screen(self, q, opening=.019, margin=0., floor_z=0., object_sphere=None, obstacles=(), floor_margin=.003):
        """Conservative OBB checks at supplied configurations, not a path proof.

        Obstacles are base-frame OBB tuples (center, rotation, half extents).
        Target sphere checks skip fingers only during intended closing (caller).
        """
        q = np.atleast_2d(q); n = len(q)
        opening = np.broadcast_to(np.asarray(opening), (n,))
        valid = np.all((q >= self.lower-1.e-10) & (q <= self.upper+1.e-10),axis=1)
        valid &= (opening >= self.gripper_limits[0]) & (opening <= self.gripper_limits[1])
        boxes = self.boxes(q,opening); floor_clearance = np.full(n,np.inf)
        def native(i,k):
            c,r,_ = boxes[i]; s = self.shapes[i]
            return fcl.CollisionObject(self.fcl_shapes[i],fcl.Transform(r[k],c[k]-r[k]@s.center))
        def too_close(a,b):
            if fcl.collide(a,b,fcl.CollisionRequest(),fcl.CollisionResult()) > 0: return True
            return fcl.distance(a,b,fcl.DistanceRequest(),fcl.DistanceResult()) < margin
        for shape_i,(s, (c,r,h)) in enumerate(zip(self.shapes,boxes)):
            if s.link in self.dynamic:
                clear = c[:,2]-np.einsum("ni,i->n",np.abs(r[:,2,:]),h)-floor_z
                floor_clearance = np.minimum(floor_clearance,clear)
                valid &= clear >= floor_margin
            if object_sphere is not None:
                center, radius, allow_fingers = object_sphere
                if not (allow_fingers and s.link.startswith("gripper_")):
                    local = np.einsum("nji,nj->ni",r,np.asarray(center)-c)
                    distance = np.linalg.norm(np.maximum(np.abs(local)-h,0),axis=1)
                    suspect = distance < radius+margin
                    if fcl is not None:
                        centers = np.broadcast_to(center,(n,3))
                        for k in np.flatnonzero(suspect & valid):
                            target = fcl.CollisionObject(fcl.Sphere(radius),fcl.Transform(centers[k]))
                            suspect[k] = too_close(native(shape_i,k),target)
                    valid &= ~suspect
            for oc, rotation, half in obstacles:
                suspect = obb_overlap(c,r,h+margin/2,np.broadcast_to(oc,c.shape),np.broadcast_to(rotation,r.shape),np.asarray(half)+margin/2)
                if fcl is not None:
                    obstacle = fcl.CollisionObject(fcl.Box(*(np.asarray(half)*2)),fcl.Transform(rotation,oc))
                    for k in np.flatnonzero(suspect & valid): suspect[k] = too_close(native(shape_i,k),obstacle)
                valid &= ~suspect
        for i,j in self.pairs:
            ca,ra,ha = boxes[i]; cb,rb,hb = boxes[j]
            suspect = obb_overlap(ca,ra,ha+margin/2,cb,rb,hb+margin/2)
            if fcl is not None:
                for k in np.flatnonzero(suspect & valid): suspect[k] = too_close(native(i,k),native(j,k))
            valid &= ~suspect
        return valid, floor_clearance

    def joint_quality(self,q):
        q = np.asarray(q)
        return np.min(np.minimum(q-self.lower,self.upper-q)/(self.upper-self.lower),axis=-1)*2

    def keyframe_self_clearance(self,q,opening=.019):
        """Minimum FCL distance over structural nonadjacent pairs/keyframes."""
        if fcl is None: return None
        q=np.atleast_2d(q); boxes=self.boxes(q,opening); smallest=math.inf; pair=None
        for k in range(len(q)):
            bodies=[]
            for s,g,(c,r,_) in zip(self.shapes,self.fcl_shapes,boxes):
                bodies.append(fcl.CollisionObject(g,fcl.Transform(r[k],c[k]-r[k]@s.center)))
            for i,j in self.pairs:
                d=fcl.distance(bodies[i],bodies[j],fcl.DistanceRequest(),fcl.DistanceResult())
                if d < smallest: smallest=float(d); pair=[self.shapes[i].link,self.shapes[j].link]
        return {"distance_m":smallest,"pair":pair,"keyframes_checked":len(q)}
