import pybullet as p
import pybullet_data
import time
import os

# ================================
# Config
# ================================
GUI = True

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(
    PROJECT_ROOT,
    "urdf",
    "davinci_gym_lite.urdf"
)

# ================================
# Connect to PyBullet
# ================================
if GUI:
    p.connect(p.GUI)
else:
    p.connect(p.DIRECT)

p.resetSimulation()
p.setGravity(0, 0, -9.81)

# ================================
# Search paths
# ================================
p.setAdditionalSearchPath(PROJECT_ROOT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# ================================
# Ground plane
# ================================
plane_id = p.loadURDF("plane.urdf")

# ================================
# Load robot
# ================================
print("📦 Loading URDF from:")
print(URDF_PATH)

robot_id = p.loadURDF(
    URDF_PATH,
    basePosition=[0, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True,
    flags=p.URDF_USE_SELF_COLLISION
)

print(f"✅ Robot loaded successfully (id={robot_id})")

# ================================
# Inspect joints
# ================================
num_joints = p.getNumJoints(robot_id)
print(f"\n🔧 Number of joints: {num_joints}\n")

joint_infos = {}
tower_joint_id = None

for jid in range(num_joints):
    info = p.getJointInfo(robot_id, jid)

    joint_name = info[1].decode("utf-8")
    joint_type = info[2]
    lower_limit = info[8]
    upper_limit = info[9]
    axis = info[13]

    joint_infos[jid] = {
        "name": joint_name,
        "type": joint_type,
        "lower": lower_limit,
        "upper": upper_limit,
        "axis": axis,
    }

    if joint_name == "tower_to_base":
        tower_joint_id = jid

    print(f"[{jid}] {joint_name}")
    print(f"     type   : {joint_type}")
    print(f"     limit  : ({lower_limit:.3f}, {upper_limit:.3f})")
    print(f"     axis   : {axis}")

# ================================
# Create debug sliders
# ================================
print("\n🎛 Creating debug sliders...\n")

slider_ids = {}

# ---- 1️⃣ 特別處理 tower height ----
if tower_joint_id is not None:
    j = joint_infos[tower_joint_id]

    tower_slider = p.addUserDebugParameter(
        paramName="🗼 Tower Height (m)",
        rangeMin=j["lower"],
        rangeMax=j["upper"],
        startValue=(j["lower"] + j["upper"]) * 0.5
    )

    slider_ids[tower_joint_id] = {
        "slider": tower_slider,
        "force": 2000  # ✅ tower 需要很大的力
    }

    print("✅ Tower height slider created.")

# ---- 2️⃣ 其餘 joints（自動）----
for jid, jinfo in joint_infos.items():

    if jinfo["type"] == p.JOINT_FIXED:
        continue

    if jid == tower_joint_id:
        continue  # ✅ 已單獨處理過

    lo = jinfo["lower"]
    hi = jinfo["upper"]

    if lo >= hi:
        lo, hi = -3.14, 3.14

    slider = p.addUserDebugParameter(
        paramName=jinfo["name"],
        rangeMin=lo,
        rangeMax=hi,
        startValue=0.0
    )

    slider_ids[jid] = {
        "slider": slider,
        "force": 50
    }

# ================================
# Main simulation loop
# ================================
print("\n✅ Debug sliders ready.")
print("👉 Use the GUI sliders to move joints (including tower height).\n")

while True:
    for jid, cfg in slider_ids.items():
        target = p.readUserDebugParameter(cfg["slider"])

        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=jid,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=cfg["force"]
        )

    p.stepSimulation()
    time.sleep(1.0 / 240.0)
