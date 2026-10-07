"""All tunable values in one place. Units: m, rad, s (sim time unless noted)."""
import math
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
SCENE = WS / "models/fr3_hand_scene.xml"
LOG_DIR = WS / "logs"

JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
# Official SRDF "ready" group_state (robots/common/group_definition.xacro)
HOME = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]
VMAX = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]  # joint_limits.yaml

# control (proposal, not official values)
KP, KD = 400.0, 40.0   # critically damped; higher Kp limits frictionloss-induced steady error
SPEED_FRAC = 0.4          # peak joint speed <= 40% of VMAX
T_MIN = 2.0

# target sampling, base(world) frame, tcp site, tool z-axis -> world -z
BOX = dict(x=(0.30, 0.60), y=(-0.30, 0.30), z=(0.15, 0.55))
R_MIN = 0.30
FIXED_TARGET = (0.45, 0.20, 0.35)
MAX_CANDIDATES = 50
GEN_WALL_LIMIT = 3.0      # s, real time
LIMIT_MARGIN = 0.05
CLEARANCE = 0.01          # contact margin used in scratch checks (m)
FLOOR_CLEAR = 0.05        # min distance hand/fingers/link5-7 to floor at goal
PATH_DQ = 0.02            # max joint change between path check samples

# success criteria
POS_TOL, ORI_TOL = 0.005, math.radians(3.0)
Q_TOL, QD_TOL = 0.01, 0.02
HOLD = 0.3
HOME_HOLD = 0.5

# failure detection
HOME_TIMEOUT = 5.0
SETTLE_EXTRA = 5.0
TRACK_ERR_MAX = 0.2
STALL_WALL = 2.0
DISK_MIN_MB = 200
DISK_CHECK_WALL = 30.0

# logs: logs/ total budget (run logs) + failure records
RUN_LOG_BYTES, RUN_LOG_BACKUPS = 2_000_000, 2   # <= 6 MB per run
RUN_LOGS_TOTAL = 20_000_000                     # all run_*.log* together
FAILURE_KEEP = 20                               # newest failure_*.json kept
