#!/usr/bin/env bash
# cap.sh name x y z pitch roll : transit (if needed) -> move -> state -> image -> state. Arm frame, safety-checked.
set -u
S=/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5629d170-20ca-4b0e-9013-c5a2431e36c6/scratchpad
C=$S/hw/call.sh; O=$S/calib2/caps; N=$1; X=$2; Y=$3; Z=$4; P=$5; R=$6
python3 - "$X" "$Y" "$Z" <<'P' || { echo "SAFETY REJECT"; exit 2; }
import sys; x,y,z=map(float,sys.argv[1:4])
ok = 0.18<=x<=0.34 and -0.09<=y<=0.19 and -0.094<=z<=0.10
sys.exit(0 if ok else 1)
P
mv() { $C move_arm_cartesian "{\"x\":$1,\"y\":$2,\"z\":$3,\"pitch\":$4,\"speed_scale\":0.2,\"settle\":\"final\",\"wrist_roll\":$5}" 2>&1; }
chk() { python3 -c "
import sys,json; t=sys.stdin.read(); d=json.loads(t.split('\n{\"robot_events')[0])
st=d.get('status'); res=d.get('residual_error') or {}; big={k:v for k,v in res.items() if abs(v)>0.1}
print('status',st,'residual',res,'achieved',d.get('achieved_tool_pose'))
sys.exit(0 if st=='converged' and not big else 1)"; }
# transit: current z -> 0.03 at the target x,y with target roll (roll changes only up here)
ok=0; for TZ in 0.03 0.0 -0.03 $Z; do if python3 -c "import sys; sys.exit(0 if $TZ >= $Z else 1)"; then mv $X $Y $TZ $P $R | chk && { ok=1; break; }; fi; done
[ $ok = 1 ] || { echo "TRANSIT FAILED"; exit 3; }
mv $X $Y $Z $P $R | chk || { echo "MOVE FAILED"; exit 3; }
sleep 1
$C get_arm_state '{}' 2>&1 | sed '/robot_events/d' > $O/$N.state1.json
img=$($C get_camera_image '{"camera":"gripper","max_px":768}' 2>&1 | sed -n 's/^IMAGE //p'); cp "$img" $O/$N.jpg
$C get_arm_state '{}' 2>&1 | sed '/robot_events/d' > $O/$N.state2.json
python3 -c "
import json; a=json.load(open('$O/$N.state1.json'))['positions']; b=json.load(open('$O/$N.state2.json'))['positions']
print('joint drift', max(abs(a[k]-b[k]) for k in a)); print(json.load(open('$O/$N.state1.json'))['tool_pose'])"
