#!/usr/bin/env bash
# touch.sh name Xbl_mm Ybl_mm pitch roll : hover 1 cm above (old model), image, 2 mm steps down to contact, lift 2 cm.
set -u
S=/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5629d170-20ca-4b0e-9013-c5a2431e36c6/scratchpad
C=$S/hw/call.sh; O=$S/calib2/touch; N=$1; P=$4; R=$5
X=$(python3 -c "print(round($2/1000-0.0592,4))"); Y=$(python3 -c "print(round($3/1000+0.05,4))")
python3 -c "import sys; sys.exit(0 if 0.18<=$X<=0.34 and -0.09<=$Y<=0.19 else 1)" || { echo "SAFETY REJECT $X $Y"; exit 2; }
mv() { $C move_arm_cartesian "{\"x\":$X,\"y\":$Y,\"z\":$1,\"pitch\":$P,\"speed_scale\":$2,\"settle\":\"final\",\"wrist_roll\":$R}" 2>&1 | sed '/robot_events/d'; }
st() { python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('status'), json.dumps(d.get('residual_error')))"; }
state() { $C get_arm_state '{}' 2>&1 | sed '/robot_events/d'; }
img() { i=$($C get_camera_image '{"camera":"gripper","max_px":768}' 2>&1 | sed -n 's/^IMAGE //p'); cp "$i" $O/$N.$1.jpg; }
echo "transit: $(mv 0.0 0.2 | st)"
r=$(mv -0.074 0.2 | st); echo "pre-hover -0.074: $r"; case "$r" in converged*) ;; *) echo ABORT; mv 0.0 0.2 >/dev/null; exit 3;; esac
r=$(mv -0.094 0.15 | st); echo "hover -0.094: $r"; case "$r" in converged*) ;; *) echo ABORT; mv 0.0 0.2 >/dev/null; exit 3;; esac
sleep 0.5; state > $O/$N.hover.json; img hover
for z in $(python3 -c "print(' '.join(f'{-0.096-0.002*i:.3f}' for i in range(10)))"); do
  out=$(mv $z 0.15); s=$(echo "$out" | st); state > $O/$N.step.json
  fz=$(python3 -c "import json; print(round(json.load(open('$O/$N.step.json'))['tool_pose']['z'],4))")
  lag=$(python3 -c "print(round($fz-($z),4))"); echo "cmd $z -> $s fk_z $fz lag $lag"
  if [[ "$s" != converged* ]] || python3 -c "import sys; sys.exit(0 if $lag >= 0.002 else 1)"; then
    cp $O/$N.step.json $O/$N.contact.json; echo "$out" > $O/$N.contact_move.json
    echo "CONTACT at cmd $z fk_z $fz"; r=$(mv -0.074 0.15 | st); echo "lift: $r"; break
  fi
  cp $O/$N.step.json $O/$N.last_free.json; img last_free
done
[ -f $O/$N.contact.json ] || { echo "NO CONTACT down to -0.114"; mv -0.074 0.15 | st; }
echo "up: $(mv 0.0 0.2 | st)"
