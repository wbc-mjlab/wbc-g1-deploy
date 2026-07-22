# Operator guide — stand / floor → Gen → clips

Joystick recipes for `wbc_g1_ctrl` + `wbc_reference_node`.
**LT/RT ≈ L2/R2.** Both processes on the same NIC.

Launch (tmux): `scripts/run_gen_controller.sh eth0`  
Or: `build/wbc_g1_ctrl --network=eth0` and `build/wbc_reference_node --network=eth0`  
Build / tmux options: [`gen_controller.md`](gen_controller.md).

`RT + A` = **enable WBC on the controller only** (never starts getup or plays a clip).  
Safety anytime: `LT + B` → Passive.

---

## Start from standing

Robot is already standing.

| Step | Press | Result |
|------|-------|--------|
| 1 | `LT + D-pad up` | FixStand + ref **Standing** (idle frame 0) |
| 2 | Wait ~2 s | Pose settle |
| 3 | `RT + A` | Enable WBC (tracks idle) |
| 4 | Continue below | Gen walk / run, or stay in clips |

No getup step.

## Start from floor

Robot is **already on the floor** — do **not** liedown first.

| Step | Press | Result |
|------|-------|--------|
| 1 | `LT + D-pad down` | FloorReady + ref **Down** (getup frame 0) |
| 2 | Wait ~2 s | Pose settle |
| 3 | `RT + A` | Enable WBC (still holding getup frame 0) |
| 4 | `RT + D-pad up` | Play getup → **Standing** |
| 5 | Wait until getup ends | Idle hold; Gen / clips unlocked |
| 6 | Continue below | Gen walk / run, or stay in clips |

While **Down**, only `RT + up` can getup (browse / Gen / idle play blocked).

---

## After Standing — Gen walk / run

Same for both start paths once you are **Standing** with WBC enabled:

| Step | Press | Result |
|------|-------|--------|
| 1 | `RT + Y` | Enter **Generator** |
| 2 | Sticks; hold `RT` = sprint / `RB` = crouch | Walk / sprint / crouch |

### Gen controls

| Action | Input |
|--------|--------|
| Forward / back (`vx`) | Left stick **Y** |
| Strafe (`vy`) | Left stick **X** |
| Yaw (`wz`) | Right stick **X** |
| Sprint (`sprint_height`, cruise × `sprint_vel_mult`) | Hold **RT** |
| Crouch (`crouch_height`, cruise × `crouch_vel_mult`) | Hold **RB** (R1) |

## Back to clips mode

| From | Press | Result |
|------|-------|--------|
| Gen | `RT + X` | Leave Gen → Standing idle hold, then browse/play |
| Gen | `A` (no RT) | Same — Standing idle hold |
| Standing | `RT + D-pad left/right` | Browse library |
| Standing | `A` (no RT) | Play selected clip |

Returning to clips always republishes **default idle frame 0** first.

---

## Quick reference

| Chord | When | Meaning |
|-------|------|---------|
| `LT + D-pad up` | Prep | Standing / FixStand |
| `LT + D-pad down` | Prep | Floor / Down (getup frame 0) |
| `RT + A` | Prep done | Enable WBC |
| `RT + D-pad up` | Down only | Play getup |
| `RT + Y` | Standing | Enter Gen |
| `RT + X` | Gen | Back to clips (idle hold) |
| `RT + D-pad down` | Standing | Liedown → Down again |
| `LT + B` | Anytime | Passive |

Architecture / DDS: [`architecture.md`](architecture.md).
