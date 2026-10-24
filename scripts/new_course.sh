#!/usr/bin/env bash
# Generate textures (once) and a course world for each seed given.
#   scripts/new_course.sh 0 1 2      -> sim/worlds/generated/course_{0,1,2}.sdf
# Extra gen_course.py flags go after "--":  scripts/new_course.sh 7 -- --light-scale 0.4
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
seeds=(); while [[ $# -gt 0 && "$1" != "--" ]]; do seeds+=("$1"); shift; done
[[ "${1:-}" == "--" ]] && shift
[[ ${#seeds[@]} -eq 0 ]] && seeds=(0)
[[ -f "$ROOT/sim/models/vt_textures/materials/textures/floor.png" ]] || python3 "$ROOT/sim/tools/gen_textures.py"
for s in "${seeds[@]}"; do python3 "$ROOT/sim/tools/gen_course.py" --seed "$s" "$@"; done
