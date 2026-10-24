#!/usr/bin/env bash
# Score a recorded run (from vt_eval trajectory_recorder) with evo.
#   scripts/eval_ate.sh results/latest
set -eo pipefail
DIR="${1:-results/latest}"
GT="$DIR/ground_truth.tum"; EST="$DIR/estimate.tum"
echo "== ATE (SE3-aligned) =="
evo_ape tum "$GT" "$EST" --align --save_results "$DIR/ape.zip"
echo "== RPE per 1 m =="
evo_rpe tum "$GT" "$EST" --align --delta 1 --delta_unit m --save_results "$DIR/rpe.zip"
evo_traj tum "$EST" --ref "$GT" --align --save_plot "$DIR/trajectory.pdf" --plot_mode xy || true
