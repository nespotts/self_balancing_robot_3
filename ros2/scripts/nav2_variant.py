#!/usr/bin/env python3
"""Make a Nav2 params variant for the controller comparison (sim_e2e_goals.sh / nav_metrics.py).

  python3 ros2/scripts/nav2_variant.py BASE.yaml OUT.yaml rpp_shim|mppi|baseline96 [--from96 OLD_NAV2.yaml]
baseline96: the #96 controller, velocity smoother and collision monitor (from --from96) on top of BASE;
rpp_shim / mppi: BASE with FollowPath replaced by that controller (everything else as in BASE).
"""
import argparse
import copy

import yaml

RPP = {
    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "desired_linear_vel": 0.18,
    "lookahead_dist": 0.45,
    "min_lookahead_dist": 0.3,
    "max_lookahead_dist": 0.7,
    "lookahead_time": 2.0,
    "use_velocity_scaled_lookahead_dist": True,
    "rotate_to_heading_angular_vel": 0.7,
    "transform_tolerance": 0.3,
    "min_approach_linear_velocity": 0.04,
    "approach_velocity_scaling_dist": 0.6,
    "use_collision_detection": True,
    "max_allowed_time_to_collision_up_to_carrot": 1.0,
    "use_regulated_linear_velocity_scaling": True,
    "use_cost_regulated_linear_velocity_scaling": True,
    "regulated_linear_scaling_min_radius": 0.6,
    "regulated_linear_scaling_min_speed": 0.06,
    "use_fixed_curvature_lookahead": False,
    "curvature_lookahead_dist": 0.6,
    "use_rotate_to_heading": False,          # the rotation shim does it
    "allow_reversing": False,
    "rotate_to_heading_min_angle": 0.6,
    "max_angular_accel": 1.2,
    "max_robot_pose_search_dist": 10.0,
    "interpolate_curvature_after_goal": False,
    "cost_scaling_dist": 0.35,
    "cost_scaling_gain": 1.0,
    "inflation_cost_scaling_factor": 3.0,
}
SHIM = {
    "plugin": "nav2_rotation_shim_controller::RotationShimController",
    "primary_controller": RPP["plugin"],
    "angular_dist_threshold": 0.6,           # turn in place first when the path is > 35 deg off
    "angular_disengage_threshold": 0.3,
    "forward_sampling_distance": 0.4,
    "rotate_to_heading_angular_vel": 0.7,
    "max_angular_accel": 1.2,
    "simulate_ahead_time": 1.0,
    "rotate_to_goal_heading": True,
}
MPPI = {
    "plugin": "nav2_mppi_controller::MPPIController",
    "time_steps": 40,                        # 4 s horizon at 0.1 s (one step per control period)
    "model_dt": 0.1,
    "batch_size": 1500,
    "iteration_count": 1,
    "temperature": 0.3,
    "gamma": 0.015,
    "motion_model": "DiffDrive",
    "vx_std": 0.15,
    "wz_std": 0.35,
    "vx_max": 0.18,
    "vx_min": -0.06,                         # a little backing up (with 0.0 it overshot a goal and got stuck)
    "wz_max": 0.9,
    "ax_max": 0.25,                          # a balancing robot leans to accelerate
    "ax_min": -0.3,
    "az_max": 1.2,
    "prune_distance": 1.2,
    "transform_tolerance": 0.3,
    "enforce_path_inversion": False,
    "regenerate_noises": False,
    "visualize": False,
    "critics": ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic",
                "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic", "VelocityDeadbandCritic"],
    "ConstraintCritic": {"enabled": True, "cost_power": 1, "cost_weight": 4.0},
    "GoalCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0, "threshold_to_consider": 1.0},
    "GoalAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.0, "threshold_to_consider": 0.5},
    "PreferForwardCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0, "threshold_to_consider": 0.5},
    "CostCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.81, "near_collision_cost": 253,
                   "critical_cost": 300.0, "consider_footprint": False, "collision_cost": 1000000.0,
                   "near_goal_distance": 0.6, "trajectory_point_step": 2},
    "PathAlignCritic": {"enabled": True, "cost_power": 1, "cost_weight": 14.0, "max_path_occupancy_ratio": 0.05,
                        "trajectory_point_step": 4, "threshold_to_consider": 0.5, "offset_from_furthest": 20,
                        "use_path_orientations": False},
    "PathFollowCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0, "offset_from_furthest": 5,
                         "threshold_to_consider": 1.0},
    "PathAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 2.0, "offset_from_furthest": 4,
                        "threshold_to_consider": 0.5, "max_angle_to_furthest": 1.0, "mode": 0},
    "VelocityDeadbandCritic": {"enabled": True, "cost_power": 1, "cost_weight": 35.0,
                               "deadband_velocities": [0.03, 0.0, 0.03]},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("base")
    ap.add_argument("out")
    ap.add_argument("variant", choices=["rpp_shim", "mppi", "baseline96"])
    ap.add_argument("--from96", default=None)
    a = ap.parse_args()
    d = yaml.safe_load(open(a.base))
    cs = d["controller_server"]["ros__parameters"]
    if a.variant == "baseline96":
        old = yaml.safe_load(open(a.from96))
        for k in ("controller_server", "velocity_smoother", "collision_monitor"):
            d[k] = copy.deepcopy(old[k])
        for cm in ("local_costmap", "global_costmap"):          # #96 inflation
            for layer in ("inflation_layer",):
                d[cm][cm]["ros__parameters"][layer] = copy.deepcopy(old[cm][cm]["ros__parameters"][layer])
    elif a.variant == "rpp_shim":
        cs["FollowPath"] = dict(SHIM, **{k: v for k, v in RPP.items() if k != "plugin"})
    else:
        cs["FollowPath"] = copy.deepcopy(MPPI)
    yaml.safe_dump(d, open(a.out, "w"), sort_keys=False)
    print("wrote", a.out, a.variant)


if __name__ == "__main__":
    main()
