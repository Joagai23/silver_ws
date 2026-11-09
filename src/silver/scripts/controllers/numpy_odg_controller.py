import numpy as np
from typing import Tuple, Union
from silver2_constants.isaac_constants import (
    SILVER2_MOUNTS,
    SILVER2_LINKS,
    GAIT_CONFIGS,
    SILVER2_STANDING_ANGLES_DEG,
)

class NumpyOmnidirectionalGaitController:
    def __init__(
        self,
        gait_type: str = "tripod",
        step_height: float = 0.05,
        gait_period: float = 2.0,
    ):
        self.step_height = float(step_height)
        self.T = float(gait_period)

        # Unpack robot link lengths from standardized constants
        self.link_lengths = SILVER2_LINKS
        if len(self.link_lengths) == 3:
            self.l0, self.l2, self.l3 = [float(x) for x in self.link_lengths]
            self.l1 = 0.0
        elif len(self.link_lengths) == 4:
            self.l0, self.l1, self.l2, self.l3 = [float(x) for x in self.link_lengths]

        # Joint limits in radians [coxa, femur, tibia]
        self.q_min = np.array([-np.pi / 2.0, -np.pi / 2.0, -np.pi / 4.0], dtype=np.float64)
        self.q_max = np.array([ np.pi / 2.0,  np.pi / 2.0,  0.75 * np.pi], dtype=np.float64)

        # Leg chirality / mirroring vector
        self.mir = np.array([1.0, 1.0, -1.0, -1.0, -1.0, 1.0], dtype=np.float64)

        if gait_type not in GAIT_CONFIGS:
            raise ValueError(f"Unknown gait type: {gait_type}. Available: {list(GAIT_CONFIGS.keys())}")

        config = GAIT_CONFIGS[gait_type]
        self.beta = float(config["epsilon"])
        self.phases = np.array(config["phases"], dtype=np.float64)

        # Ingest nominal mounting coordinates from constants
        leg_names = list(SILVER2_MOUNTS.keys())
        self.mount_pos = np.array(
            [SILVER2_MOUNTS[name]["pos"] for name in leg_names], dtype=np.float64
        )
        self.mount_yaw = np.array(
            [SILVER2_MOUNTS[name]["yaw"] for name in leg_names], dtype=np.float64
        )

        # Precompute and cache nominal standing foot positions in Body frame (6, 3)
        self.default_feet_body = self.compute_default_feet_body()

    def body_to_local_frame(self, p_body: np.ndarray) -> np.ndarray:
        """
        Transforms foot positions from Body (Chassis) Frame (..., 6, 3)
        to Local Coxa Mount Frames (..., 6, 3).
        """
        dx = p_body[..., 0] - self.mount_pos[:, 0]
        dy = p_body[..., 1] - self.mount_pos[:, 1]
        dz = p_body[..., 2] - self.mount_pos[:, 2]

        c_y = np.cos(-self.mount_yaw)
        s_y = np.sin(-self.mount_yaw)

        px = dx * c_y - dy * s_y
        py = dx * s_y + dy * c_y
        pz = dz

        return np.stack([px, py, pz], axis=-1)

    def local_to_body_frame(self, p_local: np.ndarray) -> np.ndarray:
        """
        Transforms foot positions from Local Coxa Mount Frames (..., 6, 3)
        to Body (Chassis) Frame (..., 6, 3).
        """
        c_y = np.cos(self.mount_yaw)
        s_y = np.sin(self.mount_yaw)

        x_b = self.mount_pos[:, 0] + p_local[..., 0] * c_y - p_local[..., 1] * s_y
        y_b = self.mount_pos[:, 1] + p_local[..., 0] * s_y + p_local[..., 1] * c_y
        z_b = self.mount_pos[:, 2] + p_local[..., 2]

        return np.stack([x_b, y_b, z_b], axis=-1)

    def batch_leg_inv_kine(self, feet_local: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Inverse Kinematics: maps local Cartesian feet (..., 6, 3) to joint targets (..., 6, 3)
        and an admissibility boolean mask.
        """
        feet_local = np.asarray(feet_local, dtype=np.float64)
        has_batch = (feet_local.ndim == 3)
        if not has_batch:
            feet_local = feet_local[np.newaxis, ...]

        x = feet_local[..., 0]
        y = feet_local[..., 1]
        z = feet_local[..., 2]

        L_sq = x**2 + y**2
        L = np.sqrt(np.maximum(L_sq, 1e-8))

        valid_radial = L >= abs(self.l1)
        planar_span_sq = np.maximum(L_sq - self.l1**2, 0.0)
        planar_span = np.sqrt(planar_span_sq)

        beta = np.arctan2(y, x)
        gamma = np.arctan2(self.l1, planar_span)
        q1 = beta - self.mir * gamma

        x3 = planar_span - self.l0
        y3 = z

        a_sq = x3**2 + y3**2
        a = np.sqrt(np.maximum(a_sq, 1e-8))

        cos_q2_arg = (self.l2**2 + a_sq - self.l3**2) / (2.0 * self.l2 * a)
        cos_q3_arg = (self.l2**2 + self.l3**2 - a_sq) / (2.0 * self.l2 * self.l3)

        valid_triangle = (np.abs(cos_q2_arg) <= 1.0) & (np.abs(cos_q3_arg) <= 1.0)

        q2_cos = np.clip(cos_q2_arg, -1.0, 1.0)
        q3_cos = np.clip(cos_q3_arg, -1.0, 1.0)

        q2 = np.arccos(q2_cos) + np.arctan2(y3, x3)
        q3 = np.pi - np.arccos(q3_cos)

        q = np.stack([q1, q2, q3], axis=-1)

        within_limits = np.all(q >= self.q_min, axis=-1) & np.all(q <= self.q_max, axis=-1)
        admissible = valid_radial & valid_triangle & within_limits

        return (q, admissible) if has_batch else (q.squeeze(0), admissible.squeeze(0))

    def batch_leg_for_kine(self, q: np.ndarray) -> np.ndarray:
        """
        Forward Kinematics: maps joint angles (..., 6, 3) to local Cartesian feet (..., 6, 3).
        """
        q = np.asarray(q, dtype=np.float64)
        has_batch = (q.ndim == 3)
        if not has_batch:
            q = q[np.newaxis, ...]

        q1 = q[..., 0]
        q2 = q[..., 1]
        q3 = q[..., 2]

        r_planar = self.l0 + self.l2 * np.cos(q2) + self.l3 * np.cos(q2 - q3)
        z = self.l2 * np.sin(q2) + self.l3 * np.sin(q2 - q3)

        lat_offset = self.mir * self.l1
        x = r_planar * np.cos(q1) - lat_offset * np.sin(q1)
        y = r_planar * np.sin(q1) + lat_offset * np.cos(q1)

        feet_pos = np.stack([x, y, z], axis=-1)
        return feet_pos if has_batch else feet_pos.squeeze(0)

    def admissible(self, feet_local: np.ndarray) -> np.ndarray:
        """Direct analytical reachability check for foot positions in local mount frames."""
        _, is_admissible = self.batch_leg_inv_kine(feet_local)
        return is_admissible

    def compute_default_feet_body(
        self,
        standing_angles_deg: Union[np.ndarray, list] = None
    ) -> np.ndarray:
        """
        Computes nominal standing foot coordinates in Body (chassis) frame (6, 3)
        from joint angles (in degrees).
        """
        if standing_angles_deg is None:
            standing_angles_deg = SILVER2_STANDING_ANGLES_DEG

        q_deg = np.asarray(standing_angles_deg, dtype=np.float64).reshape(6, 3)
        q_rad = np.deg2rad(q_deg)

        feet_local = self.batch_leg_for_kine(q_rad)
        return self.local_to_body_frame(feet_local)

    def compute_foot_trajectories(
        self,
        t: float,
        vx: float,
        vy: float,
        wz: float,
        default_feet_body: np.ndarray = None
    ) -> np.ndarray:
        """
        Computes instantaneous 3D Cartesian foot targets in Body Frame for all 6 legs.
        """
        base_feet = self.default_feet_body if default_feet_body is None else default_feet_body
        tau = np.mod(t / self.T + self.phases, 1.0)

        # Linear + yaw displacement per leg
        sx_lin = vx * self.T * self.beta
        sy_lin = vy * self.T * self.beta

        sx_yaw = -wz * self.mount_pos[:, 1] * self.T
        sy_yaw =  wz * self.mount_pos[:, 0] * self.T

        sx_total = sx_lin + sx_yaw
        sy_total = sy_lin + sy_yaw

        is_stance = tau < self.beta
        u_stance = tau / self.beta
        u_swing = (tau - self.beta) / (1.0 - self.beta)

        dx = np.where(
            is_stance,
            (0.5 - u_stance) * sx_total,
            (u_swing - 0.5) * sx_total
        )
        dy = np.where(
            is_stance,
            (0.5 - u_stance) * sy_total,
            (u_swing - 0.5) * sy_total
        )
        dz = np.where(
            is_stance,
            np.zeros(6, dtype=np.float64),
            self.step_height * np.sin(np.pi * u_swing)
        )

        feet_targets = base_feet.copy()
        feet_targets[:, 0] += dx
        feet_targets[:, 1] += dy
        feet_targets[:, 2] += dz

        return feet_targets

    def change_configuration(
        self,
        q_des: np.ndarray,
        q_cur: np.ndarray,
        gait_t: float = 3.0,
        lift_height: float = 0.05
    ) -> Union[bool, Tuple[np.ndarray, np.ndarray, np.ndarray, int, float]]:
        """Statically stable 3-phase posture transition planner."""
        q_des = np.asarray(q_des, dtype=np.float64).reshape(6, 3)
        q_cur = np.asarray(q_cur, dtype=np.float64).reshape(6, 3)

        if np.allclose(q_des, q_cur, atol=1e-4):
            print("Same pose as before")
            return True

        op = self.batch_leg_for_kine(q_cur)
        np_pos = self.batch_leg_for_kine(q_des)

        nh = np_pos[:, 2].min()
        oh = op[:, 2].min()

        n1 = 0 if abs(nh - oh) < 0.001 else 60
        n2 = 30
        n = n1 + 2 * n2
        delta_t = gait_t / n
        t = np.linspace(0.0, 1.0, n2)

        Qq = np.zeros((18, n), dtype=np.float64)
        Qdot = np.zeros((18, n), dtype=np.float64)
        admiss = np.ones(6, dtype=bool)

        tripod_1_legs = {0, 2, 4}

        for i in range(6):
            op_i = op[i]
            np_i = np_pos[i]

            if n1 > 0:
                xi_1 = np.repeat(op_i[0], n1)
                yi_1 = np.repeat(op_i[1], n1)
                zi_1 = np.linspace(op_i[2], np_i[2], n1)
            else:
                xi_1 = np.empty(0, dtype=np.float64)
                yi_1 = np.empty(0, dtype=np.float64)
                zi_1 = np.empty(0, dtype=np.float64)

            if i in tripod_1_legs:
                xi_2 = np.linspace(op_i[0], np_i[0], n2)
                yi_2 = np.linspace(op_i[1], np_i[1], n2)
                zi_2 = np_i[2] + lift_height * np.sin(np.pi * t)

                xi_3 = np.repeat(np_i[0], n2)
                yi_3 = np.repeat(np_i[1], n2)
                zi_3 = np.repeat(np_i[2], n2)
            else:
                xi_2 = np.repeat(op_i[0], n2)
                yi_2 = np.repeat(op_i[1], n2)
                zi_2 = np.repeat(np_i[2], n2)

                xi_3 = np.linspace(op_i[0], np_i[0], n2)
                yi_3 = np.linspace(op_i[1], np_i[1], n2)
                zi_3 = np_i[2] + lift_height * np.sin(np.pi * t)

            T_i = np.stack([
                np.concatenate([xi_1, xi_2, xi_3]),
                np.concatenate([yi_1, yi_2, yi_3]),
                np.concatenate([zi_1, zi_2, zi_3]),
            ], axis=0)  # (3, n)

            eval_pts = np.repeat(T_i.T[:, np.newaxis, :], 6, axis=1)  # (n, 6, 3)
            q_traj, valid_mask = self.batch_leg_inv_kine(eval_pts)
            leg_valid = bool(np.all(valid_mask[:, i]))
            admiss[i] = leg_valid

            if leg_valid:
                Qq[3 * i : 3 * i + 3, :] = q_traj[:, i, :].T
                Qdot[3 * i : 3 * i + 3, 1:n] = np.abs(
                    np.diff(Qq[3 * i : 3 * i + 3, :], axis=1) / delta_t
                )
            else:
                print(f"Non admissible trajectory for leg_id: {i}")

        return Qq, Qdot, admiss, n, delta_t