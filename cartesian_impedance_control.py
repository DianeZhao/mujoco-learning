import mujoco
import numpy as np
import mujoco_viewer
from scipy.spatial.transform import Rotation as R

class CartesianImpedanceController(mujoco_viewer.CustomViewer):
    def __init__(self, path):
        super().__init__(path, 1.5, azimuth=45, elevation=-30)
        self.ee_site_name = "ee_site"  # name of end-effector site in XML

    def runBefore(self):
        # Stiffness and damping in Cartesian space
        # self.Kp = np.diag([200, 200, 200, 50, 50, 50])  # pos (N/m) + rot (Nm/rad)
        # self.Kd = np.diag([30, 30, 30, 5, 5, 5])        # damping
        self.Kp = np.diag([150, 150, 150, 0, 0, 0])  # pos (N/m) + rot (Nm/rad)
        self.Kd = np.diag([0, 0, 0, 0, 0, 0])        # damping

        # Get EE site ID
        self.ee_site_id = self.model.site(self.ee_site_name).id

        # Desired EE pose
        self.xd = self.get_ee_pose()
        for i in range(self.model.nbody):
            print(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i))


        #self.target_mocap_id = self.model.body("target_mocap").id

        print("init_ee_pos", self.xd)
        #-1.27801257e-07 -4.07770729e-01  1.16398639e-01  1.98409371e-05 -1.57080265e+00 -8.30121355e-06]
        # Desired target in world coordinates
        # target_pos = np.array([0.4, 0.0, 0.3])  # [x, y, z]
        # target_rotvec = R.from_euler('xyz', [0, 0, 0]).as_rotvec()  # [rx, ry, rz]

        #self.xd = np.concatenate([target_pos, target_rotvec])


    def get_ee_pose(self):
        pos = self.data.site_xpos[self.ee_site_id].copy()
        rotmat = self.data.site_xmat[self.ee_site_id].reshape(3, 3)
        rotvec = R.from_matrix(rotmat).as_rotvec()
        
        return np.concatenate([pos, rotvec])  # [x, y, z, rx, ry, rz]

    def runFunc(self):
        # Get current EE pose and velocity
        x = self.get_ee_pose()
        v = self.get_ee_velocity()

        # Cartesian error
        pos_err = self.xd[:3] - x[:3]
        rot_err = self.xd[3:] - x[3:]  # approximate for small angles

        F = self.Kp @ np.concatenate([pos_err, rot_err]) - self.Kd @ v

        # Get Jacobian
        J_pos = np.zeros((3, self.model.nv))
        J_rot = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, J_pos, J_rot, self.ee_site_id)

        J_full = np.vstack([J_pos, J_rot])
        tau = J_full.T @ F

        # Visualization the target
        # Set mocap position
        # self.data.mocap_pos[self.target_mocap_id] = self.xd[:3]
        # # Convert rotvec to quaternion
        # quat = R.from_rotvec(self.xd[3:]).as_quat()  # [x, y, z, w]
        # self.data.mocap_quat[self.target_mocap_id] = np.array([quat[3], quat[0], quat[1], quat[2]])  # MuJoCo uses [w, x, y, z]
        
        
        # Apply torque control
        self.data.ctrl[:] = tau[:self.model.nu]

    def get_ee_velocity(self):
        # EE velocity in Cartesian space (3 lin + 3 ang)
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(
            self.model, self.data,
            mujoco.mjtObj.mjOBJ_SITE, self.ee_site_id,
            vel, 0
        )
        return vel

if __name__ == "__main__":
    test = CartesianImpedanceController("./model/trs_so_arm100/scene_torque.xml")
    test.run_loop()
