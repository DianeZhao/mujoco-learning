import mujoco
import mujoco.viewer

class CustomViewer:
    def __init__(self, model_path, distance=3, azimuth=0, elevation=-30):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.handle = mujoco.viewer.launch_passive(self.model, self.data)
        self.handle.cam.distance = distance
        self.handle.cam.azimuth = azimuth
        self.handle.cam.elevation = elevation

    def is_running(self):
        return self.handle.is_running()

    def sync(self):
        self.handle.sync()

    @property
    def cam(self):
        return self.handle.cam

    @property
    def viewport(self):
        return self.handle.viewport

    def run_loop(self):
        self.runBefore()
        while self.is_running():
            mujoco.mj_forward(self.model, self.data)
            # When you manually change state (like setting data.qpos[:] = ...) and want the system to update derived values.
            # Before reading sensor outputs or torques without stepping. i.e. data.blahblah
            # For visualizing static poses or doing inverse kinematics.
            self.runFunc()
            mujoco.mj_step(self.model, self.data)
            #When you want to simulate physics forward in time.
            #This is what you use in your simulation loop for real-time or batch simulation.
            self.sync()
    
    def runBefore(self):
        pass

    def runFunc(self):
        pass