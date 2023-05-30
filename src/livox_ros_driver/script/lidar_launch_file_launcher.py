#! /usr/bin/env python
import threading

import roslaunch
import rospkg
import rospy

from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import PointCloud2

class LidarDriver:
    def __init__(self):
        rospy.init_node("lidar_driver", anonymous=False)

        # Get the file path for livox_ros_driver launch files
        self.launch_path = rospkg.RosPack().get_path('livox_ros_driver') + '/launch'
        self.launch_files = ["{}/livox_lidar_nodelet.launch".format(self.launch_path)]

        self.service = rospy.Service("~restart", SetBool, self._on_restart_request)
        self.pipeline = None
        self.create_pipeline(self.launch_files)
        self.running = False
        self.pipeline_up = False

        rospy.on_shutdown(self.shutdown)

        self.lidar_subscriber = rospy.Subscriber('/livox/lidar', PointCloud2, self._on_lidar_msg, queue_size=1)

        # Watchdog variables.
        self.watchdog_hard_kill_timeout = rospy.Duration.from_sec(20)
        self.watchdog_thread = None

    def _on_restart_request(self, request):
        if request.data:
            self.restart()

        response = SetBoolResponse()
        response.success = True
        return response

    def start(self):
        # Can't start the pipeline if the driver doesn't exist.
        if self.pipeline is None:
            rospy.logwarn("start() called when lidar pipeline wasn't configured")
            return

        # Avoid starting the pipeline multiple times.
        if self.running:
            rospy.logwarn("start() called while pipeline was running")
            return

        self.pipeline.start()
        self.running = True

    def stop(self):
        # Avoid stopping the pipeline multiple times.
        if not self.running and self.pipeline is None: return

        # Stop reading from the lidar and close the device.
        self.running = False
        self.pipeline.shutdown()
        self.pipeline = None

    def restart(self):
        # Block multiple restart attempts until the pipeline has a chance to come up.
        if self.pipeline is None:
            rospy.logwarn("restart() called, but pipeline isn't alive")

        # Check if the pipeline is up.
        # TODO: I don't know if there's a way to query the status of the `ROSLaunchParent` class.

        # Recreate the pipeline.
        self.stop()
        self.create_pipeline(self.launch_files)
        self.start()

    def shutdown(self):
        self.stop()
        self.shutdown_signal.set()

    def create_pipeline(self, launch_file_paths):
        if self.pipeline is not None:
            rospy.logerr("Tried to create new lidar pipeline while one already exists.")
            return

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        roslaunch.configure_logging(self.uuid)
        self.pipeline = roslaunch.parent.ROSLaunchParent(self.uuid, launch_file_paths)

    def _run(self):
        # Start the lidar node.
        self.start()

        # Run while we haven't been requested to shutdown.
        rospy.spin()

    def run(self):
        self.shutdown_signal = threading.Event()
        watchdog_thread = threading.Thread(target=self._watchdog, name="watchdog")
        try:
            rospy.loginfo('Watchdog activated...')
            watchdog_thread.start()
            self._run()
        finally:
            rospy.loginfo('Watchdog shutting down...')
            self.shutdown_signal.set()
            watchdog_thread.join()
        return 0

    def _on_lidar_msg(self, msg):
        if len(msg.data) > 0:
            rospy.loginfo_throttle(1, "PointCloud received from lidar.")
            # Tickle the watchdog.
            self.last_progress_time = rospy.get_rostime()

    ## NOTE: this watchdog implementation is based off of: https://gist.github.com/zombig/7f2ccd60755863537763b026ef04f476.
    def _watchdog(self, timeout = 1.0):
        # This variable is initially set here to avoid killing the process at the beginning.
        self.last_progress_time = rospy.get_rostime()
        while True:
            # Wait to see if the watchdog thread should be shutdown.
            # This blocks the watchdog thread to avoid a hot-loop.
            # This timeout should be short to allow the watchdog thread to be responsive.
            if self.shutdown_signal.wait(timeout=timeout):
                return

            # Check if the processing thread watchdog has timed out.
            last_progress_delay = rospy.get_rostime() - self.last_progress_time
            if last_progress_delay < self.watchdog_hard_kill_timeout:
                # Continue waiting to see if the processing will pick back up.
                continue

            # Kill the process since the watchdog timer has elapsed.
            rospy.logerr(
                'no progress in {:0.01f} seconds\n'
                'restarting lidar driver...'.format(last_progress_delay.to_sec())
            )

            # Restart the lidar pipeline.
            if not rospy.is_shutdown():
                self.last_progress_time = rospy.get_rostime()
                self.restart()

if __name__ == "__main__":
    try:
        lidar_driver = LidarDriver()
        lidar_driver.run()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down lidar driver...')
        lidar_driver.shutdown()
        exit(0)

