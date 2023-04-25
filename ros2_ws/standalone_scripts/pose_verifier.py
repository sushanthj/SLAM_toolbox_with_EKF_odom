import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from typing import List
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R
import numpy as np
import argparse
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms


def confidence_ellipse(x, y, ax, n_std=3.0, facecolor='none', **kwargs):
    """
    Create a plot of the covariance confidence ellipse of *x* and *y*.

    Parameters
    ----------
    x, y : array-like, shape (n, )
        Input data.

    ax : matplotlib.axes.Axes
        The axes object to draw the ellipse into.

    n_std : float
        The number of standard deviations to determine the ellipse's radiuses.

    **kwargs
        Forwarded to `~matplotlib.patches.Ellipse`

    Returns
    -------
    matplotlib.patches.Ellipse
    """
    if x.size != y.size:
        raise ValueError("x and y must be the same size")

    cov = np.cov(x, y)
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensional dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                      facecolor=facecolor, **kwargs)

    # Calculating the standard deviation of x from
    # the squareroot of the variance and multiplying
    # with the given number of standard deviations.
    scale_x = np.sqrt(cov[0, 0]) * n_std
    mean_x = np.mean(x)

    # calculating the standard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std
    mean_y = np.mean(y)

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)



@dataclass
class Pose:
    x: float
    y: float
    yaw: float

    @classmethod
    def translation_error(cls):
        return 0.1

    @classmethod
    def yaw_error(cls):
        return 10

    def measure_error(self, poses: List['Pose']):
        xs = [pose.x for pose in poses]
        ys = [pose.y for pose in poses]
        yaws = [pose.yaw for pose in poses]

        max_x = np.max(xs)
        max_y = np.max(ys)
        max_yaw = np.max(yaws)

        max_deviation_x = abs(max_x - self.x)
        max_deviation_y = abs(max_y - self.y)
        max_deviation_yaw = abs(max_yaw - self.yaw)

        print("Error Analysis:")

        print(f"Max to GT deviation x: {max_deviation_x * 100} centimeters.")
        print(f"Max to GT deviation y: {max_deviation_y * 100} centimeters.")
        print(f"Max to GT deviation yaw: {max_deviation_yaw} degrees.")

        min_x = np.min(xs)
        min_y = np.min(ys)
        min_yaw = np.min(yaws)

        min_deviation_x = abs(min_x - self.x)
        min_deviation_y = abs(min_y - self.y)
        min_deviation_yaw = abs(min_yaw - self.yaw)

        print(f"Min to GT deviation x: {min_deviation_x * 100} centimeters.")
        print(f"Min to GT deviation y: {min_deviation_y * 100} centimeters.")
        print(f"Min to GT deviation yaw: {min_deviation_yaw} degrees.")

        fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, figsize=(7.5, 5), sharey=False, gridspec_kw={'width_ratios': [2, 1]})
        one_sigma = confidence_ellipse(np.array(xs), np.array(ys), ax1, n_std=1, edgecolor='blue')
        two_sigma = confidence_ellipse(np.array(xs), np.array(ys), ax1, n_std=2, edgecolor='cyan')
        three_sigma = confidence_ellipse(np.array(xs), np.array(ys), ax1, n_std=3, edgecolor='purple')
        # ax1.scatter(xs, ys)
        circle = plt.Circle((self.x, self.y), self.translation_error(), color='r', alpha=0.2)
        patch = ax1.add_patch(circle)
        us = np.cos(yaws)
        vs = np.sin(yaws)
        quiver = ax1.scatter(np.array(xs), np.array(ys))
        patch.set_label("Allowable error region.")
        one_sigma.set_label("One standard deviation confidence interval")
        two_sigma.set_label("Two standard deviation confidence interval")
        three_sigma.set_label("Three standard deviation confidence interval")
        quiver.set_label("Sampled poses.")
        ax1.set_title('Position profile')
        ax1.set_xlabel("x (meters)")
        ax1.set_ylabel("y (meters)")
        ax1.legend()

        # arange = np.arange(0.8, 1.2, 0.01)
        # vio = ax1.violinplot(xs)
        # line = ax1.fill_between(arange, self.x - self.translation_error(), self.x + self.translation_error(),
        #                         color='C1', alpha=0.2)
        # ax1.set_title("Translation profile in the x axis (meters)")

        # vio['bodies'][0].set_label("Deviations")
        # line.set_label("Allowable deviation")
        # ax1.legend()

        # vio = ax2.violinplot(ys)
        # line = ax2.fill_between(arange, self.y - self.translation_error(), self.y + self.translation_error(),
        #                         color='C1', alpha=0.2)
        # ax2.set_title("Translation profile in the y axis (meters)")
        # vio['bodies'][0].set_label("Deviations")
        # line.set_label("Allowable deviation")
        # ax2.legend()

        arange = np.arange(0.7, 1.3, 0.05)
        vio = ax2.violinplot(yaws)
        line = ax2.fill_between(arange, self.yaw - self.yaw_error(), self.yaw + self.yaw_error(), color='red', alpha=0.2)
        ax2.set_title("Yaw profile")
        vio['bodies'][0].set_label("Deviations")
        line.set_label("Allowable deviation")
        ax2.legend()
        ax2.set_xlabel("Density (centered at one)")
        ax2.set_ylabel("Yaw (degrees)")
        plt.show()


class PoseVerifyNode(Node):
    def __init__(self, observation_id: int = 0, num_observations: int = 10):
        super().__init__('pose_verify_node')

        self._observation_id = observation_id
        self._num_observations = num_observations

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            500)

        self._logger = self.get_logger()

        self.id2pose_dict = {
            '0': Pose(x=9.5, y=18.4, yaw=0),
            '1': Pose(x=9.3, y=18.9, yaw=22.5),
            '2': Pose(x=9.0, y=19.3, yaw=90),
            '3': Pose(x=9.16, y=25.7, yaw=90),
            '4': Pose(x=9.16, y=28.25, yaw=90),
            '5': Pose(x=9.47, y=34.71, yaw=90),
            '6': Pose(x=8.97, y=42.4, yaw=180)
        }

        self.publisher = self.create_publisher(PoseStamped, 'pose_filtered', 500)
        self.filtered_pose = None
        self.logger = self.get_logger()
        self._observations: List[Pose] = []
        self._gt_pose = self.id2pose_dict[str(self._observation_id)]

    @property
    def current_observations(self):
        return len(self._observations)

    def finished_observing(self):
        return self.current_observations >= self._num_observations

    def pose_callback(self, msg):
        quat = msg.pose.pose.orientation
        rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        yaw = rotation.as_euler('zyx', degrees=True)[0]

        print(f"Collecting observation {len(self._observations)}/{self._num_observations}")
        observed_pose = Pose(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y, yaw=yaw)
        self._observations.append(observed_pose)

        if self.finished_observing():
            self._gt_pose.measure_error(self._observations)


def main(args=None):
    rclpy.init()
    pose_smoothing_node = PoseVerifyNode(observation_id=args.id, num_observations=10)
    while not pose_smoothing_node.finished_observing():
        rclpy.spin_once(pose_smoothing_node)
    pose_smoothing_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', type=int, default=0)
    args = parser.parse_args()
    main(args)