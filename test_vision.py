from pyniryo import *
import time

robot_ip_address = "10.10.10.10"
workspace_name = "workspace_1"
workspace_size = (0.175, 0.175)
obj_size = 0.029
obs_pose = PoseObject(
    x=0.20, y=0., z=0.35,
    roll=0.0, pitch=1.57, yaw=0.0,
)

h_off = 0.005


# -- MAIN PROGRAM
class RobotDemontrator:

    def __init__(self, robot_ip, observation_pose, height_offset=h_off):

        self.__ip = robot_ip

        self.__robot = NiryoRobot(self.__ip)

        self.__cam_mtx, self.__cam_distortion = self.__robot.get_camera_intrinsics()
        self.__img = None
        self.__obs_pose = observation_pose
        self.__pick_height_off = height_offset

        # self.__fx = self.__cam_mtx[0][0]
        # self.__fy = self.__cam_mtx[1][1]
        # self.__cx = self.__cam_mtx[0][2]
        # self.__cy = self.__cam_mtx[1][2]

        self.__robot.calibrate_auto()
        self.__robot.update_tool()

    def stop(self):
        self.__robot.go_to_sleep()
        self.__robot.close_connection()

    def get_workspace_image(self):
        img_compressed = self.__robot.get_img_compressed()
        self.__img = uncompress_image(img_compressed)
        self.__img = undistort_image(self.__img, self.__cam_mtx, self.__cam_distortion)
        im_work = extract_img_workspace(self.__img, workspace_ratio=1.0)
        # print self.__img.shape, im_work.shape
        return im_work

    def find_object_in_workspace(self, img, color_hsv, display=True):
        img_thresh = threshold_hsv(img, *color_hsv)
        img_thresh = morphological_transformations(img_thresh, MorphoType.OPEN, kernel_shape=(5, 5))
        contour = biggest_contour_finder(img_thresh)
        img_thresh_rgb = cv2.cvtColor(img_thresh, cv2.COLOR_GRAY2BGR)

        if len(contour):
            cx, cy = get_contour_barycenter(contour)
            cx_rel, cy_rel = relative_pos_from_pixels(img, cx, cy)

            angle = get_contour_angle(contour)

            if display:
                img_thresh_rgb = draw_contours(img_thresh_rgb, [contour])
                show_img("Image thresh", img_thresh_rgb, wait_ms=25)
                cv2.waitKey(25)

            return cx_rel, cy_rel, angle
        return None, None, None

    def process(self):
        im_work = None
        self.__robot.move_pose(self.__obs_pose)

        while im_work is None:
            im_work = self.get_workspace_image()
            if im_work is None:
                print("Unable to find markers")

            else:
                cv2.imshow("Workspace", im_work)

            if self.__img is not None:
                cv2.imshow("Last image saw", self.__img)
                cv2.waitKey(25)
        else:
            cx_rel, cy_rel, angle = self.find_object_in_workspace(im_work, ColorHSV.GREEN.value)

            if cx_rel:
                obj_pose = self.__robot.get_target_pose_from_rel(workspace_name,
                                                                 height_offset=self.__pick_height_off,
                                                                 x_rel=cx_rel, y_rel=cy_rel,
                                                                 yaw_rel=angle)
                self.__robot.pick_from_pose(obj_pose)

        place_pose = PoseObject(x=0.2, y=0.2, z=0.1, roll=3.0, pitch=1.57, yaw=2.57, )
        self.__robot.place_from_pose(place_pose)
        self.__robot.move_pose(self.__obs_pose)


# def relative_size_from_pixels(width, height, img):
#     return float(width) / img.shape[1], float(height) / img.shape[0]


if __name__ == '__main__':
    robot = RobotDemontrator(robot_ip_address, obs_pose)
    robot.process()

    cv2.waitKey(0) & 0xFF == ord('q')
    robot.stop()
