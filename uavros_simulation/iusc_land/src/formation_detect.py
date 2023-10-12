#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def image_callback(msg):
    try:
        bridge = CvBridge()
        # 将ROS图像消息转换为OpenCV图像
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 在这里添加你的图像处理代码，例如显示图像
        board_colors = []  # 用于存储棋盘上物体颜色的4x4列表

        # 定义红色和绿色物体的颜色范围
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        lower_green = np.array([50, 100, 100])
        upper_green = np.array([70, 255, 255])

        # 将大图转换为HSV颜色空间
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 创建红色和绿色物体的掩码
        mask_red_board = cv2.inRange(image_hsv, lower_red, upper_red)
        mask_green_board = cv2.inRange(image_hsv, lower_green, upper_green)

        # 在掩码中找到红色和绿色物体的轮廓
        _, contours_red_board, _ = cv2.findContours(
            mask_red_board, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        _, contours_green_board, _ = cv2.findContours(
            mask_green_board, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 合并所有红色和绿色物体的轮廓为一个列表
        all_contours = []
        all_contours.extend(contours_red_board)
        all_contours.extend(contours_green_board)

        # 创建一个掩码，其中包含所有轮廓区域
        mask_all_contours = np.zeros_like(mask_red_board)

        # 确定棋盘的位置（假设棋盘是最大的轮廓）
        x, y, w, h = cv2.boundingRect(
            cv2.drawContours(
                mask_all_contours, all_contours, -1, 255, thickness=cv2.FILLED
            )
        )
        chessboard_region = image[y : y + h, x : x + w]

        # 分析棋盘区域的颜色
        chessboard_hsv = cv2.cvtColor(chessboard_region, cv2.COLOR_BGR2HSV)
        mask_red_chessboard = cv2.inRange(chessboard_hsv, lower_red, upper_red)
        mask_green_chessboard = cv2.inRange(chessboard_hsv, lower_green, upper_green)

        # 划分棋盘区域为4x4的格子
        cell_height = h // 4
        cell_width = w // 4

        # 遍历4x4的每个格子，确定颜色
        for i in range(4):
            row_colors = []
            for j in range(4):
                x_start = j * cell_width
                x_end = (j + 1) * cell_width
                y_start = i * cell_height
                y_end = (i + 1) * cell_height

                # 计算当前格子中红色和绿色像素数
                red_pixels = np.sum(
                    mask_red_chessboard[y_start:y_end, x_start:x_end] > 0
                )
                green_pixels = np.sum(
                    mask_green_chessboard[y_start:y_end, x_start:x_end] > 0
                )

                # 根据像素数确定颜色
                if red_pixels > green_pixels:
                    row_colors.append("Red")
                else:
                    row_colors.append("Green")

            board_colors.append(row_colors)

        # 保存提取的棋盘图像
        # cv2.imwrite(output_filepath, chessboard_region)

        for row in board_colors:
            print(row)

        print("\n")

    except Exception as e:
        rospy.logerr(e)


def main():
    rospy.init_node("image_subscriber", anonymous=True)
    rospy.Subscriber("image_topic", Image, image_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
