#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

way_point = [[37.9166364, 114.593419], [37.9165338, 114.5932539], [37.9165528, 114.5934863],[37.9167296, 114.5934568], [37.9157116, 114.5932274]]

for point_a in way_point:
    # 坐标系 b 的原点位置
    origin_b = np.array([37.9168368, 114.5942735])

    # 坐标系 b 的 x 轴上的一个点
    x_axis_b = np.array([37.9168128, 114.593935])

    # 坐标系 b 的 y 轴上的一个点
    y_axis_b = np.array([37.9165672, 114.5943033])

    # 计算坐标系 b 的 x 轴和 y 轴的单位向量
    unit_x_b = (x_axis_b - origin_b) / np.linalg.norm(x_axis_b - origin_b)
    unit_y_b = (y_axis_b - origin_b) / np.linalg.norm(y_axis_b - origin_b)

    # 计算点 A 在坐标系 b 中的坐标
    vector_a = point_a - origin_b
    x_coord_b = np.dot(vector_a, unit_x_b)
    y_coord_b = np.dot(vector_a, unit_y_b)

    print(f"点 A 在坐标系 b 中的 x 坐标：{x_coord_b}")
    print(f"点 A 在坐标系 b 中的 y 坐标：{y_coord_b}")



