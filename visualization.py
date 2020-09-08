import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':

    # ------读取传感器数据---------------------
    measure_file = 'data/measure.txt'
    lines = [line for line in open(measure_file, 'r')]
    camera_data = []
    lidar_data = []

    for line in lines:
        content = line.split(' ')
        if float(content[0]) == 1:
            camera_data.append([float(content[1]), float(content[2])])
        elif float(content[0]) == 2:
            lidar_data.append([float(content[1]), float(content[2])])

    # ---------读取label-----------------
    gt_file = 'data/rm_gt.txt'
    lines = [line for line in open(gt_file, 'r')]
    gt = []
    for line in lines:
        content = line.split(' ')
        gt.append([float(content[0]), float(content[1])])

    # ---------读取融合结果-----------------
    result_file = 'bin/fusion_result.txt'
    lines = [line.rstrip('\n') for line in open(result_file, 'r')]
    result_data = []
    for i in range(len(lines)):
            line=lines[i]
            content = line.split(' ')
            result_data.append([float(content[1]), float(content[3])])

    # ---------可视化---------------------
    plt.figure()
    camera_data = np.array(camera_data)
    lidar_data = np.array(lidar_data)
    gt = np.array(gt)
    result_data = np.array(result_data)

    plt.scatter(camera_data[:, 0], camera_data[:, 1],c='b' , s=3, label='camera_data')
    plt.scatter(lidar_data[:, 0], lidar_data[:, 1],c='m' , s=3, label='lidar_data')
    plt.plot(result_data[:, 0], result_data[:, 1], 'g-', lw=2, label='result')
    plt.plot(gt[:, 0], gt[:, 1], 'r-', lw=2, label='gt')

    plt.legend(loc='upper right')
    plt.show()




