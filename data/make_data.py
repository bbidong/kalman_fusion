import numpy as np
import matplotlib.pyplot as plt


def get_rm_data(rm_gt, noise, frequency, valid_rate, flag):
    """
    在真值上加噪声，构造传感器观测数据
    :param rm_gt: 真值, (x,y,time)
    :param noise: 噪声的误差大小
    :param frequency: 传感器观测频率
    :param valid_rate: 传感器观测值的概率丢失
    :param flag: 是哪一种传感器
    :return:
    """
    gt_time=rm_gt[:,-1].reshape(-1,1)
    sensor_time = (np.arange(frequency, max(gt_time), frequency)).reshape(-1, 1)
    data_sensor = np.zeros((len(sensor_time), 3))
    for i in range(0, len(sensor_time)):
        ind = (gt_time.reshape(len(gt_time), ) >= sensor_time[i, 0]).tolist().index(1)
        # 对应的gt点位置
        a=rm_gt[ind]
        gt_point = rm_gt[ind - 1, :2] + \
                   (rm_gt[ind, :2] - rm_gt[ind - 1, :2]) * (sensor_time[i, 0] - gt_time[ind - 1, 0]) / (
                               gt_time[ind, 0] - gt_time[ind - 1, 0])
        # 加噪声, 当作观测点
        noise_x = np.random.normal(gt_point[0], noise)
        noise_y = np.random.normal(gt_point[1], noise)

        data_sensor[i, 1:] = [noise_x, noise_y]
    data_sensor[:, 0] = flag
    data_sensor = np.hstack((data_sensor, sensor_time))
    valid = []
    for i in range(len(data_sensor)):
        valid.append(True) if np.random.random() > valid_rate else valid.append(False)
    valid = np.array(valid)
    data_sensor = data_sensor[valid == True, :]
    return data_sensor

def make_rm_data():

    rm_gt= np.loadtxt('./rm_gt.txt',delimiter=' ')	# 返回类型为array

    camera=get_rm_data(rm_gt,0.2,0.3,0.1,1)
    lidar = get_rm_data(rm_gt,0.5, 0.16, 0.4, 2)

    total_data=np.vstack((camera,lidar))
    total_data = total_data[total_data[:, -1].argsort(), :]  # 按时间排序
    np.savetxt('measure.txt', total_data, delimiter=' ', newline='\n')
    print('ok')

if __name__ == '__main__':

    make_rm_data()
