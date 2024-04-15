# -*- coding: utf-8 -*-
"""
@Time ： 2022/11/12 11:31
@Auth ： 邓浩然
@File ：depth_calibration.py
@IDE ：PyCharm
@Description：负责对深度相机进行校准的程序
"""
import cv2
import torch
import numpy
import numpy as np
import opencv_camera

def save_txt():
    kkkl = cv2.imread("depth.png", -1)
    kkkl = np.array(kkkl)
    k = []
    for i in range(480):
        for j in range(640):
            b = []
            b.append(i)
            b.append(j)
            b.append(kkkl[i, j])
            k.append(b)
    k = np.array(k)
    # print(k)
    np.savetxt("target_point_cloud.txt", k)


def linear_regression(x, y):
    N = len(x)
    sumx = sum(x)
    sumy = sum(y)
    sumx2 = sum(x ** 2)
    sumxy = sum(x * y)

    A = np.mat([[N, sumx], [sumx, sumx2]])
    b = np.array([sumy, sumxy])

    return np.linalg.solve(A, b)


def adjusting():
    save_txt()
    filename = "color.png"
    # model_type = "DPT_Large"  # MiDaS v3 - Large     (highest accuracy, slowest inference speed)
    model_type = "DPT_Hybrid"   # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)

    # model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

    midas = torch.hub.load('C:/Users/deng/.cache/torch/hub/intel-isl_MiDaS_master', model_type, source='local')

    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    midas.to(device)
    midas.eval()

    midas_transforms = torch.hub.load('C:/Users/deng/.cache/torch/hub/intel-isl_MiDaS_master', "transforms",
                                      source='local')

    if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
        transform = midas_transforms.dpt_transform
    else:
        transform = midas_transforms.small_transform

    img = cv2.imread(filename)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    input_batch = transform(img).to(device)

    with torch.no_grad():
        prediction = midas(input_batch)

        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()
    prediction = prediction
    output = prediction.cpu().numpy()
    print(output.shape)

    f = open("target_point_cloud.txt", "r")
    file = f.readlines()
    data = []
    print()
    for i in file:
        i = i.strip("\n").split(" ")
        datalist = []
        for a in i:
            datalist.append(float(a))
        datalist.append(output[int(float(i[0]))][int(float(i[1]))])
        data.append(datalist)
    # print(data)
    X = []
    Y = []
    for aaa in data:
        X.append(aaa[3])
        Y.append(aaa[2])
    X = numpy.array(X)
    Y = numpy.array(Y)

    # X = numpy.array([8.19,2.72,6.39,8.71,4.7,2.66,3.78])
    # Y = numpy.array([7.01,2.78,6.4,6.71,4.1,4.23,4.05])
    print(X)
    print(Y)
    # def residuals(p,x):
    #     k,b = p
    #     #return Y-(k*X+b)
    #     return k*x+b
    #
    # def error(p,x,y):
    #     return residuals(p, x)- y
    # r = optimize.leastsq(error, [1, 1],args=(X,Y))
    # k, b = r[0]
    # print("k = ", k ,"b = ", b)

    a0, a1 = linear_regression(X, Y)
    print("k = ", a1, "b = ", a0)
    return a1, a0
# XX = numpy.arange(1,10,0.1)
# YY = k*XX+b
# plt.plot(XX,YY)
# plt.plot(X,Y,"o")
# plt.xlabel("X")
# plt.ylabel("Y")
# output = output.astype(numpy.uint16)
# cv2.imwrite("./new/"+ssum+"_1.png", output)
# cv2.imwrite("./new/"+ssum+"_2.png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
# convertPNG("./new/"+ssum+"_1.png", ssum)#C:/Users/BAMBOO/Desktop/source pics/rgbd_6/color


if __name__ == '__main__':
    ca = opencv_camera.Camera()
    ca.run()
    adjusting()
