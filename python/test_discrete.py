'''
@ author: SonSang (Sanghyun Son)
@ email: shh1295@gmail.com

This script solves inverse problem for single lane environment.
'''
# from matplotlib import pyplot as plt
# from singlelane_env import SingleLaneEnv
# from arz.arz_lane import ARZLane
import torch
# import env_presets
import os

n_epoch = 100
n = 100
dt = 1
seg_l = 5
seg_r = 15
target_density = 0.1


def f_naive():
    rho = torch.tensor([0.2] * n, dtype=torch.float32, requires_grad=True)
    capacitor = torch.tensor([0.0], dtype=torch.float32)

    vehicles = []
    density = []
    optimizer = torch.optim.Adam([rho], lr=0.01)
    for epoch in range(n_epoch):
        for i in range(n):
            last_capcitor = capacitor.int()
            capacitor += rho[i] * dt
            # print(capacitor.int(), last_capcitor)
            if capacitor.int() > last_capcitor:
                vehicles.append([1, 0])

            for v in vehicles:
                v[1] += v[0] * dt

            count = torch.tensor([0], dtype=torch.float32)
            for v in vehicles:
                if v[1] > seg_l and v[1] < seg_r:
                    count += 1

            density.append(count / (seg_r - seg_l))

        density = torch.cat(density).max()
        loss = (density - target_density)**2


        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        print("epoch: {:04d}, loss: {:.5f}, density: {:.5f}".format(epoch, loss, density))


f_naive()



