import torch
import os
import matplotlib.pyplot as plt
import numpy as np



class CapacitorLayer(torch.autograd.Function):
    @staticmethod
    def forward(ctx, rho, last_capcitor):
        capacitor = rho + last_capcitor
        if capacitor.int() > last_capcitor:
            vehicle = torch.tensor([1.0])
        else:
            vehicle = torch.tensor([0.0])
        ctx.save_for_backward(rho, last_capcitor, vehicle)
        return vehicle, capacitor

    @staticmethod
    def backward(ctx, dldv, dldc):
        rho, last_capcitor, vehicle = ctx.saved_tensors
        if vehicle > 0:
            new_dldc = dldv
        else:
            new_dldc = dldc

        new_dldrho = new_dldc
        return new_dldrho, new_dldc


capacitor_layer = CapacitorLayer.apply

n_epoch = 300
n = 1000
dt = 1
seg_l = 5
seg_r = 15
target_density = 0.1


def f_approximation():
    rho = torch.tensor([1] * n, dtype=torch.float32, requires_grad=True)
    
    rhos = []
    final_densities = []
    
    optimizer = torch.optim.Adam([rho], lr=0.01)
    for epoch in range(n_epoch):
        capacitor = torch.tensor([0.0], dtype=torch.float32)

        vehicles = []
        densities = []

        for i in range(n):
            vehicle, capacitor = capacitor_layer(rho[i] * dt, capacitor)
            if vehicle > 0:
                vehicles.append([1, 0, vehicle])

            for v in vehicles:
                v[1] += v[0] * dt + np.random.normal()

            count = torch.tensor([0], dtype=torch.float32)
            for v in vehicles:
                if v[1] > seg_l and v[1] < seg_r:
                    count += v[2]

            densities.append(count / (seg_r - seg_l))

        density = torch.cat(densities).mean()
        loss = (density - target_density)**2

        rhos.append(rho.mean().detach().numpy())
        final_densities.append(density.detach().numpy())

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        print("epoch: {:04d}, loss: {:.5f}, density: {:.5f}, rho: {:.5f}".format(epoch, loss, density, rho.mean()))

    # plt.clf()
    # plt.plot(rhos)
    # plt.savefig('flux_rho.png')
    # plt.clf()
    # plt.plot(final_densities)
    # plt.savefig('flux_density.png')


f_approximation()



