import arz
import pylab
import matplotlib
import numpy

def riemann_bounds(speeds, q_l, q_r):
    xlow = min(-1, speeds[0]*1)
    xhigh = max(1, speeds[1]*1)
    s0 = slope(speeds[0], (xlow, xhigh), (0, 1))
    s1 = slope(speeds[1], (xlow, xhigh), (0, 1))

    i0 = speeds[0]*q_l.rho
    i1 = speeds[1]*q_r.rho

    return xlow, xhigh, i0, i1, s0, s1

def slope(speed, xr, yr):
    box0 = [[speed*yr[0], speed*yr[1]], [yr[0], yr[1]]]
    if speed == 0:
        return box0
    else:
        return [[xr[0], xr[1]], [xr[0]/speed, xr[1]/speed]]

def riemann_plot(ax, q_l, q_r, u_max, gamma):
    if q_l.rho < 1e-6:
        speeds = [0, q_l.u]
    elif q_r.rho < 1e-6:
        q_m = arz.full_q(0.0, q_l.u + (u_max - q_l.u_eq),
                         u_max, gamma)

        lambda_0_l = q_l.lambda0(u_max, gamma)
        lambda_0_m = q_l.lambda1()

        speeds = [(lambda_0_l + lambda_0_m)/2,
                  (lambda_0_l + lambda_0_m)/2]
    elif abs(q_l.u - q_r.u) < 1e-6:
        speeds = [0.0, q_r.u]
        ax.add_line(matplotlib.lines.Line2D((speeds[0]*q_l.rho, speeds[1]*q_l.rho),(q_l.rho, q_l.rho)))
    elif q_l.u > q_r.u:
        q_m = arz.rho_m(q_l, q_r, 1.0/u_max, 1.0/gamma)

        flux_0_diff = q_m.rho*q_m.u - q_l.rho*q_l.u
        if abs(flux_0_diff) < 1e-6:
            speeds = [0.0,
                      q_r.u]
        else:
            speeds = [flux_0_diff/(q_m.rho - q_l.rho),
                      q_r.u]
        ax.add_line(matplotlib.lines.Line2D((speeds[0]*q_m.rho, speeds[1]*q_m.rho),(q_m.rho, q_m.rho)))
    elif u_max + q_l.u - q_l.u_eq > q_r.u:
        q_m = arz.rho_m(q_l, q_r, 1.0/u_max, 1.0/gamma)

        lambda0_l = q_l.lambda0(u_max, gamma)
        lambda0_m = q_m.lambda0(u_max, gamma)

        speeds = [lambda0_l,
                  q_r.u]

        if(lambda0_l < 0 and lambda0_m > 0):
            x = numpy.linspace(lambda0_l, lambda0_m)
            y = [q_l.centered_rarefaction_rho(eta, u_max, gamma) for eta in x]
            x = numpy.linspace(lambda0_l*q_l.rho, lambda0_m*q_m.rho)
            ax.plot(x, y)
        else:
            ax.add_line(matplotlib.lines.Line2D((speeds[0]*q_m.rho, speeds[1]*q_m.rho),(q_m.rho, q_m.rho)))

    else:
        q_m = arz.full_q(0, u_max + q_l.u - q_l.u_eq,
                         u_max, gamma)

        lambda0_l = q_l.lambda_0(u_max, gamma);
        lambda0_m = q_m.lambda_1();

        speeds = [(lambda0_l + lambda0_m)/2,
                  q_r.u]

    xlow, xhigh, i0, i1, s0, s1 = riemann_bounds(speeds, q_l, q_r)

    ax.add_line(matplotlib.lines.Line2D(s0[0], s0[1]))
    ax.add_line(matplotlib.lines.Line2D(s1[0], s1[1]))

    ax.plot([xlow, i0], [q_l.rho, q_l.rho])
    ax.plot([i1,  xhigh], [q_r.rho, q_r.rho])
    ax.axis([xlow, xhigh, 0, 1])

    return ax

if __name__ == '__main__':
    u_max = 20.0
    gamma = 0.5
    pylab.clf()
    riemann_plot(pylab.axes(),
                 arz.full_q(0.8, 1.0, u_max, gamma),
                 arz.full_q(0.4, 1.0, u_max, gamma),
                 u_max,
                 gamma)
    pylab.show()
