from math import sqrt, pi, e
import matplotlib.pyplot as plt
import numpy as np


def plot_grasp():
    def fail(x):
        if x == 1:
            return 1
        else:
            return 0

    def success(x):
        if x == 3:
            return 1
        else:
            return 0

    xs = [i*.01 for i in range(401)]
    pf = []
    ps = []
    p = []
    alpha = 0.2
    for i in [x*.01 for x in range(401)]:
        pf.append(fail(i))
        ps.append(success(i))
        p.append(alpha*fail(i) + (1 - alpha)*success(i))

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pf)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'gripper.x_0', '', 'gripper.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_x_fail')
    plt.savefig('grasp0.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, ps)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'gripper.x_0', '', 'gripper.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_x_grasp')
    plt.savefig('grasp1.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, p)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'gripper.x_0', '', 'gripper.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_x')
    plt.savefig('grasp.png')


def plot_place():
    def fail(x):
        if x == 1:
            return 1
        else:
            return 0

    def success(x):
        if x == 3:
            return 1
        else:
            return 0

    def fail_ix(x):
        if x == 1:
            return 1
        else:
            return 0

    def success_ix(x, sigma):
        return 1.0/sqrt(2*pi*pow(sigma, 2))*pow(e, -0.5*pow(x - 3, 2)/pow(sigma, 2))

    def fail_iz(z):
        if z == 1:
            return 1
        else:
            return 0

    def success_iz(z):
        if z == 3:
            return 1
        else:
            return 0

    def fall_iz(z, lam):
        if z > 3:
            return 0
        lam *= -1
        return 1.0/(1 - pow(e, -lam * 3)) * lam * pow(e, -lam*z)

    xs = [i*.01 for i in range(401)]
    pgf = []
    pgs = []
    pg = []
    pixf = []
    pixs = []
    pix = []
    pizf = []
    pizs = []
    pizfall = []
    piz = []
    alpha = 0.3
    sigma = 0.25
    beta = 0.2
    lam = .5
    for i in [x*.01 for x in range(401)]:
        pgf.append(fail(i))
        pgs.append(success(i))
        pg.append(alpha*fail(i) + (1 - alpha)*success(i))

        pixf.append(fail_ix(i))
        pixs.append(success_ix(i, sigma))
        pix.append(alpha*fail_ix(i) + (1 - alpha)*success_ix(i, sigma))

        pizf.append(fail_iz(i))
        pizs.append(success_iz(i))
        pizfall.append(fall_iz(i, lam))
        piz.append(alpha*fail_iz(i) + beta*fall_iz(i, lam) + (1 - (alpha + beta))*success_iz(i))

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pgf)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'gripper.x_0', '', 'gripper.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_gx_fail')
    plt.savefig('place_gx0.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pgs)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'gripper.x_0', '', 'gripper.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_gx_place')
    plt.savefig('place_gx1.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pg)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'gripper.x_0', '', 'gripper.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_gx')
    plt.savefig('place_gx.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pixf)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'item.x_0', '', 'item.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_ix_fail')
    plt.savefig('place_ix0.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pixs)
    plt.xlim((0, 4))
    plt.ylim(ymin=0)
    plt.xticks(np.arange(5), ('', 'item.x_0', '', 'item.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_ix_place')
    plt.savefig('place_ix1.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pix)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'item.x_0', '', 'item.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_ix')
    plt.savefig('place_ix.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pizf)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'item.z_0', '', 'item.z_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_iz_fail')
    plt.savefig('place_iz0.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pizs)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'item.z_0', '', 'item.z_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_iz_place')
    plt.savefig('place_iz1.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pizfall)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'item.z_0', '', 'item.z_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_iz_fall')
    plt.savefig('place_iz2.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, piz)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'item.z_0', '', 'item.z_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_iz')
    plt.savefig('place_iz.png')


def plot_move():
    def short(x, lam):
        if x < 1:
            return 0
        if x > 3:
            return 0
        else:
            return 1.0/(1 - pow(e, -lam * 3)) * lam * pow(e, -lam*x)

    def success(x):
        if x == 3:
            return 1
        else:
            return 0

    def fail_ix(x):
        return 1.0/sqrt(2*pi*pow(sigma, 2))*pow(e, -0.5*pow(x - 1, 2)/pow(sigma, 2))

    def success_ix(x, sigma):
        return 1.0/sqrt(2*pi*pow(sigma, 2))*pow(e, -0.5*pow(x - 3, 2)/pow(sigma, 2))

    def short_ix(x, lam):
        if x < 1:
            return 0
        if x > 3:
            return 0
        else:
            return 1.0/(1 - pow(e, -lam * 3)) * lam * pow(e, -lam*x)

    def other_ix(x):
        return .2

    def stay_iz(z):
        if z == 2:
            return 1
        else:
            return 0

    def fall_iz(z, lam):
        if z > 2:
            return 0
        lam *= -1
        return 1.0/(1 - pow(e, -lam * 3)) * lam * pow(e, -lam*z)

    xs = [i*.01 for i in range(401)]
    pgshort = []
    pgs = []
    pg = []
    pixf = []
    pixs = []
    pixshort = []
    pixrand = []
    pix = []
    pizs = []
    pizfall = []
    piz = []
    w_short = 0.5
    w_short_i = 0.3
    w_nudge = 0.4
    w_push = 0.2
    w_stay = 0.8
    sigma = 0.3
    sigma2 = 0.35
    lam_short = .25
    lam = .5
    for i in [x*.01 for x in range(401)]:
        pgshort.append(short(i, lam_short))
        pgs.append(success(i))
        pg.append(w_short*short(i, lam_short) + (1 - w_short)*success(i))

        pixf.append(fail_ix(i))
        pixs.append(success_ix(i, sigma2))
        pixshort.append(short_ix(i, lam_short))
        pixrand.append(other_ix(i))
        pix.append(w_nudge*fail_ix(i) + w_short_i*short_ix(i, sigma) + w_push*success_ix(i, sigma2) + (1 - (w_nudge + w_short_i + w_push))*other_ix(i))

        pizs.append(stay_iz(i))
        pizfall.append(fall_iz(i, lam))
        piz.append(w_stay*stay_iz(i) + (1 - w_stay)*fall_iz(i, lam))

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pgshort)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'gripper.x_0', '', 'gripper.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_gx_short')
    plt.savefig('move_gx0.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pgs)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'gripper.x_0', '', 'gripper.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_gx_move')
    plt.savefig('move_gx1.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pg)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'gripper.x_0', '', 'gripper.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_gx')
    plt.savefig('move_gx.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pixf)
    plt.xlim((0, 4))
    plt.ylim(ymin=0)
    plt.xticks(np.arange(5), ('', 'item.x_0', '', 'item.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_ix_nudge')
    plt.savefig('move_ix0.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pixs)
    plt.xlim((0, 4))
    plt.ylim(ymin=0)
    plt.xticks(np.arange(5), ('', 'item.x_0', '', 'item.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_ix_push')
    plt.savefig('move_ix1.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pixshort)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'item.x_0', '', 'item.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_ix_short')
    plt.savefig('move_ix2.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pixrand)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'item.x_0', '', 'item.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_ix_rand')
    plt.savefig('move_ix3.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pix)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', 'item.x_0', '', 'item.x_target', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_ix')
    plt.savefig('move_ix.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pizs)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', '', 'item.z_0', '', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_iz_stay')
    plt.savefig('move_iz0.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, pizfall)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', '', 'item.z_0', '', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_iz_fall')
    plt.savefig('move_iz1.png')

    plt.figure(figsize=(5, 3))
    plt.plot(xs, piz)
    plt.xlim((0, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(5), ('', '', 'item.z_0', '', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_iz')
    plt.savefig('move_iz.png')


def plot_move_constrained():
    def short(x, lam, c=False):
        if x < 1:
            return 0
        if not c:
            if x > 3:
                return 0
            else:
                return 1.0/(1 - pow(e, -lam * 3)) * lam * pow(e, -lam*x)
        else:
            if x > 2.5:
                return 0
            else:
                return 1.0/(1 - pow(e, -lam * 2.5)) * lam * pow(e, -lam*x)

    def success(x, c=False):
        if not c and x == 3:
            return 1
        elif c and x == 2.5:
            return 1
        else:
            return 0

    def fail_ix(x, c=False):
        if not c:
            return 1.0/sqrt(2*pi*pow(sigma, 2))*pow(e, -0.5*pow(x - 1, 2)/pow(sigma, 2))
        else:
            if x > 2.5 or x < -0.5:
                return 0
            else:
                return 1.0/sqrt(2*pi*pow(sigma, 2))*pow(e, -0.5*pow(x - 1, 2)/pow(sigma, 2))

    def success_ix(x, sigma, c=False):
        if not c:
            return 1.0/sqrt(2*pi*pow(sigma, 2))*pow(e, -0.5*pow(x - 3, 2)/pow(sigma, 2))
        else:
            if x > 2.5 or x < -0.5:
                return 0
            else:
                return 1.0/sqrt(2*pi*pow(sigma, 2))*pow(e, -0.5*pow(x - 2.5, 2)/pow(sigma, 2))

    def short_ix(x, lam, c=False):
        if x < 1:
            return 0
        if not c:
            if x > 3:
                return 0
            else:
                return 1.0/(1 - pow(e, -lam * 3)) * lam * pow(e, -lam*x)
        else:
            if x > 2.5:
                return 0
            else:
                return 1.0/(1 - pow(e, -lam * 2.5)) * lam * pow(e, -lam*x)

    def other_ix(x, c=False):
        if not c:
            return .2
        else:
            if x > 2.5 or x < -0.5:
                return 0
            else:
                return .2

    xs = [i*.01 for i in range(-100, 401)]
    pgshort = []
    pgs = []
    pg = []
    pgc = []
    pixf = []
    pixs = []
    pixshort = []
    pixrand = []
    pix = []
    pixc = []
    w_short = 0.5
    w_short_i = 0.3
    w_nudge = 0.4
    w_push = 0.2
    sigma = 0.3
    sigma2 = 0.35
    lam_short = .25
    for i in [x*.01 for x in range(-100, 401)]:
        pg.append(w_short*short(i, lam_short) + (1 - w_short)*success(i))
        pgc.append(w_short*short(i, lam_short, c=True) + (1 - w_short)*success(i, c=True))


        pix.append(w_nudge*fail_ix(i) + w_short_i*short_ix(i, sigma) + w_push*success_ix(i, sigma2) + (1 - (w_nudge + w_short_i + w_push))*other_ix(i))
        pixc.append(w_nudge*fail_ix(i, c=True) + w_short_i*short_ix(i, sigma, c=True) + w_push*success_ix(i, sigma2, c=True) + (1 - (w_nudge + w_short_i + w_push))*other_ix(i, c=True))

    plt.figure()
    plt.plot(xs, pg, xs, pgc)
    plt.xlim((-1, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(-1,4.1,0.5), ('', 'g.x_0 - d.w', '', '', 'g.x_0', '', '', 'g.x_0\n+ d.w', 'g.x_t', '', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_gx')
    plt.legend(['gripper_inside_drawer = False', 'gripper_inside_drawer = True'])
    plt.savefig('constrained_move_gx.png')

    plt.figure()
    plt.plot(xs, pix, xs, pixc)
    plt.xlim((-1, 4))
    plt.ylim(ymin=0, ymax=1.2)
    plt.xticks(np.arange(-1,4.1,0.5), ('', 'g.x_0 - d.w', '', '', 'g.x_0', '', '', 'g.x_0\n+ d.w', 'g.x_t', '', ''))
    plt.tick_params(axis='y', left='off', right='off', labelleft='off')
    plt.ylabel('p_ix')
    plt.legend(['gripper_inside_drawer = False', 'gripper_inside_drawer = True'])
    plt.savefig('constrained_move_ix.png')


if __name__ == '__main__':

    plot_grasp()
    plot_place()
    plot_move()
    plot_move_constrained()
