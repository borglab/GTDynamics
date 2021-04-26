import gtsam
import gtdynamics as gtd
import numpy as np


kf = gtd.KeyFormatter()
def print_net(net):
    print("Bayes Net:")
    for i in range(net.size()):
        gc = net.at(i)
        keys = gc.keys()
        print('{:d} :\t {} = R \ ( S * {} )'.format(
            i,
            kf(keys[0]),
            ' + S * '.join(kf(key) for key in keys[1:])))
def print_ordering(ordering):
    for i in range(ordering.size()):
        print(kf(ordering.at(i)))
def print_both(ordering, net):
    kf = lambda x : str(x)
    print("Bayes Net:")
    for i in range(net.size()):
        gc = net.at(i)
        keys = gc.keys()
        print('{:2d} :  {:3s}{} = R \ ( S * {} )'.format(
            i,
            kf(ordering.at(i)),
            kf(keys[0]),
            ' + S * '.join(kf(key) for key in keys[1:])))

N = 1
lid = 1

ordering = gtsam.Ordering()
ordering.push_back(0)
ordering.push_back(1)
ordering.push_back(2)

gfg = gtsam.GaussianFactorGraph()
for ki in range(2):
    gfg.add(ki, np.eye(1), [0], gtsam.noiseModel.Isotropic.Sigma(1, 1.))
gfg.add(
    2, [1],
    0, [1],
    np.zeros(1),
    gtsam.noiseModel.Isotropic.Sigma(1, 1.)
)

net = gfg.eliminateSequential(ordering)
print_both(ordering, net)
print(net)