"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  paint_parse.py
@brief Parses a .h file containing the paint trajectory.
@author Frank Dellaert
@author Gerry Chen
"""

import numpy as np
import re

UINTEXPR = '([0-9]*?)'
FLOATEXPR = '(-?[0-9]*?\.?[0-9]*?)'

def ParseFile(fname):
    with open(fname) as f:
        # painton
        assert next(f) == 'bool painton[] = {\n', "variable `painton` not found"
        paintons = [bool(int(e)) for e in next(f).strip().split(',')]
        assert next(f) == '};\n', "parse error on variable `painton`"

        # colorinds
        assert next(f) == 'uint8_t colorinds[] = {\n', "variable `colorinds` not found"
        colorinds = [int(e) for e in next(f).strip().split(',')]
        assert next(f) == '};\n', "parse error on variable `colorinds`"

        # colorpalette
        assert next(f) == 'uint8_t colorpalette[][3] = {\n', "variable `colorpalette` not found"
        colors = []
        while True:
            matches = re.search(f'\\{{{UINTEXPR}, {UINTEXPR}, {UINTEXPR}\\}},?', next(f))
            if matches is None:
                break  # next(f) was '};\n'
            colors.append([int(g) for g in matches.groups()])
        colors = np.array(colors)

        # trajectory
        assert next(f) == 'float traj[][2] = {\n', "variable `traj` not found"
        traj = []
        while True:
            matches = re.search(f'\{{{FLOATEXPR},{FLOATEXPR}}},?',
                                re.sub('\s', '', next(f)))  # remove any whitespace
            if matches is None:
                break  # next(f) was '};\n'
            traj.append([float(g) for g in matches.groups()])
        traj = np.array(traj)

        # EOF
        try:
            next(f)
        except StopIteration:
            return paintons, colorinds, colors, traj
        assert False, 'End of file expected'

def writeControls(fname, gains_ff):
    with open(fname, 'w') as f:
        Ks, uffs, vffs, xffs = zip(*gains_ff)
        f.write('// u = K * ([v;x]-[vff;xff]) + uff\n')
        f.write('float xffs[][2] = {\n')
        for xff in xffs:
            f.write('\t{{{:f}, {:f}}},\n'.format(*xff.translation()[[0, 2]]))
        f.write('};\n')
        f.write('float vffs[][2] = {\n')
        for vff in vffs:
            f.write('\t{{{:f}, {:f}}},\n'.format(*vff[[3, 5]]))
        f.write('};\n')
        f.write('float uffs[][4] = {\n')
        for uff in uffs:
            f.write('\t{{{:f}, {:f}, {:f}, {:f}}},\n'.format(*uff))
        f.write('};\n')
        f.write('// vx, vy, x, y\n')
        f.write('float Ks[][4][4] = {\n\t')
        for K in Ks:
            f.write('{\n')
            for Krow in K:
                f.write('\t {{{:f}, {:f}, {:f}, {:f}}},\n'.format(*Krow[[3,5,9,11]]))
            f.write('\t},')
        f.write('\n};\n')
