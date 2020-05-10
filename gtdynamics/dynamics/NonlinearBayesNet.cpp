/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearBayesNet.cpp
 * @brief   Chordal Bayes Net, the result of eliminating a factor graph
 * @brief   NonlinearBayesNet
 * @author  Mandy Xie
 */

// \callgraph

#pragma once

#include <gtdynamics/dynamics/NonlinearBayesNet.h>
#include <gtdynamics/dynamics/NonlinearConditional.h>
#include <gtsam/inference/FactorGraph-inst.h>

// Instantiate base class
template class gtsam::FactorGraph<gtdynamics::NonlinearConditional>;