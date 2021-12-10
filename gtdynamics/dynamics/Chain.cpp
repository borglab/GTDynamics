/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Chain.cpp
 * @brief Create a serial kinematic chain.
 * @author Dan Barladeanu, Frank Dellaert.
 */

#include <gtdynamics/dynamics/Chain.h>

namespace gtdynamics {

void Chain::monoidOperation(const Pose3 &aTb, const Matrix &bAj,
                            const Pose3 &bTc, const Matrix &cAk, 
                            Pose3 &aTc, Matrix &cAjk) {
    
    aTc = aTb.compose(bTc);
    
    Matrix c_Ad_b = bTc.inverse().AdjointMap();
    Matrix mult = c_Ad_b * bAj;
    Matrix out(mult.rows(), mult.cols() + cAk.cols());
    out.leftCols(mult.cols()) = mult;
    out.rightCols(cAk.cols()) = cAk;
    cAjk = out;
}

Chain Chain::compose(std::vector<Chain> &chain_vector){
    Pose3 T, Ti, Tres;
    Matrix A, Ai, Ares;
    T = chain_vector[0].sMb();
    A = chain_vector[0].axes();
    int i = 1 ;
    for (i ; i < chain_vector.size() ; ++i) {
      Ti = chain_vector[i].sMb();
      Ai = chain_vector[i].axes();
      monoidOperation(T,A,Ti,Ai,Tres,Ares);
      T = Tres;
      A = Ares;
    }
    return Chain(T,A);
}

Pose3 Chain::poe(const Vector &q, boost::optional<Pose3 &> fTe,
          boost::optional<Matrix &> J) {
            
if (q.size() != axes_.cols()) {
  throw std::runtime_error(
      "number of angles in q different from number of cols in axes");
    }

  std::vector<Pose3> exp;
  for (int i = 0; i < q.size(); ++i) {
    exp.push_back(Pose3::Expmap(axes_.col(i) * q(i)));
  }

  Pose3 poe;
  if (!J) {
    poe = sMb_;
    for (auto &expmap : exp){
      poe = poe.compose(expmap);
    }
    if (fTe) {
      poe = poe.compose(*fTe);
    }
  }
  else {
    Matrix Empty(6, 0), A, Ares;
    Pose3 T, Tres;
    T = sMb_;
    A = Empty;
    for (int j = 0; j < q.size(); ++j) {
      monoidOperation(T,A,exp[j],axes_.col(j), Tres, Ares);
      T = Tres;
      A = Ares;
    }
    if (fTe) {
      monoidOperation(T,A,*fTe,Empty, Tres, Ares);
      T = Tres;
      A = Ares;
    }
    poe = T;
    *J = A;

  }
  return poe;
}
}
