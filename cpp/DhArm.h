/**
 * @file  DhArm.h
 * @brief Manipulator specialized for DH Links
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <Arm.h>
#include <DHLink.h>  // TODO(manxie): fix to DhLink (type *and* file)

#include <vector>

namespace manipulator {

using DH_LinkVector = std::vector<DH_Link>;
class DhArm : public Arm<DH_Link> {
  using Arm<DH_Link>::Arm;
};

}  // namespace manipulator
