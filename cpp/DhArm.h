/**
 * @file  DhArm.h
 * @brief Manipulator specialized for DH Links
 * @author Frank Dellaert and Mandy Xie
 */

#pragma once

#include <Arm.h>
#include <DhLink.h>

#include <vector>

namespace manipulator {

using DhLinkVector = std::vector<DhLink>;
class DhArm : public Arm<DhLink> {
  using Arm<DhLink>::Arm;
};

}  // namespace manipulator
