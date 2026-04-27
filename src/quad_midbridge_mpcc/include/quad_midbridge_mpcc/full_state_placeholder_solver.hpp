#pragma once

#include <memory>

#include "quad_midbridge_mpcc/mpcc_solver_interface.hpp"

namespace quad_midbridge_mpcc
{
std::unique_ptr<MpccSolverInterface> createFullStatePlaceholderSolver();
}  // namespace quad_midbridge_mpcc
