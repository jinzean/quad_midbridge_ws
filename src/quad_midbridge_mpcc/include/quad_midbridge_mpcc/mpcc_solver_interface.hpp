#pragma once

#include <memory>
#include <string>

#include "quad_midbridge_mpcc/mpcc_types.hpp"

namespace quad_midbridge_mpcc
{

class MpccSolverInterface
{
public:
  virtual ~MpccSolverInterface() = default;
  virtual std::string name() const = 0;
  virtual SolverOutput solve(const SolverInput & input) = 0;
};

std::unique_ptr<MpccSolverInterface> createSolver(const std::string & backend_name);

}  // namespace quad_midbridge_mpcc
