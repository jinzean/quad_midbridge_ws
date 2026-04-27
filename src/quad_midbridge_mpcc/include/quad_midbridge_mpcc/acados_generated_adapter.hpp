#pragma once

#include <array>
#include <string>

#include "quad_midbridge_mpcc/acados_generated_bindings.hpp"
#include "quad_midbridge_mpcc/mpcc_types.hpp"

namespace quad_midbridge_mpcc
{

struct AcadosGeneratedAdapterResult
{
  bool ok{false};
  bool used_generated_code{false};
  std::string status{"uninitialized"};
  double objective_value{0.0};
  std::array<double, AcadosModelLayout::kNx> x1{};
  std::array<double, AcadosModelLayout::kNu> u0{};
};

class AcadosGeneratedAdapter
{
public:
  bool available() const;
  std::string availabilityReason() const;
  AcadosGeneratedAdapterResult solve(const AcadosProblemData & problem) const;

private:
  bool pushProblem(
    GeneratedSolverWorkspace & ws,
    const AcadosProblemData & problem,
    std::string & status) const;

  AcadosGeneratedBindings bindings_{};
};

}  // namespace quad_midbridge_mpcc
