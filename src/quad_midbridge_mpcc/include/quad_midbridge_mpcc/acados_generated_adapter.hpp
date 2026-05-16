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
  AcadosGeneratedAdapter() = default;
  ~AcadosGeneratedAdapter();

  AcadosGeneratedAdapter(const AcadosGeneratedAdapter &) = delete;
  AcadosGeneratedAdapter & operator=(const AcadosGeneratedAdapter &) = delete;

  bool available() const;
  std::string availabilityReason() const;
  AcadosGeneratedAdapterResult solve(const AcadosProblemData & problem);

private:
  bool pushProblem(
    GeneratedSolverWorkspace & ws,
    const AcadosProblemData & problem,
    std::string & status) const;

  void resetWorkspace();

  AcadosGeneratedBindings bindings_{};
  GeneratedSolverWorkspace workspace_{};
};

}  // namespace quad_midbridge_mpcc
