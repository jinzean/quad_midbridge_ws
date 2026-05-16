#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "quad_midbridge_mpcc/mpcc_types.hpp"

namespace quad_midbridge_mpcc
{

static constexpr std::size_t kGeneratedParamDim = 32;
static constexpr std::size_t kGeneratedConstraintDim = 9;

struct GeneratedStageBuffers
{
  std::array<double, AcadosModelLayout::kNx> x_ref{};
  std::array<double, AcadosModelLayout::kNu> u_ref{};
  std::array<double, kGeneratedParamDim> p{};  // runtime parameter vector for generated acados code
  std::array<double, kGeneratedConstraintDim> lh{};  // lower bounds for nonlinear h constraints
  std::array<double, kGeneratedConstraintDim> uh{};  // upper bounds for nonlinear h constraints
  std::array<double, 5> lbu{};  // lower bounds for [jx, jy, jz, psi_dot, s_dot]
  std::array<double, 5> ubu{};  // upper bounds for [jx, jy, jz, psi_dot, s_dot]
};

struct GeneratedTerminalBuffers
{
  std::array<double, AcadosModelLayout::kNx> x_ref{};
  std::array<double, kGeneratedParamDim> p{};
};

struct GeneratedSolverWorkspace
{
  bool created{false};
  void * capsule{nullptr};
};

class AcadosGeneratedBindings
{
public:
  bool compiledIn() const;
  std::string reason() const;

  bool create(GeneratedSolverWorkspace & ws) const;
  void destroy(GeneratedSolverWorkspace & ws) const;

  bool setInitialState(GeneratedSolverWorkspace & ws, const std::array<double, AcadosModelLayout::kNx> & x0) const;
  bool setStage(GeneratedSolverWorkspace & ws, int stage_idx, const GeneratedStageBuffers & stage) const;
  bool setTerminal(GeneratedSolverWorkspace & ws, const GeneratedTerminalBuffers & term) const;
  bool setWarmStart(GeneratedSolverWorkspace & ws, const std::array<double, AcadosModelLayout::kNu> & u_prev) const;

  bool solve(GeneratedSolverWorkspace & ws, int & status_code) const;
  bool getFirstControl(GeneratedSolverWorkspace & ws, std::array<double, AcadosModelLayout::kNu> & u0) const;
  bool getNextState(GeneratedSolverWorkspace & ws, std::array<double, AcadosModelLayout::kNx> & x1) const;
  bool getCost(GeneratedSolverWorkspace & ws, double & cost) const;
};

GeneratedStageBuffers buildGeneratedStageBuffers(const AcadosStageData & stage);
GeneratedTerminalBuffers buildGeneratedTerminalBuffers(const AcadosTerminalData & term);

}  // namespace quad_midbridge_mpcc
