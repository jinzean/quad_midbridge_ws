#include "quad_midbridge_mpcc/mpcc_solver_interface.hpp"

#include <memory>
#include <string>

#include "quad_midbridge_mpcc/acados_generated_solver.hpp"
#include "quad_midbridge_mpcc/acados_skeleton_solver.hpp"
#include "quad_midbridge_mpcc/full_state_placeholder_solver.hpp"
#include "quad_midbridge_mpcc/reference_model_solver.hpp"

namespace quad_midbridge_mpcc
{

std::unique_ptr<MpccSolverInterface> createSolver(const std::string & backend_name)
{
  if (backend_name == "acados_generated") {
    return createAcadosGeneratedSolver();
  }
  if (backend_name == "acados_skeleton") {
    return createAcadosSkeletonSolver();
  }
  if (backend_name == "full_state_placeholder") {
    return createFullStatePlaceholderSolver();
  }
  if (backend_name == "reference_model") {
    return createReferenceModelSolver();
  }
  return createFullStatePlaceholderSolver();
}

}  // namespace quad_midbridge_mpcc
