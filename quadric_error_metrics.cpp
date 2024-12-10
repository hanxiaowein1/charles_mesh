#include <map>

#include <symengine/expression.h>
#include <symengine/solve.h>
#include <symengine/basic.h>
#include <symengine/add.h>
#include <symengine/matrix.h>
#include <symengine/integer.h>

#include "quadric_error_metrics.h"
#include "charles_symengine_common.h"

std::tuple<double, double, double, double> quadric_error_metrics(
    const std::vector<std::tuple<double, double, double, double>>& planes
)
{
    auto x = SymEngine::symbol("x");
    auto y = SymEngine::symbol("y");
    auto z = SymEngine::symbol("z");
    SymEngine::Expression x_(x);
    SymEngine::Expression y_(y);
    SymEngine::Expression z_(z);
    SymEngine::Expression ex;

    for(const auto& plane: planes)
    {
        const auto& [a, b, c, d] = plane;
        ex = ex + SymEngine::pow(a * x_ + b * y_ + c * z_ + d, 2);
    }

    // calculate differential of expression and get extream point
    SymEngine::Expression derivative_x = ex.diff(x);
    SymEngine::Expression derivative_y = ex.diff(y);
    SymEngine::Expression derivative_z = ex.diff(z);

    auto solutions_3d = SymEngine::linsolve({ derivative_x.get_basic(), derivative_y.get_basic(), derivative_z.get_basic() }, { x, y, z });

    double axis_value_x = get_double_from_solution(solutions_3d[0]);
    double axis_value_y = get_double_from_solution(solutions_3d[1]);
    double axis_value_z = get_double_from_solution(solutions_3d[2]);

    std::map<SymEngine::RCP<const SymEngine::Basic>, SymEngine::RCP<const SymEngine::Basic>, SymEngine::RCPBasicKeyLess> substituter{
        {x, SymEngine::real_double(axis_value_x)},
        {y, SymEngine::real_double(axis_value_y)},
        {z, SymEngine::real_double(axis_value_z)},
    };

    auto metrics = substitute_with_number(ex, substituter);
    return {axis_value_x, axis_value_y, axis_value_z, metrics};
    // return metrics;
    // return std::make_tuple(axis_value_x, axis_value_y, axis_value_z);
}