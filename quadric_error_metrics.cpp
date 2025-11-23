#include <map>

#include <symengine/expression.h>
#include <symengine/solve.h>
#include <symengine/basic.h>
#include <symengine/add.h>
#include <symengine/matrix.h>
#include <symengine/integer.h>
#include <Eigen/Dense>

#include "quadric_error_metrics.h"
#include "charles_symengine_common.h"
#include "unit_test.h"

std::tuple<double, double, double, double> quadric_error_metrics(
    const std::vector<std::tuple<double, double, double, double>>& planes
)
{
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    double min_val = 0.0f;
    for(auto plane: planes)
    {
        Eigen::Vector3d temp_v;
        temp_v << std::get<0>(plane)
            ,std::get<1>(plane)
            ,std::get<2>(plane);
        Eigen::Matrix3d temp_A = temp_v * temp_v.transpose();
        A = A + temp_A;

        temp_v = 2 * std::get<3>(plane) * temp_v;
        b = b + temp_v;

        min_val = min_val + std::get<3>(plane) * std::get<3>(plane);
    }
    auto x_min = -0.5f * A.ldlt().solve(b);
    min_val = x_min.dot(A * x_min) + b.dot(x_min) + min_val;
    return {x_min.x(), x_min.y(), x_min.z(), min_val};
}

std::tuple<double, double, double, double> quadric_error_metrics_deprecate(
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

// passed
TEST(GlobalTest, eigen_quadric_solver_test)
{
    std::vector<std::tuple<double, double, double, double>> coefficients = {
        {1,2,3,1},
        {1,1,1,2},
        {3,8,7,3},
        {7,5,13,21},
    };
    auto ret1 = quadric_error_metrics(coefficients);
    auto ret2 = quadric_error_metrics_deprecate(coefficients);
    ASSERT_EQ(true, std::abs(std::get<0>(ret1) - std::get<0>(ret2)) < 0.000001f);
    ASSERT_EQ(true, std::abs(std::get<1>(ret1) - std::get<1>(ret2)) < 0.000001f);
    ASSERT_EQ(true, std::abs(std::get<2>(ret1) - std::get<2>(ret2)) < 0.000001f);
    std::cout << std::get<3>(ret1) << std::endl;
    std::cout << std::get<3>(ret2) << std::endl;
    ASSERT_EQ(true, std::abs(std::get<3>(ret1) - std::get<3>(ret2)) < 0.000001f);
}