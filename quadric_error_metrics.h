/**
 * @file quadric_error_metrics.h
 * @author your name (you@domain.com)
 * @brief find a new 3d point that minimize quadric error metrics of two old points
 * @version 0.1
 * @date 2024-12-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __CHARLES_QUADRIC_ERROR_METRICS_H__
#define __CHARLES_QUADRIC_ERROR_METRICS_H__

#include <vector>
#include <tuple>

std::tuple<double, double, double, double> quadric_error_metrics(
    const std::vector<std::tuple<double, double, double, double>>& planes
);

#endif