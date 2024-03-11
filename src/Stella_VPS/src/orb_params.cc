#include "Stella_VPS/orb_params.h"

orb_params::orb_params(const std::string& name, const float scale_factor, const unsigned int num_levels,
               const unsigned int ini_fast_thr, const unsigned int min_fast_thr)
               : name_(name), scale_factor_(scale_factor), log_scale_factor_(std::log(scale_factor)),
                 num_levels_(num_levels), ini_fast_thr_(ini_fast_thr), min_fast_thr_(min_fast_thr)
{
    scale_factors_ = calc_scale_factors(num_levels_, scale_factor_);
    inv_scale_factors_ = calc_inv_scale_factors(num_levels_, scale_factor_);
    level_sigma_sq_ = calc_level_sigma_sq(num_levels_, scale_factor_);
    inv_level_sigma_sq_ = calc_inv_level_sigma_sq(num_levels_, scale_factor_);
}
orb_params::orb_params(const std::string& name) : orb_params(name, 1.2, 1, 20, 7) {};

std::vector<float> orb_params::calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor)
{
    std::vector<float> scale_factors(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factors.at(level) = scale_factor * scale_factors.at(level - 1);
    }
    return scale_factors;
}

//! Calculate inverses of scale factors
std::vector<float> orb_params::calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor)
{
    std::vector<float> inv_scale_factors(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        inv_scale_factors.at(level) = (1.0f / scale_factor) * inv_scale_factors.at(level - 1);
    }
    return inv_scale_factors;
}

//! Calculate squared sigmas at all levels
std::vector<float> orb_params::calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor)
{
    float scale_factor_at_level = 1.0;
    std::vector<float> level_sigma_sq(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        level_sigma_sq.at(level) = scale_factor_at_level * scale_factor_at_level;
    }
    return level_sigma_sq;
}

//! Calculate inverses of squared sigmas at all levels
std::vector<float> orb_params::calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor)
{
    float scale_factor_at_level = 1.0;
    std::vector<float> inv_level_sigma_sq(num_scale_levels, 1.0);
    for (unsigned int level = 1; level < num_scale_levels; ++level) {
        scale_factor_at_level = scale_factor * scale_factor_at_level;
        inv_level_sigma_sq.at(level) = 1.0f / (scale_factor_at_level * scale_factor_at_level);
    }
    return inv_level_sigma_sq;
}