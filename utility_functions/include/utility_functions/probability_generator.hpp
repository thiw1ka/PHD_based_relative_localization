#include <random>
#include <iostream>

#ifndef PROBABILITY_GENERATOR_HPP
#define PROBABILITY_GENERATOR_HPP

/**
 * @brief this class creates a probability generator.
 * generator will be created as an object. 
 * 
 */
struct ProbabilityGenerator {
    /**
     * @brief Construct a new Robot Probability Container object
     * @param p probability (between 0.0 - 1.0);
     */
    ProbabilityGenerator (double p) {
        generator_ = new std::mt19937(rd_());
        bernoulli_dist_ = new std::bernoulli_distribution(p);
        std::printf("[ProbabilityGenerator] generator created with probability: %f \n", p);        
    };

    std::random_device rd_;

    std::mt19937* generator_;

    std::bernoulli_distribution* bernoulli_dist_; //pd generator

    /*request a sample based on the probability*/
    bool get_sample () {
        return bernoulli_dist_->operator()(*generator_);
    }
};

#endif