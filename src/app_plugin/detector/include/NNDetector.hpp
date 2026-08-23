#pragma once

#include "Detector.hpp"

#include <memory>

class NNDetector final : public Detector
{
public:
    NNDetector();
    ~NNDetector() override;

    NNDetector(const NNDetector &) = delete;
    NNDetector &operator=(const NNDetector &) = delete;
    NNDetector(NNDetector &&) = delete;
    NNDetector &operator=(NNDetector &&) = delete;

    void process(const app::Context &context) override;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
