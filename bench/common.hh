#pragma once

// Shared by coven_bench and coven_load: benchmark scenes, and the machine description in each CSV row.

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/vector.hh>

namespace coven::bench
{
    using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

    // The scene `name`, with every obstacle moved by `shift` along x. `name` is empty, cage (VAMP's sphere
    // cage example) or spheresN (N random spheres, fixed seed). Exits on an unknown name.
    inline auto make_environment(const std::string &name, float shift) -> Environment
    {
        vamp::collision::Environment<float> environment;
        const auto add = [&](float x, float y, float z, float radius)
        { environment.spheres.emplace_back(vamp::collision::factory::sphere::flat(x + shift, y, z, radius)); };

        if (name == "cage")
        {
            static const std::vector<std::array<float, 3>> centers = {
                {0.55, 0, 0.25},     {0.35, 0.35, 0.25}, {0, 0.55, 0.25},    {-0.55, 0, 0.25},
                {-0.35, -0.35, 0.25}, {0, -0.55, 0.25},   {0.35, -0.35, 0.25}, {0.35, 0.35, 0.8},
                {0, 0.55, 0.8},      {-0.35, 0.35, 0.8}, {-0.55, 0, 0.8},    {-0.35, -0.35, 0.8},
                {0, -0.55, 0.8},     {0.35, -0.35, 0.8},
            };

            for (const auto &c : centers)
            {
                add(c[0], c[1], c[2], 0.2F);
            }
        }
        else if (name.rfind("spheres", 0) == 0)
        {
            // Small spheres scattered through the workspace, clear of the column above the base
            std::mt19937 generator(759);
            const auto uniform = [&](float low, float high)
            { return low + (high - low) * static_cast<float>(generator() >> 8) / static_cast<float>(1U << 24); };

            const auto n = std::stoul(name.substr(7));
            while (environment.spheres.size() < n)
            {
                const auto x = uniform(-0.9F, 0.9F);
                const auto y = uniform(-0.9F, 0.9F);
                const auto z = uniform(0.05F, 1.1F);
                if (x * x + y * y > 0.3F * 0.3F)
                {
                    add(x, y, z, 0.06F);
                }
            }
        }
        else if (name != "empty")
        {
            std::cerr << "unknown environment " << name << std::endl;
            std::exit(2);
        }

        environment.sort();
        return Environment(environment);
    }

    inline auto cpu_model() -> std::string
    {
        std::ifstream cpuinfo("/proc/cpuinfo");
        for (std::string line; std::getline(cpuinfo, line);)
        {
            if (line.rfind("model name", 0) == 0)
            {
                auto model = line.substr(line.find(':') + 2);
                std::replace(model.begin(), model.end(), ',', ' ');
                return model;
            }
        }

        return "unknown";
    }

    inline auto compiler() -> std::string
    {
#if defined(__clang__)
        return std::string("clang ") + __clang_version__;
#else
        return std::string("gcc ") + __VERSION__;
#endif
    }
}  // namespace coven::bench
