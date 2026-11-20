#pragma once

// Shared by coven_bench and coven_load: benchmark scenes, and the machine description in each CSV row.

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/vector.hh>

namespace coven::bench
{
    using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

    // One MotionBenchMaker problem for the Panda, as bench/mbm_export.py writes them
    struct Problem
    {
        std::string scenario;
        int number{0};
        bool valid{false};  // start and goal are collision-free
        std::array<float, 7> start{};
        std::array<float, 7> goal{};
        vamp::collision::Environment<float> environment;
    };

    // Adds obstacles the way VAMP's Python Environment.add_cuboid() and add_capsule() do
    inline void add_cuboid(vamp::collision::Environment<float> &e, const vamp::collision::Cuboid<float> &c)
    {
        (c.axis_3_z == 1. ? e.z_aligned_cuboids : e.cuboids).emplace_back(c);
    }

    inline void add_capsule(vamp::collision::Environment<float> &e, const vamp::collision::Capsule<float> &c)
    {
        (c.xv == 0. and c.yv == 0. ? e.z_aligned_capsules : e.capsules).emplace_back(c);
    }

    // All problems in the file, with every obstacle moved by `shift` along x. Exits if it cannot be read.
    inline auto load_problems(const std::string &path, float shift = 0.F) -> std::vector<Problem>
    {
        namespace factory = vamp::collision::factory;
        std::ifstream file(path);
        if (not file)
        {
            std::cerr << "cannot read " << path << " (run from the repository root, or pass --data)" << std::endl;
            std::exit(2);
        }

        std::vector<Problem> problems;
        for (std::string line; std::getline(file, line);)
        {
            std::istringstream in(line);
            std::string kind;
            in >> kind;
            if (kind.empty() or kind[0] == '#')
            {
                continue;
            }

            if (kind == "problem")
            {
                problems.emplace_back();
                in >> problems.back().scenario >> problems.back().number >> problems.back().valid;
                continue;
            }

            auto &p = problems.back();
            std::vector<float> v;
            for (float x; in >> x;)
            {
                v.push_back(x);
            }

            if (kind == "start" or kind == "goal")
            {
                std::copy(v.begin(), v.end(), (kind == "start" ? p.start : p.goal).begin());
            }
            else if (kind == "sphere")
            {
                p.environment.spheres.emplace_back(factory::sphere::flat(v[0] + shift, v[1], v[2], v[3]));
            }
            else if (kind == "cuboid")
            {
                add_cuboid(p.environment, factory::cuboid::flat(v[0] + shift, v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]));
            }
            else if (kind == "capsule")
            {
                add_capsule(p.environment, factory::cylinder::center::flat(v[0] + shift, v[1], v[2], v[3], v[4], v[5], v[6], v[7]));
            }
        }

        for (auto &p : problems)
        {
            p.environment.sort();
        }

        return problems;
    }

    // Where coven_bench and coven_mbm look for the problems; MBM_DATA overrides it
    inline auto problem_file() -> std::string
    {
        const char *path = std::getenv("MBM_DATA");
        return path ? path : "bench/mbm_panda.txt";
    }

    // The scene `name`, with every obstacle moved by `shift` along x. `name` is empty, cage (VAMP's sphere
    // cage example), spheresN (N random spheres, fixed seed), or mbm:<scenario> (the scene of that
    // MotionBenchMaker scenario's first problem). Exits on an unknown name.
    inline auto make_environment(const std::string &name, float shift) -> Environment
    {
        if (name.rfind("mbm:", 0) == 0)
        {
            for (const auto &p : load_problems(problem_file(), shift))
            {
                if (p.scenario == name.substr(4))
                {
                    return Environment(p.environment);
                }
            }

            std::cerr << "unknown MotionBenchMaker scenario " << name.substr(4) << std::endl;
            std::exit(2);
        }

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
