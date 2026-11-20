// MotionBenchMaker benchmark: solves every valid Panda problem once and prints one CSV row per problem.
//
//   coven_mbm --method rrtc|lazy|eager [--nodes 10000] [--threads 1] [--connections 64]
//             [--data bench/mbm_panda.txt] [--commit HASH] [--header]
//
// rrtc         VAMP's RRT-Connect, as VAMP's own evaluate_mbm.py runs it: range 1.0, a Halton sampler
//              reset for every problem, VAMP's default path simplification afterwards.
// lazy, eager  Coven's planning service on one roadmap of --nodes vertices, built once in free space
//              (not timed). Each problem is a new scene: setup_ns is set_scene (vertex checks, and with
//              eager every edge), plan_ns is the plan request. The path is simplified the same way.
//
// cost is the path length in configuration space before simplification, simplified_cost after.

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <omp.h>

#include <coven/service.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/panda.hh>

#include "common.hh"

using Robot = vamp::robots::Panda;
static constexpr std::size_t rake = vamp::FloatVectorWidth;
using PPRM = coven::PPRM<Robot, rake, Robot::resolution>;
using Service = coven::Service<Robot, rake, Robot::resolution>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;
using Path = vamp::planning::Path<Robot>;
using Clock = std::chrono::steady_clock;

static auto cost(const Path &path) -> double
{
    double total = 0;
    for (std::size_t i = 0; i + 1 < path.size(); ++i)
    {
        total += path[i].distance(path[i + 1]);
    }

    return total;
}

static auto nanoseconds(Clock::duration d) -> long long
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
}

auto main(int argc, char **argv) -> int
{
    std::map<std::string, std::string> args{
        {"method", "rrtc"}, {"nodes", "10000"}, {"threads", "1"}, {"connections", "64"},
        {"data", coven::bench::problem_file()}, {"commit", "unknown"},
    };

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--header")
        {
            std::cout << "method,scenario,problem,nodes,threads,connections,success,setup_ns,plan_ns,simplify_ns,cost,"
                         "simplified_cost,commit,compiler,cpu"
                      << std::endl;
            return 0;
        }
        if (arg.rfind("--", 0) == 0 and args.count(arg.substr(2)) and i + 1 < argc)
        {
            args[arg.substr(2)] = argv[++i];
        }
        else
        {
            std::cerr << "bad argument " << arg << std::endl;
            return 2;
        }
    }

    const auto method = args["method"];
    if (method != "rrtc" and method != "lazy" and method != "eager")
    {
        std::cerr << "bad method" << std::endl;
        return 2;
    }

    const auto nodes = std::stoul(args["nodes"]);
    const auto threads = std::stoi(args["threads"]);
    const auto problems = coven::bench::load_problems(args["data"]);
    const auto halton = []() { return std::make_shared<vamp::rng::Halton<Robot>>(); };
    const vamp::planning::SimplifySettings simplify_settings;
    const auto compiler = coven::bench::compiler();
    const auto cpu = coven::bench::cpu_model();

    // Coven: one roadmap in free space, built with every core
    std::unique_ptr<Service> service;
    if (method != "rrtc")
    {
        omp_set_num_threads(omp_get_num_procs());
        vamp::planning::RoadmapSettings<vamp::planning::PRMStarNeighborParams> settings(
            vamp::planning::PRMStarNeighborParams(Robot::dimension, Robot::space_measure()));
        settings.max_samples = nodes;
        settings.max_iterations = 1000 * nodes;
        const Robot::Configuration seed(problems.front().start);
        auto roadmap = PPRM::build_roadmap(seed, seed, coven::bench::Environment(), settings, halton());

        coven::ServiceOptions options;
        options.threads = threads;
        options.eager = method == "eager";
        options.connections = std::stoul(args["connections"]);
        service = std::make_unique<Service>(std::move(roadmap), options);
    }
    omp_set_num_threads(threads);

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 1.0F;  // VAMP's ROBOT_RRT_RANGES["panda"]

    for (const auto &problem : problems)
    {
        if (not problem.valid)
        {
            continue;
        }

        const Robot::Configuration start(problem.start);
        const Robot::Configuration goal(problem.goal);
        const coven::bench::Environment environment(problem.environment);
        auto rng = halton();
        Path path;
        long long setup = 0;
        long long plan = 0;

        if (method == "rrtc")
        {
            const auto result = RRTC::solve(start, goal, environment, rrtc_settings, rng);
            path = result.path;
            plan = static_cast<long long>(result.nanoseconds);
        }
        else
        {
            const auto begin = Clock::now();
            service->set_scene("mbm", environment).get();
            const auto set = Clock::now();
            const auto reply = service->plan("mbm", start, goal).get();
            setup = nanoseconds(set - begin);
            plan = nanoseconds(Clock::now() - set);
            path = reply.path;
        }

        long long simplify = 0;
        double simplified = 0;
        if (not path.empty())
        {
            const auto result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
                path, environment, simplify_settings, rng);
            simplify = static_cast<long long>(result.nanoseconds);
            simplified = cost(result.path);
        }

        std::cout << method << ',' << problem.scenario << ',' << problem.number << ','
                  << (method == "rrtc" ? 0 : nodes) << ',' << threads << ','
                  << (method == "rrtc" ? "0" : args["connections"]) << ',' << (path.empty() ? 0 : 1) << ','
                  << setup << ',' << plan << ',' << simplify << ',' << cost(path) << ',' << simplified << ','
                  << args["commit"] << ',' << compiler << ',' << cpu << std::endl;
    }

    return 0;
}
