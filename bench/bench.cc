// Benchmark driver. Prints one CSV row per timed run.
//
//   coven_bench --mode serial|parallel|revalidate [--env cage] [--nodes 10000] [--batch 1024]
//               [--threads 1] [--schedule dynamic] [--chunk 0] [--repeats 20] [--warmup 1]
//               [--commit HASH] [--header] [--check]
//
// serial      unmodified vamp PRM::build_roadmap()
// parallel    PPRM::build_roadmap(), with its time breakdown
// revalidate  PPRM::revalidate() of a finished roadmap after the obstacles moved
//
// --env is empty, cage, or spheresN (N random spheres, fixed seed).
// --check also compares the parallel roadmap against the serial one and fails if they differ.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <omp.h>

#include <vamp/collision/factory.hh>
#include <coven/pprm.hh>
#include <vamp/planning/prm.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
static constexpr std::size_t rake = vamp::FloatVectorWidth;
using Environment = vamp::collision::Environment<vamp::FloatVector<rake>>;
using Roadmap = vamp::planning::Roadmap<Robot>;
using PRM = vamp::planning::PRM<Robot, rake, Robot::resolution>;
using PPRM = coven::PPRM<Robot, rake, Robot::resolution>;

// Start and goal of VAMP's sphere cage example
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};

// The scene `name`, with every obstacle moved by `shift` along x.
static auto make_environment(const std::string &name, float shift) -> Environment
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

static auto edge_set(const Roadmap &roadmap) -> std::vector<std::pair<std::size_t, std::size_t>>
{
    std::vector<std::pair<std::size_t, std::size_t>> edges;
    for (std::size_t j = 0; j < roadmap.edges.size(); ++j)
    {
        for (const auto i : roadmap.edges[j])
        {
            if (i < j)
            {
                edges.emplace_back(i, j);
            }
        }
    }

    std::sort(edges.begin(), edges.end());
    return edges;
}

static auto cpu_model() -> std::string
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

auto main(int argc, char **argv) -> int
{
    std::map<std::string, std::string> args{
        {"mode", "parallel"}, {"env", "cage"},  {"nodes", "10000"},  {"batch", "1024"},
        {"threads", "1"},     {"chunk", "0"},   {"repeats", "20"},   {"warmup", "1"},
        {"commit", "unknown"}, {"schedule", "dynamic"},
    };

    bool header = false;
    bool check = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--header")
        {
            header = true;
        }
        else if (arg == "--check")
        {
            check = true;
        }
        else if (arg.rfind("--", 0) == 0 and args.count(arg.substr(2)) and i + 1 < argc)
        {
            args[arg.substr(2)] = argv[++i];
        }
        else
        {
            std::cerr << "bad argument " << arg << std::endl;
            return 2;
        }
    }

    const auto mode = args["mode"];
    const auto nodes = std::stoul(args["nodes"]);
    const auto threads = std::stoi(args["threads"]);
    const auto chunk = std::stoi(args["chunk"]);
    const auto repeats = std::stoi(args["repeats"]);
    const auto warmup = std::stoi(args["warmup"]);
    const std::map<std::string, omp_sched_t> schedules{
        {"static", omp_sched_static}, {"dynamic", omp_sched_dynamic}, {"guided", omp_sched_guided}};
    if (not schedules.count(args["schedule"]) or (mode != "serial" and mode != "parallel" and mode != "revalidate"))
    {
        std::cerr << "bad mode or schedule" << std::endl;
        return 2;
    }

    omp_set_num_threads(threads);
    omp_set_schedule(schedules.at(args["schedule"]), chunk);

    const auto environment = make_environment(args["env"], 0.F);
    vamp::planning::RoadmapSettings<vamp::planning::PRMStarNeighborParams> settings(
        vamp::planning::PRMStarNeighborParams(Robot::dimension, Robot::space_measure()));
    settings.max_samples = nodes;
    settings.max_iterations = 1000 * nodes;
    settings.batch_size = std::stoul(args["batch"]);

    const Robot::Configuration start_v(start);
    const Robot::Configuration goal_v(goal);
    const auto rng = []() { return std::make_shared<vamp::rng::Halton<Robot>>(); };

    if (header)
    {
        std::cout << "mode,env,nodes,batch,threads,schedule,chunk,repeat,total_ns,sample_ns,neighbor_ns,"
                     "validate_ns,insert_ns,edges_checked,vertices,edges,commit,compiler,cpu"
                  << std::endl;
    }

    if (check)
    {
        const auto serial = PRM::build_roadmap(start_v, goal_v, environment, settings, rng());
        const auto parallel = PPRM::build_roadmap(start_v, goal_v, environment, settings, rng());
        if (edge_set(serial) != edge_set(parallel) or serial.vertices.size() != parallel.vertices.size() or
            serial.iterations != parallel.iterations)
        {
            std::cerr << "MISMATCH: parallel roadmap differs from serial roadmap" << std::endl;
            return 1;
        }

        std::cerr << "check passed: " << parallel.vertices.size() << " vertices, " << edge_set(parallel).size()
                  << " edges, identical" << std::endl;
    }

    // Revalidation rechecks one fixed roadmap against the scene after it moved 5 cm
    Roadmap fixed;
    coven::RoadmapGraph graph;
    const auto moved = make_environment(args["env"], 0.05F);
    if (mode == "revalidate")
    {
        omp_set_num_threads(omp_get_num_procs());
        fixed = PPRM::build_roadmap(start_v, goal_v, environment, settings, rng());
        graph = PPRM::graph(fixed);
        omp_set_num_threads(threads);
    }

#if defined(__clang__)
    const std::string compiler = std::string("clang ") + __clang_version__;
#else
    const std::string compiler = std::string("gcc ") + __VERSION__;
#endif
    const auto cpu = cpu_model();

    for (int repeat = -warmup; repeat < repeats; ++repeat)
    {
        coven::PPRMStats stats;
        std::size_t total = 0;
        std::size_t n_vertices = 0;
        std::size_t n_edges = 0;

        if (mode == "revalidate")
        {
            const auto begin = std::chrono::steady_clock::now();
            const auto validity = PPRM::revalidate(fixed, graph, moved);
            total = vamp::utils::get_elapsed_nanoseconds(begin);
            stats.edges_checked = graph.edges.size();
            n_vertices = std::count(validity.vertices.begin(), validity.vertices.end(), 1);
            n_edges = std::count(validity.edges.begin(), validity.edges.end(), 1);
        }
        else
        {
            const auto roadmap = (mode == "serial") ?
                                     PRM::build_roadmap(start_v, goal_v, environment, settings, rng()) :
                                     PPRM::build_roadmap(start_v, goal_v, environment, settings, rng(), &stats);
            total = roadmap.nanoseconds;
            n_vertices = roadmap.vertices.size();
            for (const auto &neighbors : roadmap.edges)
            {
                n_edges += neighbors.size();
            }

            n_edges /= 2;
        }

        if (repeat >= 0)
        {
            std::cout << mode << ',' << args["env"] << ',' << nodes << ',' << settings.batch_size << ','
                      << threads << ',' << args["schedule"] << ',' << chunk << ',' << repeat << ',' << total
                      << ',' << stats.sample_nanoseconds << ',' << stats.neighbor_nanoseconds << ','
                      << stats.validate_nanoseconds << ',' << stats.insert_nanoseconds << ','
                      << stats.edges_checked << ',' << n_vertices << ',' << n_edges << ',' << args["commit"]
                      << ',' << compiler << ',' << cpu << std::endl;
        }
    }

    return 0;
}
