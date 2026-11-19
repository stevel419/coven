// Load test of the planning service, in process. Prints one CSV row per run.
//
//   coven_load [--env cage] [--nodes 10000] [--threads 1] [--window-us 0] [--mode lazy|eager]
//              [--rate 1000] [--scenes 1] [--update-ms 100] [--duration 3] [--warmup 1]
//              [--max-in-flight 4096] [--commit HASH] [--header]
//
// Requests arrive open loop, as a Poisson process at --rate per second, each in a random one of
// --scenes scenes, between a random pair from a pool of collision-free configurations. Scene s is
// --env shifted along x by 0.1 sin(2 pi (t / 2 s + s / scenes)); every --update-ms (0: never, t = 0)
// all scenes move. Latency runs from a request's scheduled arrival to its answer, so a slow service
// cannot slow down its own load. Only requests scheduled after the warm-up count. Throughput is their
// answers over the time from the end of the warm-up to the last answer, so it falls below the rate
// once the service saturates. bench/load.py runs the same test over HTTP.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <omp.h>

#include <coven/service.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/panda.hh>

#include "common.hh"

using Robot = vamp::robots::Panda;
static constexpr std::size_t rake = vamp::FloatVectorWidth;
using PPRM = coven::PPRM<Robot, rake, Robot::resolution>;
using Service = coven::Service<Robot, rake, Robot::resolution>;
using Clock = std::chrono::steady_clock;

// Start and goal of VAMP's sphere cage example; they only seed the roadmap
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};

static auto percentile(std::vector<double> values, double p) -> double
{
    if (values.empty())
    {
        return 0;
    }

    const auto k = std::min(values.size() - 1, static_cast<std::size_t>(p * static_cast<double>(values.size())));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(k), values.end());
    return values[k];
}

auto main(int argc, char **argv) -> int
{
    std::map<std::string, std::string> args{
        {"env", "cage"},    {"nodes", "10000"},    {"threads", "1"},   {"window-us", "0"},
        {"mode", "lazy"},   {"rate", "1000"},      {"scenes", "1"},    {"update-ms", "100"},
        {"duration", "3"},  {"warmup", "1"},       {"max-in-flight", "4096"}, {"commit", "unknown"},
    };

    bool header = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--header")
        {
            header = true;
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

    if (header)
    {
        std::cout << "transport,mode,env,nodes,threads,window_us,rate,scenes,update_ms,duration_s,requests,ok,"
                     "overloaded,no_path,throughput,p50_ms,p90_ms,p99_ms,p999_ms,mean_ms,rounds,checks,searches,"
                     "fallbacks,checks_per_round,update_busy,search_busy,check_busy,commit,compiler,cpu"
                  << std::endl;
        return 0;
    }

    const auto env = args["env"];
    const auto nodes = std::stoul(args["nodes"]);
    const auto rate = std::stod(args["rate"]);
    const auto n_scenes = std::stoul(args["scenes"]);
    const auto update_ms = std::stoi(args["update-ms"]);
    const auto duration = std::stod(args["duration"]);
    const auto warmup = std::stod(args["warmup"]);
    if (args["mode"] != "lazy" and args["mode"] != "eager")
    {
        std::cerr << "bad mode" << std::endl;
        return 2;
    }

    // Roadmap in free space, built with every core
    omp_set_num_threads(omp_get_num_procs());
    vamp::planning::RoadmapSettings<vamp::planning::PRMStarNeighborParams> settings(
        vamp::planning::PRMStarNeighborParams(Robot::dimension, Robot::space_measure()));
    settings.max_samples = nodes;
    settings.max_iterations = 1000 * nodes;
    auto roadmap = PPRM::build_roadmap(
        Robot::Configuration(start),
        Robot::Configuration(goal),
        coven::bench::Environment(),
        settings,
        std::make_shared<vamp::rng::Halton<Robot>>());

    coven::ServiceOptions options;
    options.threads = std::stoi(args["threads"]);
    options.window = std::chrono::microseconds(std::stol(args["window-us"]));
    options.eager = args["mode"] == "eager";
    options.max_in_flight = std::stoul(args["max-in-flight"]);
    Service service(std::move(roadmap), options);

    std::vector<std::string> scenes;
    const auto scene_at = [&](std::size_t s, double t)
    {
        const auto phase = t / 2.0 + static_cast<double>(s) / static_cast<double>(n_scenes);
        return coven::bench::make_environment(env, static_cast<float>(0.1 * std::sin(2 * M_PI * phase)));
    };

    for (std::size_t s = 0; s < n_scenes; ++s)
    {
        scenes.push_back("s" + std::to_string(s));
        service.set_scene(scenes.back(), scene_at(s, 0)).get();
    }

    // Collision-free configurations in the unmoved scene
    const auto base = coven::bench::make_environment(env, 0.F);
    std::vector<Robot::Configuration> configurations;
    vamp::rng::Halton<Robot> halton;
    for (int i = 0; i < 777; ++i)
    {
        halton.next();
    }

    while (configurations.size() < 512)
    {
        const auto c = halton.next();
        if (vamp::planning::validate_motion<Robot, rake, 1>(c, c, base))
        {
            configurations.push_back(c);
        }
    }

    const auto t0 = Clock::now();
    const auto since = [&](Clock::time_point t) { return std::chrono::duration<double>(t - t0).count(); };

    std::atomic<bool> stop{false};
    std::thread updater;
    if (update_ms > 0)
    {
        updater = std::thread(
            [&]()
            {
                auto next = t0;
                while (not stop)
                {
                    next += std::chrono::milliseconds(update_ms);
                    std::this_thread::sleep_until(next);
                    for (std::size_t s = 0; s < n_scenes; ++s)
                    {
                        service.set_scene(scenes[s], scene_at(s, since(Clock::now())));
                    }
                }
            });
    }

    struct Sent
    {
        double scheduled;
        double submitted;
        std::future<Service::Reply> reply;
    };

    std::vector<Sent> sent;
    sent.reserve(static_cast<std::size_t>(1.2 * rate * (duration + warmup)) + 16);
    std::mt19937_64 random(759);
    std::exponential_distribution<double> gap(rate);
    std::uniform_int_distribution<std::size_t> pick_scene(0, n_scenes - 1);
    std::uniform_int_distribution<std::size_t> pick(0, configurations.size() - 1);
    for (double scheduled = gap(random); scheduled < warmup + duration; scheduled += gap(random))
    {
        std::this_thread::sleep_until(t0 + std::chrono::duration_cast<Clock::duration>(
                                               std::chrono::duration<double>(scheduled)));
        const auto &a = configurations[pick(random)];
        const auto &b = configurations[pick(random)];
        const auto submitted = since(Clock::now());
        sent.push_back({scheduled, submitted, service.plan(scenes[pick_scene(random)], a, b)});
    }

    std::vector<double> latencies;
    double last_done = warmup + duration;  // when the last measured answer came; past the end when saturated
    std::size_t requests = 0, ok = 0, overloaded = 0, no_path = 0, fallbacks = 0;
    double rounds = 0, checks = 0, searches = 0;
    for (auto &s : sent)
    {
        const auto reply = s.reply.get();
        if (s.scheduled < warmup)
        {
            continue;
        }

        ++requests;
        if (reply.status == coven::PlanStatus::overloaded)
        {
            ++overloaded;
            continue;
        }

        ++ok;
        no_path += reply.path.empty();
        latencies.push_back(1e3 * (s.submitted - s.scheduled + reply.counters.total_seconds));
        last_done = std::max(last_done, s.submitted + reply.counters.total_seconds);
        rounds += reply.counters.rounds;
        checks += reply.counters.checks;
        searches += reply.counters.searches;
        fallbacks += reply.counters.fallback;
    }

    stop = true;
    if (updater.joinable())
    {
        updater.join();
    }

    // Share of the run the dispatcher spent on each phase
    const auto stats = service.stats();
    const auto elapsed = since(Clock::now());
    double mean = 0;
    for (const auto l : latencies)
    {
        mean += l / static_cast<double>(latencies.size());
    }

    const auto per = [&](double total) { return ok ? total / static_cast<double>(ok) : 0.0; };
    std::cout << "inproc," << args["mode"] << ',' << env << ',' << nodes << ',' << options.threads << ','
              << options.window.count() << ',' << rate << ',' << n_scenes << ',' << update_ms << ',' << duration
              << ',' << requests << ',' << ok << ',' << overloaded << ',' << no_path << ','
              << static_cast<double>(ok) / (last_done - warmup) << ',' << percentile(latencies, 0.5) << ','
              << percentile(latencies, 0.9) << ',' << percentile(latencies, 0.99) << ','
              << percentile(latencies, 0.999) << ',' << mean << ',' << per(rounds) << ',' << per(checks) << ','
              << per(searches) << ',' << fallbacks << ','
              << (stats.rounds ? static_cast<double>(stats.checks) / static_cast<double>(stats.rounds) : 0.0)
              << ',' << stats.update_seconds / elapsed << ',' << stats.search_seconds / elapsed << ','
              << stats.check_seconds / elapsed << ',' << args["commit"] << ',' << coven::bench::compiler() << ',' << coven::bench::cpu_model()
              << std::endl;
    return 0;
}
