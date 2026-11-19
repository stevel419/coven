#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <coven/service.hh>
#include <vamp/random/halton.hh>

#include "planning_common.hh"

using namespace coven::test;
using namespace std::chrono_literals;

namespace
{
    using PPRM = coven::PPRM<Robot, rake, Robot::resolution>;
    using Service = coven::Service<Robot, rake, Robot::resolution>;
    using Path = vamp::planning::Path<Robot>;
    using Pair = std::pair<Configuration, Configuration>;

    auto free_space_roadmap(std::size_t nodes) -> Roadmap
    {
        return PRM::build_roadmap(
            start(), goal(), Environment(), settings(nodes), std::make_shared<vamp::rng::Halton<Robot>>());
    }

    // n start and goal pairs, both collision-free in the environment
    auto valid_pairs(const Environment &environment, std::size_t n) -> std::vector<Pair>
    {
        vamp::rng::Halton<Robot> rng;
        for (int i = 0; i < 777; ++i)
        {
            rng.next();
        }

        std::vector<Configuration> configurations;
        while (configurations.size() < 2 * n)
        {
            const auto c = rng.next();
            if (valid(c, environment))
            {
                configurations.push_back(c);
            }
        }

        std::vector<Pair> pairs;
        for (std::size_t i = 0; i < n; ++i)
        {
            pairs.emplace_back(configurations[2 * i], configurations[2 * i + 1]);
        }

        return pairs;
    }

    auto cost(const Path &path) -> float
    {
        float total = 0;
        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            total += path[i].distance(path[i + 1]);
        }

        return total;
    }

    // Starts and ends where asked, and every motion along it is collision-free
    auto valid_path(const Path &path, const Pair &pair, const Environment &environment) -> bool
    {
        bool result = path.size() >= 2 and same(path.front(), pair.first) and same(path.back(), pair.second);
        for (std::size_t i = 0; result and i + 1 < path.size(); ++i)
        {
            result = valid(path[i], path[i + 1], environment);
        }

        return result;
    }

    auto options(int threads, std::chrono::microseconds window = 0us, bool eager = false) -> coven::ServiceOptions
    {
        coven::ServiceOptions result;
        result.threads = threads;
        result.window = window;
        result.eager = eager;
        return result;
    }
}  // namespace

TEST_CASE("Service paths have the cost of queries on a revalidated roadmap", "[service]")
{
    const auto roadmap = free_space_roadmap(1500);
    const auto graph = PPRM::graph(roadmap);

    for (const bool eager : {false, true})
    {
        for (const int threads : {1, 4})
        {
            CAPTURE(eager, threads);
            Service service(roadmap, options(threads, 0us, eager));
            std::uint64_t expected_version = 0;
            std::size_t found = 0;
            for (const float shift : {0.F, 0.05F, 0.2F})
            {
                CAPTURE(shift);
                const auto environment = sphere_cage(shift);
                REQUIRE(service.set_scene("cage", environment).get() == ++expected_version);
                const auto validity = PPRM::revalidate(roadmap, graph, environment);

                // All at once, so they share rounds
                const auto pairs = valid_pairs(environment, 20);
                std::vector<std::future<Service::Reply>> replies;
                for (const auto &pair : pairs)
                {
                    replies.push_back(service.plan("cage", pair.first, pair.second));
                }

                for (std::size_t i = 0; i < pairs.size(); ++i)
                {
                    CAPTURE(i);
                    const auto reply = replies[i].get();
                    const auto expected =
                        PPRM::query(roadmap, graph, validity, pairs[i].first, pairs[i].second, environment);
                    REQUIRE(reply.status == coven::PlanStatus::ok);
                    REQUIRE(reply.version == expected_version);
                    REQUIRE(reply.path.empty() == expected.empty());
                    if (not expected.empty())
                    {
                        ++found;
                        REQUIRE_THAT(cost(reply.path), Catch::Matchers::WithinRel(cost(expected), 1e-4F));
                        REQUIRE(valid_path(reply.path, pairs[i], environment));
                    }
                }
            }

            // The comparison means something only if most queries have a path
            REQUIRE(found > 30);
            const auto stats = service.stats();
            REQUIRE(stats.requests == 60);
            REQUIRE(stats.updates == 3);
        }
    }
}

TEST_CASE("Service answers the trivial, unknown, erased and overloaded cases", "[service]")
{
    const auto roadmap = free_space_roadmap(500);

    {
        Service service(roadmap, options(2));
        REQUIRE(service.set_scene("free", Environment()).get() == 1);
        const auto direct = service.plan("free", start(), goal()).get();
        REQUIRE(direct.status == coven::PlanStatus::ok);
        REQUIRE(direct.path.size() == 2);
        REQUIRE(direct.counters.rounds == 2);  // one to check the straight line, one to answer

        REQUIRE(service.plan("nowhere", start(), goal()).get().status == coven::PlanStatus::unknown_scene);
        REQUIRE(service.erase_scene("free").get());
        REQUIRE_FALSE(service.erase_scene("free").get());
        REQUIRE(service.plan("free", start(), goal()).get().status == coven::PlanStatus::unknown_scene);

        // A sphere that swallows the robot: no path
        vamp::collision::Environment<float> blocked;
        blocked.spheres.emplace_back(vamp::collision::factory::sphere::flat(0, 0, 0.5, 2.0));
        service.set_scene("blocked", Environment(blocked)).get();
        const auto none = service.plan("blocked", start(), goal()).get();
        REQUIRE(none.status == coven::PlanStatus::ok);
        REQUIRE(none.path.empty());
    }

    auto refusing = options(1);
    refusing.max_in_flight = 0;
    Service service(roadmap, refusing);
    service.set_scene("free", Environment()).get();
    REQUIRE(service.plan("free", start(), goal()).get().status == coven::PlanStatus::overloaded);
}

TEST_CASE("A reply's path is valid in the scene version it names", "[service]")
{
    // Built in the cage, so start and goal stay connected after a small move
    const auto roadmap = PRM::build_roadmap(
        start(), goal(), sphere_cage(), settings(3000), std::make_shared<vamp::rng::Halton<Robot>>());
    const auto scene = [](std::uint64_t version) { return sphere_cage(version == 1 ? 0.F : 0.05F); };

    Service service(roadmap, options(2));
    REQUIRE(service.set_scene("cage", scene(1)).get() == 1);

    // The update may land while the first request is in flight; the second is sent after it
    auto first = service.plan("cage", start(), goal());
    auto version = service.set_scene("cage", scene(2));
    auto second = service.plan("cage", start(), goal());
    REQUIRE(version.get() == 2);

    for (auto *reply : {&first, &second})
    {
        const auto result = reply->get();
        REQUIRE(result.status == coven::PlanStatus::ok);
        REQUIRE(result.path.size() > 2);
        REQUIRE(valid_path(result.path, {start(), goal()}, scene(result.version)));
        if (reply == &second)
        {
            REQUIRE(result.version == 2);
        }
    }
}

TEST_CASE("Only the last update of a scene in a batch is applied, before its requests", "[service]")
{
    const auto roadmap = free_space_roadmap(500);

    // The window holds everything in one batch
    Service service(roadmap, options(2, 50ms));
    auto first = service.set_scene("scene", sphere_cage(0.2F));
    auto second = service.set_scene("scene", sphere_cage());
    auto reply = service.plan("scene", start(), goal());
    auto third = service.set_scene("scene", Environment());

    REQUIRE(first.get() == 1);
    REQUIRE(second.get() == 1);
    REQUIRE(third.get() == 1);
    const auto result = reply.get();
    REQUIRE(result.version == 1);
    REQUIRE(result.path.size() == 2);  // planned in the last update, free space
    REQUIRE(service.stats().updates == 1);
}

TEST_CASE("Requests in flight are answered before the service stops", "[service]")
{
    const auto roadmap = free_space_roadmap(800);
    const auto environment = sphere_cage();
    const auto pairs = valid_pairs(environment, 16);
    std::vector<std::future<Service::Reply>> replies;
    {
        Service service(roadmap, options(2));
        service.set_scene("cage", environment);
        for (const auto &pair : pairs)
        {
            replies.push_back(service.plan("cage", pair.first, pair.second));
        }
    }

    for (auto &reply : replies)
    {
        REQUIRE(reply.get().status == coven::PlanStatus::ok);
    }
}

// Small on purpose: this is the case to run under -fsanitize=thread.
TEST_CASE("Service with concurrent clients and scene updates", "[service][race]")
{
    const auto roadmap = free_space_roadmap(600);
    const std::vector<std::string> scenes{"a", "b", "c", "d"};

    // Each scene's obstacles at each version; versions count set_scene calls, so they are known ahead
    const auto shift = [](std::size_t scene, std::uint64_t version)
    { return 0.01F * static_cast<float>((version + scene) % 5); };
    constexpr std::uint64_t versions = 6;

    Service service(roadmap, options(4, 200us));
    for (std::size_t s = 0; s < scenes.size(); ++s)
    {
        service.set_scene(scenes[s], sphere_cage(shift(s, 1))).get();
    }

    struct Answer
    {
        std::size_t scene;
        Pair pair;
        Service::Reply reply;
    };

    const auto pairs = valid_pairs(sphere_cage(), 8);
    constexpr std::size_t clients = 8;
    constexpr std::size_t per_client = 20;
    std::vector<std::vector<Answer>> answers(clients);
    std::vector<std::thread> threads;
    for (std::size_t c = 0; c < clients; ++c)
    {
        threads.emplace_back(
            [&, c]()
            {
                for (std::size_t i = 0; i < per_client; ++i)
                {
                    const auto scene = (c + i) % scenes.size();
                    const auto &pair = pairs[(c * per_client + i) % pairs.size()];
                    answers[c].push_back({scene, pair, service.plan(scenes[scene], pair.first, pair.second).get()});
                }
            });
    }

    for (std::uint64_t version = 2; version <= versions; ++version)
    {
        for (std::size_t s = 0; s < scenes.size(); ++s)
        {
            REQUIRE(service.set_scene(scenes[s], sphere_cage(shift(s, version))).get() == version);
        }

        std::this_thread::sleep_for(2ms);
    }

    for (auto &thread : threads)
    {
        thread.join();
    }

    std::size_t found = 0;
    for (const auto &client : answers)
    {
        for (const auto &answer : client)
        {
            REQUIRE(answer.reply.status == coven::PlanStatus::ok);
            REQUIRE(answer.reply.version >= 1);
            REQUIRE(answer.reply.version <= versions);
            if (not answer.reply.path.empty())
            {
                ++found;
                REQUIRE(valid_path(
                    answer.reply.path, answer.pair, sphere_cage(shift(answer.scene, answer.reply.version))));
            }
        }
    }

    REQUIRE(found > 0);
}
