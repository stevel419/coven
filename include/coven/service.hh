#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

#include <coven/pprm.hh>

#if defined(_OPENMP)
#define COVEN_SERVICE_OMP(directive) _Pragma(#directive)
#else
#define COVEN_SERVICE_OMP(directive)
#endif

namespace coven
{
    struct ServiceOptions
    {
        int threads = 1;                      // OpenMP threads for searching and collision checking
        std::chrono::microseconds window{0};  // How long an idle service collects requests before it starts
        bool eager = false;                   // Check every edge of a scene when it changes
        std::size_t max_in_flight = 65536;    // Requests queued or being planned; more are refused
        std::size_t max_rounds = 64;          // Rounds before a request has its whole scene checked
        std::size_t connections = 64;         // Roadmap vertices tried for the start and for the goal
    };

    enum class PlanStatus : std::uint8_t
    {
        ok,             // path holds the answer, empty when there is no path
        unknown_scene,  // no scene with that id
        overloaded,     // max_in_flight requests were already queued or being planned
        shut_down,      // the service is stopping
    };

    struct PlanCounters
    {
        std::uint32_t rounds{0};   // batches the request took part in
        std::uint32_t searches{0};  // shortest-route searches it ran
        std::uint32_t checks{0};   // collision checks it added to those batches
        bool fallback{false};      // reached max_rounds and had its whole scene checked
        double queued_seconds{0};  // from plan() until its first batch
        double total_seconds{0};   // from plan() until the reply
    };

    struct ServiceStats
    {
        std::uint64_t requests{0};  // plan requests taken into a batch
        std::uint64_t updates{0};   // set_scene calls applied
        std::uint64_t rounds{0};    // batches run
        std::uint64_t checks{0};    // collision checks run in batches (motions, not scene updates)
        double update_seconds{0};   // dispatcher time spent on scene updates
        double search_seconds{0};   // ... on nearest-vertex scans and route searches
        double check_seconds{0};    // ... on batched collision checks
    };

    // A planning service: one roadmap, any number of named scenes (obstacle sets), and many concurrent
    // plan requests. One dispatcher thread owns all scene state and runs requests in rounds. Each round
    // finds, for every active request, the shortest route through what is not known to be blocked;
    // collects the unchecked edges and connections on those routes; checks them all in one parallel loop
    // (an edge two requests need is checked once); and answers the requests whose route is now fully
    // valid. This is Lazy PRM (Bohlin and Kavraki, 2000) with checks batched across requests. Paths have
    // the same cost as PPRM::query() on a fully revalidated scene.
    //
    // A request plans in the newest version of its scene when it is taken into a batch (updates in the
    // same batch are applied first), and its reply names that version: later updates never restart it. What a version's requests
    // learn about its edges is shared until the scene changes. Every vertex is checked when a scene
    // changes (one pose each, cheap); edges are checked lazily, or all at once with ServiceOptions::eager.
    template <typename Robot, std::size_t rake, std::size_t resolution>
    class Service
    {
    public:
        using PPRM = coven::PPRM<Robot, rake, resolution>;
        using Configuration = typename Robot::Configuration;
        using Environment = typename PPRM::Environment;
        using Roadmap = vamp::planning::Roadmap<Robot>;
        using Path = vamp::planning::Path<Robot>;
        using Clock = std::chrono::steady_clock;

        struct Reply
        {
            PlanStatus status{PlanStatus::ok};
            Path path;                 // start, roadmap vertices, goal; empty when there is no path
            std::uint64_t version{0};  // the scene version the path is collision-free in
            PlanCounters counters;
        };

        Service(Roadmap roadmap, ServiceOptions options)
          : roadmap_(std::move(roadmap)), graph_(PPRM::graph(roadmap_)), options_(options)
        {
            dispatcher_ = std::thread([this]() { run(); });
        }

        Service(const Service &) = delete;
        auto operator=(const Service &) -> Service & = delete;

        // Answers what is already queued, refuses new work, then stops.
        ~Service()
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                stopping_ = true;
            }

            wake_.notify_one();
            dispatcher_.join();
        }

        // Create or replace a scene's obstacles. The future holds the scene's new version (1 for a new
        // scene, 0 when the service is stopping). Of several updates of one scene that arrive together,
        // only the last is applied, and every one's future holds its version.
        auto set_scene(std::string id, Environment environment) -> std::future<std::uint64_t>
        {
            SetScene work{std::move(id), std::move(environment), {}};
            auto done = work.done.get_future();
            if (not enqueue(work))
            {
                work.done.set_value(0);
            }

            return done;
        }

        // Remove a scene; requests already in flight on it still finish. False if there was none.
        auto erase_scene(std::string id) -> std::future<bool>
        {
            EraseScene work{std::move(id), {}};
            auto done = work.done.get_future();
            if (not enqueue(work))
            {
                work.done.set_value(false);
            }

            return done;
        }

        auto plan(std::string scene, const Configuration &start, const Configuration &goal)
            -> std::future<Reply>
        {
            Plan work{std::move(scene), start, goal, {}, Clock::now()};
            auto reply = work.reply.get_future();
            if (in_flight_.fetch_add(1) >= options_.max_in_flight)
            {
                answer(work, Reply{PlanStatus::overloaded, {}, 0, {}});
            }
            else if (not enqueue(work))
            {
                answer(work, Reply{PlanStatus::shut_down, {}, 0, {}});
            }

            return reply;
        }

        auto stats() const -> ServiceStats
        {
            return {
                requests_.load(),
                updates_.load(),
                rounds_.load(),
                checks_.load(),
                1e-9 * static_cast<double>(update_ns_.load()),
                1e-9 * static_cast<double>(search_ns_.load()),
                1e-9 * static_cast<double>(check_ns_.load())};
        }

        auto options() const -> const ServiceOptions &
        {
            return options_;
        }

        auto roadmap() const -> const Roadmap &
        {
            return roadmap_;
        }

    private:
        // ponytail: fixed thresholds from laptop measurements; tune per machine if small-load latency matters
        static constexpr std::size_t min_parallel_searches = 2;
        static constexpr std::size_t min_parallel_checks = 64;

        enum State : std::uint8_t
        {
            unknown = 0,
            valid = 1,
            blocked = 2,
            pending = 3,  // being checked in the current round
        };

        // One version of a scene
        struct Scene
        {
            Environment environment;
            std::uint64_t version{0};
            std::vector<std::uint8_t> vertices;  // State per roadmap vertex, always known
            std::vector<std::uint8_t> edges;     // State per RoadmapGraph edge
        };

        struct SetScene
        {
            std::string id;
            Environment environment;
            std::promise<std::uint64_t> done;
            bool skip{false};  // replaced by a later update in the same batch
        };

        struct EraseScene
        {
            std::string id;
            std::promise<bool> done;
        };

        struct Plan
        {
            std::string scene;
            Configuration start;
            Configuration goal;
            std::promise<Reply> reply;
            Clock::time_point arrival;
        };

        using Work = std::variant<SetScene, EraseScene, Plan>;

        struct Request
        {
            Plan work;
            std::shared_ptr<Scene> scene;  // the version it plans in
            PlanCounters counters;
            std::uint8_t direct{unknown};
            std::vector<typename PPRM::Link> starts;
            std::vector<typename PPRM::Link> goals;
            std::vector<std::uint8_t> start_states;
            std::vector<std::uint8_t> goal_states;
            typename PPRM::Route route;
            std::size_t first_link{0};  // indices into starts and goals of the route's ends
            std::size_t last_link{0};
        };

        // One motion to check; the answer goes to *out
        struct Check
        {
            const Configuration *from;
            const Configuration *to;
            const Environment *environment;
            std::uint8_t *out;
        };

        static auto seconds(Clock::duration d) -> double
        {
            return std::chrono::duration<double>(d).count();
        }

        static auto nanoseconds(Clock::duration d) -> std::uint64_t
        {
            return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(d).count());
        }

        template <typename W>
        auto enqueue(W &work) -> bool
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (stopping_)
                {
                    return false;
                }

                if (queue_.empty())
                {
                    first_arrival_ = Clock::now();
                }

                queue_.emplace_back(std::move(work));
            }

            wake_.notify_one();
            return true;
        }

        void answer(Plan &work, Reply reply)
        {
            reply.counters.total_seconds = seconds(Clock::now() - work.arrival);
            work.reply.set_value(std::move(reply));
            --in_flight_;
        }

        void answer(Request &request, Path path)
        {
            answer(request.work, Reply{PlanStatus::ok, std::move(path), request.scene->version, request.counters});
        }

        void run()
        {
#if defined(_OPENMP)
            omp_set_num_threads(options_.threads);
#endif
            std::vector<Work> batch;
            for (;;)
            {
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    if (active_.empty())
                    {
                        wake_.wait(lock, [this]() { return stopping_ or not queue_.empty(); });
                        if (queue_.empty())
                        {
                            return;
                        }

                        // Idle: let a batch build up. Busy: requests join the next round directly.
                        if (options_.window.count() > 0)
                        {
                            const auto deadline = first_arrival_ + options_.window;
                            lock.unlock();
                            std::this_thread::sleep_until(deadline);
                            lock.lock();
                        }
                    }

                    batch.swap(queue_);
                }

                // Scene changes first, so new requests plan in the newest versions
                skip_superseded(batch);
                for (auto &work : batch)
                {
                    if (not std::holds_alternative<Plan>(work))
                    {
                        std::visit([this](auto &w) { apply(w); }, work);
                    }
                }

                for (auto &work : batch)
                {
                    if (auto *request = std::get_if<Plan>(&work))
                    {
                        apply(*request);
                    }
                }

                batch.clear();
                if (not active_.empty())
                {
                    round();
                }
            }
        }

        // Only the last update of each scene in a batch is applied; the others are never seen by any
        // request, and their callers get the later version. Without this, scenes that change faster than
        // they are checked would build an endless backlog.
        void skip_superseded(std::vector<Work> &batch)
        {
            std::unordered_map<std::string, bool> later;  // a later update of this scene is in the batch
            for (auto it = batch.rbegin(); it != batch.rend(); ++it)
            {
                if (auto *update = std::get_if<SetScene>(&*it))
                {
                    update->skip = later[update->id];
                    later[update->id] = true;
                }
            }
        }

        void apply(SetScene &work)
        {
            if (work.skip)
            {
                skipped_[work.id].push_back(std::move(work.done));
                return;
            }

            auto &current = scenes_[work.id];
            auto scene = std::make_shared<Scene>();
            scene->environment = std::move(work.environment);
            scene->version = current ? current->version + 1 : 1;
            const auto begin = Clock::now();
            check_scene(*scene);
            update_ns_ += nanoseconds(Clock::now() - begin);
            current = std::move(scene);
            ++updates_;
            work.done.set_value(current->version);

            const auto skipped = skipped_.find(work.id);
            if (skipped != skipped_.end())
            {
                for (auto &done : skipped->second)
                {
                    done.set_value(current->version);
                }

                skipped_.erase(skipped);
            }
        }

        void apply(EraseScene &work)
        {
            work.done.set_value(scenes_.erase(work.id) > 0);
        }

        void apply(Plan &work)
        {
            const auto it = scenes_.find(work.scene);
            if (it == scenes_.end())
            {
                answer(work, Reply{PlanStatus::unknown_scene, {}, 0, {}});
                return;
            }

            auto request = std::make_unique<Request>();
            request->work = std::move(work);
            request->scene = it->second;
            request->counters.queued_seconds = seconds(Clock::now() - request->work.arrival);
            fresh_.push_back(request.get());
            active_.push_back(std::move(request));
            ++requests_;
        }

        // A new scene version: check every vertex, and every edge when eager
        void check_scene(Scene &scene)
        {
            const std::size_t n = roadmap_.vertices.size();
            scene.vertices.resize(n);
            scene.edges.assign(graph_.edges.size(), unknown);
            if (options_.eager)
            {
                const auto validity = PPRM::revalidate(roadmap_, graph_, scene.environment);
                for (std::size_t v = 0; v < n; ++v)
                {
                    scene.vertices[v] = validity.vertices[v] ? valid : blocked;
                }

                for (std::size_t e = 0; e < scene.edges.size(); ++e)
                {
                    scene.edges[e] = validity.edges[e] ? valid : blocked;
                }

                return;
            }

            const bool parallel = not scene.environment.attachments;
            COVEN_SERVICE_OMP(omp parallel for schedule(dynamic, 64) if (parallel))
            for (std::size_t v = 0; v < n; ++v)
            {
                const auto &vertex = roadmap_.vertices[v];
                scene.vertices[v] =
                    vamp::planning::validate_motion<Robot, rake, 1>(vertex, vertex, scene.environment) ? valid :
                                                                                                         blocked;
            }
        }

        void round()
        {
            ++rounds_;
            checks_round_.clear();
            parallel_ = true;

            // New requests connect start and goal to the nearest valid vertices, as in PPRM::query(),
            // and check the straight line and all those connections at once
            const auto begin = Clock::now();
            const std::size_t n_fresh = fresh_.size();
            COVEN_SERVICE_OMP(omp parallel for schedule(dynamic) if (n_fresh >= min_parallel_searches))
            for (std::size_t i = 0; i < n_fresh; ++i)
            {
                connect(*fresh_[i]);
            }

            for (auto *request : fresh_)
            {
                add(*request, request->work.start, request->work.goal, request->direct);
                for (std::size_t s = 0; s < request->starts.size(); ++s)
                {
                    const auto &vertex = roadmap_.vertices[request->starts[s].second];
                    add(*request, request->work.start, vertex, request->start_states[s]);
                }

                for (std::size_t g = 0; g < request->goals.size(); ++g)
                {
                    const auto &vertex = roadmap_.vertices[request->goals[g].second];
                    add(*request, vertex, request->work.goal, request->goal_states[g]);
                }
            }

            fresh_.clear();

            // Small rounds run on the dispatcher alone: waking the other threads costs more than the work
            const std::size_t n_active = active_.size();
            COVEN_SERVICE_OMP(omp parallel for schedule(dynamic) if (n_active >= min_parallel_searches))
            for (std::size_t i = 0; i < n_active; ++i)
            {
                search(*active_[i]);
            }

            const auto searched = Clock::now();
            search_ns_ += nanoseconds(searched - begin);

            std::vector<std::unique_ptr<Request>> waiting;
            for (auto &request : active_)
            {
                ++request->counters.rounds;
                if (collect(*request))
                {
                    waiting.push_back(std::move(request));
                }
            }

            const auto collected = Clock::now();
            const std::size_t n_checks = checks_round_.size();
            COVEN_SERVICE_OMP(omp parallel for schedule(dynamic) if (parallel_ and n_checks >= min_parallel_checks))
            for (std::size_t i = 0; i < n_checks; ++i)
            {
                const auto &check = checks_round_[i];
                *check.out = vamp::planning::validate_motion<Robot, rake, resolution>(
                                 *check.from, *check.to, *check.environment) ?
                                 valid :
                                 blocked;
            }

            checks_ += n_checks;
            check_ns_ += nanoseconds(Clock::now() - collected);
            active_.swap(waiting);
        }

        // Reads scene state, writes only the request
        void connect(Request &request) const
        {
            const auto &scene = *request.scene;
            const auto usable = [&](std::uint32_t v) { return scene.vertices[v] == valid; };
            request.starts = PPRM::nearest(roadmap_, request.work.start, options_.connections, usable);
            request.goals = PPRM::nearest(roadmap_, request.work.goal, options_.connections, usable);
            request.start_states.assign(request.starts.size(), unknown);
            request.goal_states.assign(request.goals.size(), unknown);
        }

        // Find a new shortest route through what is not known to be blocked, unless the current one is
        // still open. Reads scene state, writes only the request.
        void search(Request &request) const
        {
            const auto &scene = *request.scene;
            auto &route = request.route;
            const auto open_edge = [&](std::uint32_t e)
            {
                const auto [i, j] = graph_.edges[e];
                return scene.edges[e] != blocked and scene.vertices[i] == valid and scene.vertices[j] == valid;
            };

            if (request.direct == valid)
            {
                return;
            }

            if (not route.vertices.empty() and request.start_states[request.first_link] != blocked and
                request.goal_states[request.last_link] != blocked and
                std::all_of(route.edges.begin(), route.edges.end(), open_edge))
            {
                return;
            }

            const auto open = [](const std::vector<typename PPRM::Link> &links,
                                 const std::vector<std::uint8_t> &states)
            {
                std::vector<typename PPRM::Link> result;
                for (std::size_t i = 0; i < links.size(); ++i)
                {
                    if (states[i] != blocked)
                    {
                        result.push_back(links[i]);
                    }
                }

                return result;
            };

            ++request.counters.searches;
            route = PPRM::search(
                roadmap_,
                graph_,
                open(request.starts, request.start_states),
                open(request.goals, request.goal_states),
                request.work.goal,
                open_edge);

            if (not route.vertices.empty())
            {
                const auto index = [](const std::vector<typename PPRM::Link> &links, std::uint32_t vertex)
                {
                    std::size_t i = 0;
                    while (links[i].second != vertex)
                    {
                        ++i;
                    }

                    return i;
                };

                request.first_link = index(request.starts, route.vertices.front());
                request.last_link = index(request.goals, route.vertices.back());
            }
        }

        void add(Request &request, const Configuration &from, const Configuration &to, std::uint8_t &out)
        {
            out = pending;
            checks_round_.push_back({&from, &to, &request.scene->environment, &out});
            parallel_ = parallel_ and not request.scene->environment.attachments;
            ++request.counters.checks;
        }

        // Queue what the request's route still needs checked. Answers the request and returns false when
        // nothing is left to check.
        auto collect(Request &request) -> bool
        {
            auto &scene = *request.scene;
            const auto &start = request.work.start;
            const auto &goal = request.work.goal;

            // The straight line wins whenever it is valid, as in PPRM::query()
            if (request.direct == valid)
            {
                Path path;
                path.emplace_back(start);
                path.emplace_back(goal);
                answer(request, std::move(path));
                return false;
            }

            bool outstanding = request.direct == pending;
            const auto &route = request.route;
            if (route.vertices.empty())
            {
                if (not outstanding)
                {
                    answer(request, {});
                }

                return outstanding;
            }

            outstanding = outstanding or request.start_states[request.first_link] == pending or
                          request.goal_states[request.last_link] == pending;
            for (const auto e : route.edges)
            {
                if (scene.edges[e] == unknown)
                {
                    const auto [i, j] = graph_.edges[e];
                    add(request, roadmap_.vertices[i], roadmap_.vertices[j], scene.edges[e]);
                }

                outstanding = outstanding or scene.edges[e] == pending;
            }

            if (not outstanding)
            {
                Path path;
                path.emplace_back(start);
                for (const auto v : route.vertices)
                {
                    path.emplace_back(roadmap_.vertices[v]);
                }

                path.emplace_back(goal);
                answer(request, std::move(path));
                return false;
            }

            // Too many rounds: check every edge this request could still use, so its next search is final
            if (not request.counters.fallback and request.counters.rounds >= options_.max_rounds)
            {
                request.counters.fallback = true;
                for (std::size_t e = 0; e < scene.edges.size(); ++e)
                {
                    const auto [i, j] = graph_.edges[e];
                    if (scene.edges[e] == unknown and scene.vertices[i] == valid and scene.vertices[j] == valid)
                    {
                        add(request, roadmap_.vertices[i], roadmap_.vertices[j], scene.edges[e]);
                    }
                }
            }

            return true;
        }

        const Roadmap roadmap_;
        const RoadmapGraph graph_;
        const ServiceOptions options_;

        std::mutex mutex_;
        std::condition_variable wake_;
        std::vector<Work> queue_;
        Clock::time_point first_arrival_;
        bool stopping_{false};
        std::atomic<std::size_t> in_flight_{0};

        // Owned by the dispatcher thread
        std::unordered_map<std::string, std::shared_ptr<Scene>> scenes_;  // the current version of each
        std::vector<std::unique_ptr<Request>> active_;
        std::vector<Request *> fresh_;  // admitted since the last round
        std::unordered_map<std::string, std::vector<std::promise<std::uint64_t>>> skipped_;
        std::vector<Check> checks_round_;
        bool parallel_{true};

        std::atomic<std::uint64_t> requests_{0};
        std::atomic<std::uint64_t> updates_{0};
        std::atomic<std::uint64_t> rounds_{0};
        std::atomic<std::uint64_t> checks_{0};
        std::atomic<std::uint64_t> update_ns_{0};
        std::atomic<std::uint64_t> search_ns_{0};
        std::atomic<std::uint64_t> check_ns_{0};

        std::thread dispatcher_;
    };
}  // namespace coven

#undef COVEN_SERVICE_OMP
