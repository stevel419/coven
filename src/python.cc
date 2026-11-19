// coven._core: Python bindings for the parallel roadmap planner.
//
// The arguments and results that are VAMP types (vamp.Environment, vamp.PRMSettings, and each robot's
// Roadmap, Path and RNG) are the classes VAMP's own extension registers. Both extensions are built
// with the same nanobind, so they share one type registry; import vamp before coven._core.

#include <cstdint>
#include <cstring>
#include <string>

#include <omp.h>

#include <coven/pprm.hh>
#include <vamp/random/rng.hh>
#include <vamp/robots/baxter.hh>
#include <vamp/robots/fetch.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/sphere.hh>
#include <vamp/robots/ur5.hh>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace
{
    constexpr std::size_t rake = vamp::FloatVectorWidth;
    using EnvironmentInput = vamp::collision::Environment<float>;
    using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
    using Settings = vamp::planning::RoadmapSettings<vamp::planning::PRMStarNeighborParams>;

    // Copy n * columns values into a new numpy array
    template <typename T>
    auto numpy_copy(const void *data, std::size_t n, std::size_t columns)
    {
        auto *copy = new T[n * columns];
        std::memcpy(copy, data, n * columns * sizeof(T));
        nb::capsule owner(copy, [](void *a) noexcept { delete[] reinterpret_cast<T *>(a); });
        return columns == 1 ? nb::ndarray<nb::numpy, T>(copy, {n}, owner) :
                              nb::ndarray<nb::numpy, T>(copy, {n, columns}, owner);
    }

    template <typename Robot>
    void bind_robot(nb::module_ &parent, const char *name)
    {
        using PPRM = coven::PPRM<Robot, rake, Robot::resolution>;
        using Roadmap = vamp::planning::Roadmap<Robot>;
        using Input = nb::ndarray<const float, nb::shape<Robot::dimension>, nb::device::cpu>;
        using RNG = typename vamp::rng::RNG<Robot>::Ptr;

        const auto configuration = [](const Input &a)
        {
            typename Robot::ConfigurationArray c;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                c[i] = a(i);
            }

            return typename Robot::Configuration(c);
        };

        auto m = parent.def_submodule(name, "Parallel roadmap planning for this robot.");

        m.def(
            "build_roadmap",
            [configuration](
                const Input &start,
                const Input &goal,
                const EnvironmentInput &environment,
                Settings settings,
                RNG rng,
                std::size_t batch_size)
            {
                const auto start_v = configuration(start);
                const auto goal_v = configuration(goal);
                const EnvironmentVector environment_v(environment);
                settings.batch_size = batch_size;
                nb::gil_scoped_release release;
                return PPRM::build_roadmap(start_v, goal_v, environment_v, settings, rng);
            },
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "rng"_a,
            "batch_size"_a = 1024,
            "PRM roadmap with batched, multi-threaded collision checking. The same roadmap as "
            "vamp.<robot>.roadmap() with the same arguments, for any batch size and thread count.");

        m.def(
            "graph",
            [](const Roadmap &roadmap) { return PPRM::graph(roadmap); },
            "roadmap"_a,
            "Edges of a roadmap, each undirected edge once.");

        m.def(
            "revalidate",
            [](const Roadmap &roadmap, const coven::RoadmapGraph &graph, const EnvironmentInput &environment)
            {
                const EnvironmentVector environment_v(environment);
                nb::gil_scoped_release release;
                return PPRM::revalidate(roadmap, graph, environment_v);
            },
            "roadmap"_a,
            "graph"_a,
            "environment"_a,
            "Recheck every vertex and edge of a roadmap against an environment, multi-threaded.");

        m.def(
            "query",
            [configuration](
                const Roadmap &roadmap,
                const coven::RoadmapGraph &graph,
                const coven::RoadmapValidity &validity,
                const Input &start,
                const Input &goal,
                const EnvironmentInput &environment)
            {
                const auto start_v = configuration(start);
                const auto goal_v = configuration(goal);
                const EnvironmentVector environment_v(environment);
                nb::gil_scoped_release release;
                return PPRM::query(roadmap, graph, validity, start_v, goal_v, environment_v);
            },
            "roadmap"_a,
            "graph"_a,
            "validity"_a,
            "start"_a,
            "goal"_a,
            "environment"_a,
            "Shortest path from start to goal over the valid part of a roadmap. Empty if there is none.");

        m.def(
            "vertices",
            [](const Roadmap &roadmap)
            {
                const std::size_t n = roadmap.vertices.size();
                auto *data = new float[n * Robot::dimension + Robot::Configuration::num_scalars_rounded];
                for (std::size_t i = 0; i < n; ++i)
                {
                    roadmap.vertices[i].to_array_unaligned(data + i * Robot::dimension);
                }

                nb::capsule owner(data, [](void *a) noexcept { delete[] reinterpret_cast<float *>(a); });
                return nb::ndarray<nb::numpy, float>(data, {n, Robot::dimension}, owner);
            },
            "roadmap"_a,
            nb::rv_policy::move,
            "Vertices of a roadmap as an (n, dimension) numpy matrix.");
    }
}  // namespace

NB_MODULE(_core, m)
{
    nb::module_::import_("vamp");

    nb::class_<coven::RoadmapGraph>(m, "RoadmapGraph", "Edges of a roadmap, each undirected edge once.")
        .def(
            "__len__", [](const coven::RoadmapGraph &g) { return g.edges.size(); }, "Number of edges.")
        .def_prop_ro(
            "edges",
            [](const coven::RoadmapGraph &g)
            { return numpy_copy<std::uint32_t>(g.edges.data(), g.edges.size(), 2); },
            nb::rv_policy::move,
            "Vertex index pairs (i, j) with i < j, as an (n, 2) numpy matrix.");

    nb::class_<coven::RoadmapValidity>(
        m, "RoadmapValidity", "Which vertices and edges of a roadmap are collision-free.")
        .def_prop_ro(
            "vertices",
            [](const coven::RoadmapValidity &v)
            { return numpy_copy<bool>(v.vertices.data(), v.vertices.size(), 1); },
            nb::rv_policy::move,
            "One boolean per roadmap vertex.")
        .def_prop_ro(
            "edges",
            [](const coven::RoadmapValidity &v)
            { return numpy_copy<bool>(v.edges.data(), v.edges.size(), 1); },
            nb::rv_policy::move,
            "One boolean per edge of the RoadmapGraph.");

    m.def(
        "max_threads",
        []() { return omp_get_max_threads(); },
        "Number of threads the roadmap functions use.");

    m.def(
        "set_num_threads",
        [](int threads) { omp_set_num_threads(threads); },
        "threads"_a,
        "Set the number of threads the roadmap functions use, for the calling thread.");

    bind_robot<vamp::robots::Sphere>(m, "sphere");
    bind_robot<vamp::robots::UR5>(m, "ur5");
    bind_robot<vamp::robots::Panda>(m, "panda");
    bind_robot<vamp::robots::Fetch>(m, "fetch");
    bind_robot<vamp::robots::Baxter>(m, "baxter");
}
