#include "ModelMusclePlotPanel.hpp"

#include "src/Actions/ActionFunctions.hpp"
#include "src/Bindings/ImGuiHelpers.hpp"
#include "src/OpenSimBindings/ModelStateCommit.hpp"
#include "src/OpenSimBindings/OpenSimHelpers.hpp"
#include "src/OpenSimBindings/UndoableModelStatePair.hpp"
#include "src/Platform/App.hpp"
#include "src/Platform/Log.hpp"
#include "src/Utils/Assertions.hpp"
#include "src/Utils/Algorithms.hpp"
#include "src/Utils/CStringView.hpp"
#include "src/Utils/Cpp20Shims.hpp"
#include "src/Utils/SynchronizedValue.hpp"

#include <glm/glm.hpp>
#include <IconsFontAwesome5.h>
#include <imgui.h>
#include <implot.h>
#include <nonstd/span.hpp>
#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/ComponentList.h>
#include <OpenSim/Common/ComponentPath.h>
#include <OpenSim/Common/PropertyObjArray.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string_view>
#include <sstream>
#include <type_traits>
#include <utility>
#include <vector>

namespace SimTK { class State; }

// muscle outputs
//
// wraps OpenSim::Muscle member methods in a higher-level API that the UI
// can present to the user
namespace
{
    // describes a single output from an OpenSim::Muscle
    class MuscleOutput {
    public:
        MuscleOutput(char const* name, char const* units, double(*getter)(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const& c)) :
            m_Name{std::move(name)},
            m_Units{std::move(units)},
            m_Getter{std::move(getter)}
        {
        }

        char const* getName() const
        {
            return m_Name.c_str();
        }

        char const* getUnits() const
        {
            return m_Units.c_str();
        }

        double operator()(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const& c) const
        {
            return m_Getter(st, muscle, c);
        }

    private:
        friend bool operator<(MuscleOutput const&, MuscleOutput const&);
        friend bool operator==(MuscleOutput const&, MuscleOutput const&);
        friend bool operator!=(MuscleOutput const&, MuscleOutput const&);

        osc::CStringView m_Name;
        osc::CStringView m_Units;
        double(*m_Getter)(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const& c);
    };

    bool operator<(MuscleOutput const& a, MuscleOutput const& b)
    {
        return a.m_Name < b.m_Name;
    }

    bool operator==(MuscleOutput const& a, MuscleOutput const& b)
    {
        return a.m_Name == b.m_Name && a.m_Units == b.m_Units && a.m_Getter == b.m_Getter;
    }

    bool operator!=(MuscleOutput const& a, MuscleOutput const& b)
    {
        return !(a == b);
    }

    double GetMomentArm(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const& c)
    {
        return muscle.getGeometryPath().computeMomentArm(st, c);
    }

    double GetFiberLength(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getFiberLength(st);
    }

    double GetTendonLength(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getTendonLength(st);
    }

    double GetPennationAngle(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return glm::degrees(muscle.getPennationAngle(st));
    }

    double GetNormalizedFiberLength(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getNormalizedFiberLength(st);
    }

    double GetTendonStrain(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getTendonStrain(st);
    }

    double GetFiberPotentialEnergy(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getFiberPotentialEnergy(st);
    }

    double GetTendonPotentialEnergy(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getTendonPotentialEnergy(st);
    }

    double GetMusclePotentialEnergy(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getMusclePotentialEnergy(st);
    }

    double GetTendonForce(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getTendonForce(st);
    }

    double GetActiveFiberForce(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getActiveFiberForce(st);
    }

    double GetPassiveFiberForce(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getPassiveFiberForce(st);
    }

    double GetTotalFiberForce(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getFiberForce(st);
    }

    double GetFiberStiffness(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getFiberStiffness(st);
    }

    double GetFiberStiffnessAlongTendon(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getFiberStiffnessAlongTendon(st);
    }

    double GetTendonStiffness(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getTendonStiffness(st);
    }

    double GetMuscleStiffness(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getMuscleStiffness(st);
    }

    double GetFiberActivePower(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getFiberActivePower(st);
    }

    double GetFiberPassivePower(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getFiberActivePower(st);
    }

    double GetTendonPower(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getTendonPower(st);
    }

    double GetMusclePower(SimTK::State const& st, OpenSim::Muscle const& muscle, OpenSim::Coordinate const&)
    {
        return muscle.getTendonPower(st);
    }

    MuscleOutput GetDefaultMuscleOutput()
    {
        return MuscleOutput{"Moment Arm", "Unitless", GetMomentArm};
    }

    std::vector<MuscleOutput> GenerateMuscleOutputs()
    {
        std::vector<MuscleOutput> rv =
        {{
            GetDefaultMuscleOutput(),
            {"Tendon Length", "m", GetTendonLength},
            {"Fiber Length", "m", GetFiberLength},
            {"Pennation Angle", "deg", GetPennationAngle},
            {"Normalized Fiber Length", "Unitless", GetNormalizedFiberLength},
            {"Tendon Strain", "Unitless", GetTendonStrain},
            {"Fiber Potential Energy", "J", GetFiberPotentialEnergy},
            {"Tendon Potential Energy", "J", GetTendonPotentialEnergy},
            {"Muscle Potential Energy", "J", GetMusclePotentialEnergy},
            {"Tendon Force", "N", GetTendonForce},
            {"Active Fiber Force", "N", GetActiveFiberForce},
            {"Passive Fiber Force", "N", GetPassiveFiberForce},
            {"Total Fiber Force", "N", GetTotalFiberForce},
            {"Fiber Stiffness", "N/m", GetFiberStiffness},
            {"Fiber Stiffness Along Tendon", "N/m", GetFiberStiffnessAlongTendon},
            {"Tendon Stiffness", "N/m", GetTendonStiffness},
            {"Muscle Stiffness", "N/m", GetMuscleStiffness},
            {"Fiber Active Power", "W", GetFiberActivePower},
            {"Fiber Passive Power", "W", GetFiberPassivePower},
            {"Tendon Power", "W", GetTendonPower},
            {"Muscle Power", "W", GetMusclePower},
            }};
        std::sort(rv.begin(), rv.end());
        return rv;
    }
}

// backend datastructures
//
// these are the datastructures that the widget mostly plays around with
namespace
{
    // parameters for generating a plot line
    //
    // i.e. changing any part of the parameters may produce a different curve
    class PlotParameters final {
    public:
        PlotParameters(osc::ModelStateCommit commit,
            OpenSim::ComponentPath coordinatePath,
            OpenSim::ComponentPath musclePath,
            MuscleOutput output,
            int requestedNumDataPoints) :

            m_Commit{std::move(commit)},
            m_CoordinatePath{std::move(coordinatePath)},
            m_MusclePath{std::move(musclePath)},
            m_Output{std::move(output)},
            m_RequestedNumDataPoints{std::move(requestedNumDataPoints)}
        {
        }

        osc::ModelStateCommit const& getCommit() const
        {
            return m_Commit;
        }

        void setCommit(osc::ModelStateCommit const& commit)
        {
            m_Commit = commit;
        }

        OpenSim::ComponentPath const& getCoordinatePath() const
        {
            return m_CoordinatePath;
        }

        void setCoordinatePath(OpenSim::ComponentPath const& cp)
        {
            m_CoordinatePath = cp;
        }

        OpenSim::ComponentPath const& getMusclePath() const
        {
            return m_MusclePath;
        }

        void setMusclePath(OpenSim::ComponentPath const& cp)
        {
            m_MusclePath = cp;
        }

        MuscleOutput const& getMuscleOutput() const
        {
            return m_Output;
        }

        void setMuscleOutput(MuscleOutput const& output)
        {
            m_Output = output;
        }

        int getNumRequestedDataPoints() const
        {
            return m_RequestedNumDataPoints;
        }

        void setNumRequestedDataPoints(int v)
        {
            m_RequestedNumDataPoints = v;
        }

    private:
        friend bool operator==(PlotParameters const&, PlotParameters const&);
        friend bool operator!=(PlotParameters const&, PlotParameters const&);

        osc::ModelStateCommit m_Commit;
        OpenSim::ComponentPath m_CoordinatePath;
        OpenSim::ComponentPath m_MusclePath;
        MuscleOutput m_Output;
        int m_RequestedNumDataPoints;
    };

    bool operator==(PlotParameters const& a, PlotParameters const& b)
    {
        return
            a.m_Commit == b.m_Commit &&
            a.m_CoordinatePath == b.m_CoordinatePath &&
            a.m_MusclePath == b.m_MusclePath &&
            a.m_Output == b.m_Output &&
            a.m_RequestedNumDataPoints == b.m_RequestedNumDataPoints;
    }

    bool operator!=(PlotParameters const& a, PlotParameters const& b)
    {
        return !(a == b);
    }

    double GetFirstXValue(PlotParameters const& p, OpenSim::Coordinate const& c)
    {
        return c.getRangeMin();
    }

    double GetLastXValue(PlotParameters const& p, OpenSim::Coordinate const& c)
    {
        return c.getRangeMax();
    }

    double GetStepBetweenXValues(PlotParameters const& p, OpenSim::Coordinate const& c)
    {
        double start = GetFirstXValue(p, c);
        double end = GetLastXValue(p, c);

        return (end - start) / std::max(1, p.getNumRequestedDataPoints() - 1);
    }

    // a single data point in the plot, as emitted by a PlottingTask
    struct PlotDataPoint final {
        float x;
        float y;
    };

    // plot data points are naturally ordered by their independent (X) variable
    bool operator<(PlotDataPoint const& a, PlotDataPoint const& b)
    {
        return a.x < b.x;
    }

    // virtual interface to a thing that can receive datapoints from a plotter
    class PlotDataPointConsumer {
    protected:
        PlotDataPointConsumer() = default;
    public:
        PlotDataPointConsumer(PlotDataPointConsumer const&) = delete;
        PlotDataPointConsumer(PlotDataPointConsumer&&) noexcept = delete;
        PlotDataPointConsumer& operator=(PlotDataPointConsumer const&) = delete;
        PlotDataPointConsumer& operator=(PlotDataPointConsumer&&) noexcept = delete;
        virtual ~PlotDataPointConsumer() noexcept = default;

        virtual void operator()(PlotDataPoint) = 0;
    };

    // the status of a "live" plotting task
    enum class PlottingTaskStatus {
        Running,
        Cancelled,
        Finished,
        Error,
    };

    // mutable data that is shared between the plot worker thread and the top-level
    // plotting task
    class PlottingTaskThreadsafeSharedData final {
    public:
        PlottingTaskStatus getStatus() const
        {
            return m_Status.load();
        }

        std::optional<std::string> getErrorMessage() const
        {
            auto lock = m_ErrorMessage.lock();
            return *lock;
        }

        void setErrorMessage(std::string s)
        {
            auto lock = m_ErrorMessage.lock();
            *lock = std::move(s);
        }

        void setStatus(PlottingTaskStatus s)
        {
            m_Status = s;
        }

    private:
        std::atomic<PlottingTaskStatus> m_Status = PlottingTaskStatus::Running;
        osc::SynchronizedValue<std::string> m_ErrorMessage;
    };

    // all inputs to the plotting function
    struct PlottingTaskInputs final {
        PlottingTaskInputs(
            std::shared_ptr<PlottingTaskThreadsafeSharedData> shared_,
            PlotParameters const& plotParameters_,
            std::shared_ptr<PlotDataPointConsumer> dataPointConsumer_) :

            shared{ std::move(shared_) },
            plotParameters{ plotParameters_ },
            dataPointConsumer{ std::move(dataPointConsumer_) }
        {
        }

        std::shared_ptr<PlottingTaskThreadsafeSharedData> shared;
        PlotParameters plotParameters;
        std::shared_ptr<PlotDataPointConsumer> dataPointConsumer;
    };

    // inner (exception unsafe) plot function
    //
    // this is the function that actually does the "work" of computing plot points
    PlottingTaskStatus ComputePlotPointsUnguarded(osc::stop_token const& stopToken, PlottingTaskInputs& inputs)
    {
        PlottingTaskThreadsafeSharedData& shared = *inputs.shared;
        PlotParameters const& params = inputs.plotParameters;
        PlotDataPointConsumer& callback = *inputs.dataPointConsumer;

        if (params.getNumRequestedDataPoints() <= 0)
        {
            return PlottingTaskStatus::Finished;
        }

        // create a local copy of the model
        std::unique_ptr<OpenSim::Model> model = std::make_unique<OpenSim::Model>(*params.getCommit().getModel());

        if (stopToken.stop_requested())
        {
            return PlottingTaskStatus::Cancelled;
        }

        // init the model + state

        osc::InitializeModel(*model);

        if (stopToken.stop_requested())
        {
            return PlottingTaskStatus::Cancelled;
        }

        SimTK::State& state = osc::InitializeState(*model);

        if (stopToken.stop_requested())
        {
            return PlottingTaskStatus::Cancelled;
        }

        OpenSim::Muscle const* maybeMuscle = osc::FindComponent<OpenSim::Muscle>(*model, params.getMusclePath());
        if (!maybeMuscle)
        {
            shared.setErrorMessage(params.getMusclePath().toString() + ": cannot find a muscle with this name");
            return PlottingTaskStatus::Error;
        }
        OpenSim::Muscle const& muscle = *maybeMuscle;

        OpenSim::Coordinate const* maybeCoord = osc::FindComponentMut<OpenSim::Coordinate>(*model, params.getCoordinatePath());
        if (!maybeCoord)
        {
            shared.setErrorMessage(params.getCoordinatePath().toString() + ": cannot find a coordinate with this name");
            return PlottingTaskStatus::Error;
        }
        OpenSim::Coordinate const& coord = *maybeCoord;

        int const numDataPoints = params.getNumRequestedDataPoints();
        double const firstXValue = GetFirstXValue(params, coord);
        double const lastXValue = GetLastXValue(params, coord);
        double const stepBetweenXValues = GetStepBetweenXValues(params, coord);

        if (firstXValue > lastXValue)
        {
            // this invariant is necessary because other algorithms assume X increases over
            // the datapoint collection (e.g. for optimized binary searches, std::lower_bound etc.)

            shared.setErrorMessage(params.getCoordinatePath().toString() + ": cannot plot a coordinate with reversed min/max");
            return PlottingTaskStatus::Error;
        }

        // this fixes an unusual bug (#352), where the underlying assembly solver in the
        // model ends up retaining invalid values across a coordinate (un)lock, which makes
        // it sets coordinate values from X (what we want) to 0 after model assembly
        //
        // I don't exactly know *why* it's doing it - it looks like OpenSim holds a solver
        // internally that, itself, retains invalid coordinate values or something
        //
        // see #352 for a lengthier explanation
        coord.setLocked(state, false);
        model->updateAssemblyConditions(state);

        if (stopToken.stop_requested())
        {
            return PlottingTaskStatus::Cancelled;
        }

        for (int i = 0; i < numDataPoints; ++i)
        {
            if (stopToken.stop_requested())
            {
                return PlottingTaskStatus::Cancelled;
            }

            double xVal = firstXValue + (i * stepBetweenXValues);
            coord.setValue(state, xVal);

            model->equilibrateMuscles(state);

            if (stopToken.stop_requested())
            {
                return PlottingTaskStatus::Cancelled;
            }

            model->realizeReport(state);

            if (stopToken.stop_requested())
            {
                return PlottingTaskStatus::Cancelled;
            }

            float const xDisplayVal = osc::ConvertCoordValueToDisplayValue(coord, xVal);
            float const yVal = static_cast<float>(params.getMuscleOutput()(state, muscle, coord));

            callback(PlotDataPoint{xDisplayVal, yVal});
        }

        return PlottingTaskStatus::Finished;
    }

    // top-level "main" function that the Plotting task worker thread executes
    //
    // catches exceptions and propagates them to the task
    int ComputePlotPointsMain(osc::stop_token const& stopToken, PlottingTaskInputs inputs)
    {
        try
        {
            inputs.shared->setStatus(PlottingTaskStatus::Running);
            PlottingTaskStatus status = ComputePlotPointsUnguarded(stopToken, inputs);
            inputs.shared->setStatus(status);
            return 0;
        }
        catch (std::exception const& ex)
        {
            inputs.shared->setErrorMessage(ex.what());
            inputs.shared->setStatus(PlottingTaskStatus::Error);
            return -1;
        }
    }

    // a "live" plotting task that is being executed on a background thread
    //
    // the plotting task emits each plotpoint through the callback without any mutexes, so
    // it's up to the user of this class to ensure each emitted point is handled correctly
    class PlottingTask final {
    public:
        PlottingTask(PlotParameters const& params, std::shared_ptr<PlotDataPointConsumer> consumer_) :
            m_WorkerThread{ComputePlotPointsMain, PlottingTaskInputs{m_Shared, params, std::move(consumer_)}}
        {
        }

        PlottingTaskStatus getStatus() const
        {
            return m_Shared->getStatus();
        }

        std::optional<std::string> getErrorString() const
        {
            return m_Shared->getErrorMessage();
        }

    private:
        std::shared_ptr<PlottingTaskThreadsafeSharedData> m_Shared = std::make_shared<PlottingTaskThreadsafeSharedData>();
        osc::jthread m_WorkerThread;
    };

    // a data plot (line), potentially computed from a background thread, or loaded via a
    // file
    class Plot final : public PlotDataPointConsumer {
    public:
        explicit Plot(PlotParameters const& parameters) :
            m_Parameters{parameters}
        {
            m_DataPoints.lock()->reserve(m_Parameters.getNumRequestedDataPoints());
        }

        PlotParameters const& getParameters() const
        {
            return m_Parameters;
        }

        osc::SynchronizedValueGuard<std::vector<PlotDataPoint> const> lockDataPoints() const
        {
            return m_DataPoints.lock();
        }

        void operator()(PlotDataPoint p) override
        {
            {
                auto lock = m_DataPoints.lock();
                lock->push_back(p);
            }

            // HACK: something happened on a background thread, the UI thread should probably redraw
            osc::App::upd().requestRedraw();
        }

        bool getIsLocked() const
        {
            return m_IsLocked;
        }

        void setIsLocked(bool v)
        {
            m_IsLocked = std::move(v);
        }

    private:
        PlotParameters m_Parameters;
        bool m_IsLocked = false;
        osc::SynchronizedValue<std::vector<PlotDataPoint>> m_DataPoints;
    };
}

// helpers
//
// used for various UI tasks (e.g. finding the closest point for "snapping" and so on)
namespace
{
    float lerp(float a, float b, float t)
    {
        return (1.0f - t) * a + t * b;
    }

    std::optional<float> ComputeLERPedY(Plot const& p, float x)
    {
        auto lock = p.lockDataPoints();
        nonstd::span<PlotDataPoint const> const points = *lock;

        if (points.empty())
        {
            // there are no data points
            return std::nullopt;
        }

        auto const it = std::lower_bound(points.begin(), points.end(), PlotDataPoint{x, 0.0f});

        if (it == points.end())
        {
            // X is out of bounds
            return std::nullopt;
        }

        if (it == points.begin())
        {
            // X if off the left-hand side
            return points.front().y;
        }

        // else: the iterator is pointing somewhere in the middle of the data
        //       and we need to potentially LERP between two points
        size_t const aboveIdx = std::distance(points.begin(), it);
        size_t const belowIdx = aboveIdx - 1;
        PlotDataPoint const below = points[belowIdx];
        PlotDataPoint const above = points[aboveIdx];

        float const t = (x - below.x) / (above.x - below.x); // [0..1]

        return lerp(below.y, above.y, t);
    }

    std::optional<PlotDataPoint> FindNearestPoint(Plot const& p, float x)
    {
        auto lock = p.lockDataPoints();
        nonstd::span<PlotDataPoint const> points = *lock;

        if (points.empty())
        {
            // there are no data points
            return std::nullopt;
        }

        auto const it = std::lower_bound(points.begin(), points.end(), PlotDataPoint{x, 0.0f});

        if (it == points.begin())
        {
            // closest is the leftmost point
            return points.front();
        }

        if (it == points.end())
        {
            // closest is the rightmost point
            return points.back();
        }

        // else: the iterator is pointing to the element above the X location and we
        //       need to figure out if that's closer than the element below the X
        //       location
        size_t const aboveIdx = std::distance(points.begin(), it);
        size_t const belowIdx = aboveIdx - 1;
        PlotDataPoint const below = points[belowIdx];
        PlotDataPoint const above = points[aboveIdx];

        float const belowDistance = std::abs(below.x - x);
        float const aboveDistance = std::abs(above.x - x);

        size_t const closestIdx =  aboveDistance < belowDistance  ? aboveIdx : belowIdx;

        return points[closestIdx];
    }

    bool IsXInRange(Plot const& p, float x)
    {
        auto lock = p.lockDataPoints();
        nonstd::span<PlotDataPoint const> const points = *lock;

        if (points.size() <= 1)
        {
            return false;
        }

        return points.front().x <= x && x <= points.back().x;
    }

    void PlotLine(osc::CStringView lineName, Plot const& p)
    {
        auto lock = p.lockDataPoints();
        nonstd::span<PlotDataPoint const> points = *lock;


        float const* xPtr = nullptr;
        float const* yPtr = nullptr;
        if (!points.empty())
        {
            xPtr = &points.front().x;
            yPtr = &points.front().y;
        }

        ImPlot::PlotLine(
            lineName.c_str(),
            xPtr,
            yPtr,
            static_cast<int>(points.size()),
            0,
            sizeof(PlotDataPoint)
        );
    }

    std::string IthPlotLineName(Plot const& p, size_t i)
    {
        std::stringstream ss;
        ss << i << ") " << p.getParameters().getCommit().getCommitMessage();
        if (p.getIsLocked())
        {
            ss << " " ICON_FA_LOCK;
        }
        return std::move(ss).str();
    }

    // holds a collection of plotlines that are to-be-drawn on the plot
    class PlotLines final {
    public:
        PlotLines(PlotParameters const& params) :
            m_ActivePlot{std::make_shared<Plot>(params)}
        {
        }

        void onBeforeDrawing(osc::UndoableModelStatePair const& model, PlotParameters const& desiredParams)
        {
            // perform any datastructure invariant checks etc.

            // additions/changes
            //
            // if the current plot doesn't match the latest requested params, kick off
            // a new plotting task
            if (m_ActivePlot->getParameters() != desiredParams)
            {
                // (edge-case): if the user selected a different muscle output then the previous
                // plots should also be cleared
                bool const clearPrevious = m_ActivePlot->getParameters().getMuscleOutput() != desiredParams.getMuscleOutput();

                // create new active plot and swap the old active plot into the previous plots
                {
                    std::shared_ptr<Plot> p = std::make_shared<Plot>(desiredParams);
                    std::swap(p, m_ActivePlot);
                    m_PreviousPlots.push_back(p);
                }

                if (clearPrevious)
                {
                    m_PreviousPlots.clear();
                }

                // kick off a new plotting task
                m_PlottingTask = PlottingTask{m_ActivePlot->getParameters(), m_ActivePlot};
            }

            // deletions
            //
            // handle any user-requested deletions by removing the curve from the collection
            if (0 <= m_PlotTaggedForDeletion && m_PlotTaggedForDeletion < m_PreviousPlots.size())
            {
                m_PreviousPlots.erase(m_PreviousPlots.begin() + m_PlotTaggedForDeletion);
                m_PlotTaggedForDeletion = -1;
            }

            // cleanups/reductions
            //
            // handle removing previous plots, if there are too many currently stored
            {
                // algorithm:
                //
                // - go backwards through the history list and count up *unlocked* elements until
                //   either the beginning is hit (there are too few - nothing to GC) or the maximum
                //   number of history entries is hit (`hmax`)s
                //
                // - go forwards through the history list, deleting any *unlocked* elements until
                //   `hmax` is hit
                //
                // - you now have a list containing 0..`hmax` unlocked elements, plus locked elements,
                //   where the unlocked elements are the most recently used

                auto isFirstDeleteablePlot = [nth = 1, max = this->m_MaxHistoryEntries](std::shared_ptr<Plot> const& p) mutable
                {
                    if (p->getIsLocked())
                    {
                        return false;
                    }
                    return nth++ > max;
                };

                auto const backwardIt = std::find_if(m_PreviousPlots.rbegin(), m_PreviousPlots.rend(), isFirstDeleteablePlot);
                auto const forwardIt = backwardIt.base();
                size_t const idxOfDeleteableEnd = std::distance(m_PreviousPlots.begin(), forwardIt);

                auto shouldDelete = [i = 0, idxOfDeleteableEnd](std::shared_ptr<Plot> const& p) mutable
                {
                    return i++ < idxOfDeleteableEnd && !p->getIsLocked();
                };

                auto it = std::remove_if(m_PreviousPlots.begin(), m_PreviousPlots.end(), shouldDelete);
                m_PreviousPlots.erase(it, m_PreviousPlots.end());
            }
        }

        void clearUnlockedPlots()
        {
            m_PreviousPlots.clear();
        }

        PlottingTaskStatus getPlottingTaskStatus() const
        {
            return m_PlottingTask.getStatus();
        }

        std::optional<std::string> tryGetPlottingTaskErrorMessage() const
        {
            return m_PlottingTask.getErrorString();
        }

        Plot const& getActivePlot() const
        {
            return *m_ActivePlot;
        }

        size_t getNumOtherPlots() const
        {
            return m_PreviousPlots.size();
        }

        Plot const& getOtherPlot(size_t i) const
        {
            return *m_PreviousPlots.at(i);
        }

        void tagOtherPlotForDeletion(size_t i)
        {
            m_PlotTaggedForDeletion = static_cast<int>(i);
        }

        void setOtherPlotLocked(size_t i, bool v)
        {
            m_PreviousPlots.at(i)->setIsLocked(v);
        }

        int getMaxHistoryEntries() const
        {
            return m_MaxHistoryEntries;
        }

        void setMaxHistoryEntries(int i)
        {
            if (i < 0)
            {
                return;
            }
            m_MaxHistoryEntries = std::move(i);
        }

    private:
        std::shared_ptr<Plot> m_ActivePlot;
        PlottingTask m_PlottingTask{m_ActivePlot->getParameters(), m_ActivePlot};
        std::vector<std::shared_ptr<Plot>> m_PreviousPlots;
        int m_PlotTaggedForDeletion = -1;
        int m_MaxHistoryEntries = 6;
    };
}

// UI state
//
// top-level state API - all "states" of the widget share this info and implement the
// relevant state API
namespace
{
    // data that is shared between all states of the widget
    struct SharedStateData final {
        explicit SharedStateData(std::shared_ptr<osc::UndoableModelStatePair> uim) : Uim{ std::move(uim) }
        {
            OSC_ASSERT(Uim != nullptr);
        }

        SharedStateData(std::shared_ptr<osc::UndoableModelStatePair> uim,
            OpenSim::ComponentPath const& coordPath,
            OpenSim::ComponentPath const& musclePath) :
            Uim{ std::move(uim) },
            PlotParams{ Uim->getLatestCommit(), coordPath, musclePath, GetDefaultMuscleOutput(), 180 }
        {
            OSC_ASSERT(Uim != nullptr);
        }

        std::shared_ptr<osc::UndoableModelStatePair> Uim;
        PlotParameters PlotParams{ Uim->getLatestCommit(), OpenSim::ComponentPath{}, OpenSim::ComponentPath{}, GetDefaultMuscleOutput(), 180 };
    };

    // base class for a single widget state
    class MusclePlotState {
    protected:
        MusclePlotState(SharedStateData* shared_) : shared{ std::move(shared_) }
        {
            OSC_ASSERT(shared != nullptr);
        }
    public:
        virtual ~MusclePlotState() noexcept = default;
        virtual std::unique_ptr<MusclePlotState> draw() = 0;

    protected:
        SharedStateData* shared;
    };

}

// "showing plot" state
//
// this is the biggest, most important, state of the widget: it is what's used when
// the widget is showing a muscle curve to the user
namespace
{
    // state in which the plot is being shown to the user
    //
    // this is the most important state of the state machine
    class ShowingPlotState final : public MusclePlotState {
    public:
        explicit ShowingPlotState(SharedStateData* shared_) :
            MusclePlotState{std::move(shared_)},
            m_Lines{shared->PlotParams}
        {
        }

        std::unique_ptr<MusclePlotState> draw() override
        {
            onBeforeDrawing();  // perform pre-draw cleanups/updates etc.

            if (m_Lines.getPlottingTaskStatus() == PlottingTaskStatus::Error)
            {
                if (auto maybeErrorString = m_Lines.tryGetPlottingTaskErrorMessage())
                {
                    ImGui::Text("error: cannot show plot: %s", maybeErrorString->c_str());
                }
                return nullptr;
            }

            PlotParameters const& latestParams = shared->PlotParams;
            auto modelGuard = latestParams.getCommit().getModel();

            OpenSim::Coordinate const* maybeCoord = osc::FindComponent<OpenSim::Coordinate>(*modelGuard, latestParams.getCoordinatePath());
            if (!maybeCoord)
            {
                ImGui::Text("(no coordinate named %s in model)", latestParams.getCoordinatePath().toString().c_str());
                return nullptr;
            }
            OpenSim::Coordinate const& coord = *maybeCoord;

            std::string const plotTitle = computePlotTitle(coord);

            ImPlot::PushStyleVar(ImPlotStyleVar_FitPadding, {0.025f, 0.05f});
            if (ImPlot::BeginPlot(plotTitle.c_str(), ImGui::GetContentRegionAvail(), m_PlotFlags))
            {
                PlotParameters const& plotParams = shared->PlotParams;

                ImPlot::SetupLegend(
                    m_LegendLocation,
                    m_LegendFlags
                );
                ImPlot::SetupAxes(
                    computePlotXAxisTitle(coord).c_str(),
                    computePlotYAxisTitle().c_str(),
                    ImPlotAxisFlags_Lock,
                    ImPlotAxisFlags_AutoFit
                );
                ImPlot::SetupAxisLimits(
                    ImAxis_X1,
                    osc::ConvertCoordValueToDisplayValue(coord, GetFirstXValue(plotParams, coord)),
                    osc::ConvertCoordValueToDisplayValue(coord, GetLastXValue(plotParams, coord))
                );
                ImPlot::SetupFinish();

                std::optional<float> maybeMouseX = tryGetMouseXPositionInPlot();
                drawPlotLines();
                drawOverlays(coord, maybeMouseX);
                handleMouseEvents(coord, maybeMouseX);
                if (!m_LegendPopupIsOpen)
                {
                    tryDrawGeneralPlotPopup(plotTitle);
                }

                ImPlot::EndPlot();
            }

            ImPlot::PopStyleVar();

            return nullptr;
        }

    private:

        // called at the start of each `draw` call - it GCs datastructures etc.
        void onBeforeDrawing()
        {
            // ensure the legend test is reset (it's checked every frame)
            m_LegendPopupIsOpen = false;

            // ensure latest requested params reflects the latest version of the model
            shared->PlotParams.setCommit(shared->Uim->getLatestCommit());

            // ensure plot lines are valid, given the current model + desired params
            m_Lines.onBeforeDrawing(*shared->Uim, shared->PlotParams);
        }

        // tries to hittest the mouse's X position in plot-space
        std::optional<float> tryGetMouseXPositionInPlot() const
        {
            // figure out mouse hover position
            bool const isHovered = ImPlot::IsPlotHovered();
            float mouseX = static_cast<float>(ImPlot::GetPlotMousePos().x);

            // handle snapping the mouse's X position
            if (isHovered && m_SnapCursor)
            {
                auto maybeNearest = FindNearestPoint(m_Lines.getActivePlot(), mouseX);

                if (IsXInRange(m_Lines.getActivePlot(), mouseX) && maybeNearest)
                {
                    mouseX = maybeNearest->x;
                }
            }

            return isHovered ? mouseX : std::optional<float>{};
        }

        // draws the actual plot lines in the plot
        void drawPlotLines()
        {
            // plot not-active plots
            for (size_t i = 0, len = m_Lines.getNumOtherPlots(); i < len; ++i)
            {
                Plot const& plot = m_Lines.getOtherPlot(i);

                glm::vec4 color = m_ComputedPlotLineBaseColor;
                color.a *= static_cast<float>(i + 1) / static_cast<float>(len + 1);

                if (m_ShowMarkersOnOtherPlots)
                {
                    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 3.0f);
                }

                std::string const lineName = IthPlotLineName(plot, i + 1);

                ImPlot::PushStyleColor(ImPlotCol_Line, color);
                PlotLine(lineName, plot);
                ImPlot::PopStyleColor(ImPlotCol_Line);

                if (ImPlot::BeginLegendPopup(lineName.c_str()))
                {
                    m_LegendPopupIsOpen = true;

                    if (ImGui::MenuItem(ICON_FA_TRASH " delete"))
                    {
                        m_Lines.tagOtherPlotForDeletion(i);
                    }
                    if (!plot.getIsLocked() && ImGui::MenuItem(ICON_FA_LOCK " lock"))
                    {
                        m_Lines.setOtherPlotLocked(i, true);
                    }
                    if (plot.getIsLocked() && ImGui::MenuItem(ICON_FA_UNLOCK " unlock"))
                    {
                        m_Lines.setOtherPlotLocked(i, false);
                    }
                    ImPlot::EndLegendPopup();
                }
            }

            // then plot the active plot
            {
                Plot const& p = m_Lines.getActivePlot();
                std::string const lineName = IthPlotLineName(p, m_Lines.getNumOtherPlots() + 1);

                if (m_ShowMarkersOnActivePlot)
                {
                    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 3.0f);
                }

                ImPlot::PushStyleColor(ImPlotCol_Line, m_ComputedPlotLineBaseColor);
                PlotLine(lineName, p);
                ImPlot::PopStyleColor(ImPlotCol_Line);
            }
        }

        // draw overlays over the plot lines
        void drawOverlays(OpenSim::Coordinate const& coord, std::optional<float> maybeMouseX)
        {
            double coordinateXInDegrees = osc::ConvertCoordValueToDisplayValue(coord, coord.getValue(shared->Uim->getState()));

            // draw vertical drop line where the coordinate's value currently is
            {
                double v = coordinateXInDegrees;
                ImPlot::DragLineX(10, &v, {1.0f, 1.0f, 0.0f, 0.6f}, 1.0f, ImPlotDragToolFlags_NoInputs);
            }

            // also, draw an X tag on the axes where the coordinate's value currently is
            ImPlot::TagX(coordinateXInDegrees, {1.0f, 1.0f, 1.0f, 1.0f});

            // draw faded vertial drop line where the mouse currently is
            if (maybeMouseX)
            {
                double v = *maybeMouseX;
                ImPlot::DragLineX(11, &v, {1.0f, 1.0f, 0.0f, 0.3f}, 1.0f, ImPlotDragToolFlags_NoInputs);
            }

            // also, draw a faded X tag on the axes where the mouse currently is (in X)
            if (maybeMouseX)
            {
                ImPlot::TagX(*maybeMouseX, {1.0f, 1.0f, 1.0f, 0.6f});
            }

            // Y values: BEWARE
            //
            // the X values for the droplines/tags above come directly from either the model or
            // mouse: both of which are *continuous* (give or take)
            //
            // the Y values are computed from those continous values by searching through the
            // *discrete* data values of the plot and LERPing them
            {
                // draw current coordinate value as a solid dropline
                {
                    std::optional<float> maybeCoordinateY = ComputeLERPedY(m_Lines.getActivePlot(), static_cast<float>(coordinateXInDegrees));

                    if (maybeCoordinateY)
                    {
                        double v = *maybeCoordinateY;
                        ImPlot::DragLineY(13, &v, {1.0f, 1.0f, 0.0f, 0.6f}, 1.0f, ImPlotDragToolFlags_NoInputs);
                        ImPlot::Annotation(static_cast<float>(coordinateXInDegrees), *maybeCoordinateY, {1.0f, 1.0f, 1.0f, 1.0f}, {10.0f, 10.0f}, true, "%f", *maybeCoordinateY);
                    }
                }

                // (try to) draw the hovered coordinate value as a faded dropline
                if (maybeMouseX)
                {
                    std::optional<float> const maybeHoverY = ComputeLERPedY(m_Lines.getActivePlot(), *maybeMouseX);
                    if (maybeHoverY)
                    {
                        double v = *maybeHoverY;
                        ImPlot::DragLineY(14, &v, {1.0f, 1.0f, 0.0f, 0.3f}, 1.0f, ImPlotDragToolFlags_NoInputs);
                        ImPlot::Annotation(*maybeMouseX, *maybeHoverY, {1.0f, 1.0f, 1.0f, 0.6f}, {10.0f, 10.0f}, true, "%f", *maybeHoverY);
                    }
                }
            }
        }

        void handleMouseEvents(OpenSim::Coordinate const& coord, std::optional<float> maybeMouseX)
        {
            // if the plot is hovered and the user is holding their left-mouse button down,
            // then "scrub" through the coordinate in the model
            //
            // this is handy for users to visually see how a coordinate affects the model
            if (maybeMouseX && ImGui::IsMouseDown(ImGuiMouseButton_Left))
            {
                if (coord.getDefaultLocked())
                {
                    osc::DrawTooltip("scrubbing disabled", "you cannot scrub this plot because the coordinate is locked");
                }
                else
                {
                    double storedValue = osc::ConvertCoordDisplayValueToStorageValue(coord, *maybeMouseX);
                    osc::ActionSetCoordinateValue(*shared->Uim, coord, storedValue);
                }
            }

            // when the user stops dragging their left-mouse around, commit the scrubbed-to
            // coordinate to model storage
            if (maybeMouseX && ImGui::IsMouseReleased(ImGuiMouseButton_Left))
            {
                if (coord.getDefaultLocked())
                {
                    osc::DrawTooltip("scrubbing disabled", "you cannot scrub this plot because the coordinate is locked");
                }
                else
                {
                    double storedValue = osc::ConvertCoordDisplayValueToStorageValue(coord, *maybeMouseX);
                    osc::ActionSetCoordinateValueAndSave(*shared->Uim, coord, storedValue);
                }
            }
        }

        void tryDrawGeneralPlotPopup(std::string const& plotTitle)
        {
            // draw a context menu with helpful options (set num data points, export, etc.)
            if (ImGui::BeginPopupContextItem((plotTitle + "_contextmenu").c_str()))
            {
                drawPlotDataTypeSelector();

                // editor: max data points
                {
                    int currentDataPoints = shared->PlotParams.getNumRequestedDataPoints();
                    if (ImGui::InputInt("num data points", &currentDataPoints, 1, 100, ImGuiInputTextFlags_EnterReturnsTrue))
                    {
                        if (currentDataPoints >= 0)
                        {
                            shared->PlotParams.setNumRequestedDataPoints(currentDataPoints);
                        }
                    }
                }

                // editor: max history entries
                {
                    int maxHistoryEntries = m_Lines.getMaxHistoryEntries();
                    if (ImGui::InputInt("max history size", &maxHistoryEntries, 1, 100, ImGuiInputTextFlags_EnterReturnsTrue))
                    {
                        if (maxHistoryEntries >= 0)
                        {
                            m_Lines.setMaxHistoryEntries(maxHistoryEntries);
                        }
                    }
                }

                if (ImGui::MenuItem("clear unlocked plots"))
                {
                    m_Lines.clearUnlockedPlots();
                }

                if (ImGui::BeginMenu("legend"))
                {
                    drawLegendContextMenuContent();
                    ImGui::EndMenu();
                }

                ImGui::MenuItem("show markers", nullptr, &m_ShowMarkersOnActivePlot);
                ImGui::MenuItem("show markers on other plots", nullptr, &m_ShowMarkersOnOtherPlots);
                ImGui::MenuItem("snap cursor to datapoints", nullptr, &m_SnapCursor);

                ImGui::EndPopup();
            }
        }

        void drawPlotDataTypeSelector()
        {
            std::vector<char const*> names;
            names.reserve(m_AvailableMuscleOutputs.size());

            int active = -1;
            for (int i = 0; i < static_cast<int>(m_AvailableMuscleOutputs.size()); ++i)
            {
                MuscleOutput const& o = m_AvailableMuscleOutputs[i];
                names.push_back(o.getName());
                if (o == shared->PlotParams.getMuscleOutput())
                {
                    active = i;
                }
            }

            if (ImGui::Combo("data type", &active, names.data(), static_cast<int>(names.size())))
            {
                shared->PlotParams.setMuscleOutput(m_AvailableMuscleOutputs[active]);
            }
        }

        void drawLegendContextMenuContent()
        {
            ImGui::CheckboxFlags("Hide", (unsigned int*)&m_PlotFlags, ImPlotFlags_NoLegend);
            ImGui::CheckboxFlags("Outside", (unsigned int*)&m_LegendFlags, ImPlotLegendFlags_Outside);

            const float s = ImGui::GetFrameHeight();
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2, 2));
            if (ImGui::Button("NW", ImVec2(1.5f * s, s))) { m_LegendLocation = ImPlotLocation_NorthWest; } ImGui::SameLine();
            if (ImGui::Button("N", ImVec2(1.5f * s, s))) { m_LegendLocation = ImPlotLocation_North; } ImGui::SameLine();
            if (ImGui::Button("NE", ImVec2(1.5f * s, s))) { m_LegendLocation = ImPlotLocation_NorthEast; }
            if (ImGui::Button("W", ImVec2(1.5f * s, s))) { m_LegendLocation = ImPlotLocation_West; } ImGui::SameLine();
            if (ImGui::InvisibleButton("C", ImVec2(1.5f * s, s))) { m_LegendLocation = ImPlotLocation_Center; } ImGui::SameLine();
            if (ImGui::Button("E", ImVec2(1.5f * s, s))) { m_LegendLocation = ImPlotLocation_East; }
            if (ImGui::Button("SW", ImVec2(1.5f * s, s))) { m_LegendLocation = ImPlotLocation_SouthWest; } ImGui::SameLine();
            if (ImGui::Button("S", ImVec2(1.5f * s, s))) { m_LegendLocation = ImPlotLocation_South; } ImGui::SameLine();
            if (ImGui::Button("SE", ImVec2(1.5f * s, s))) { m_LegendLocation = ImPlotLocation_SouthEast; }
            ImGui::PopStyleVar();
        }

        std::string computePlotTitle(OpenSim::Coordinate const& c)
        {
            std::stringstream ss;
            ss << shared->PlotParams.getMusclePath().getComponentName() << ' ';
            appendYAxisName(ss);
            ss << " vs ";
            appendXAxisName(c, ss);
            return std::move(ss).str();
        }

        void appendYAxisName(std::stringstream& ss)
        {
            ss << shared->PlotParams.getMuscleOutput().getName();
        }

        void appendXAxisName(OpenSim::Coordinate const& c, std::stringstream& ss)
        {
            ss << c.getName();
        }

        std::string computePlotYAxisTitle()
        {
            std::stringstream ss;
            appendYAxisName(ss);
            ss << " [" << shared->PlotParams.getMuscleOutput().getUnits() << ']';
            return std::move(ss).str();
        }

        std::string computePlotXAxisTitle(OpenSim::Coordinate const& c)
        {
            std::stringstream ss;
            appendXAxisName(c, ss);
            ss << " value [" << osc::GetCoordDisplayValueUnitsString(c) << ']';
            return std::move(ss).str();
        }

        // plot data state
        PlotLines m_Lines;

        // UI/drawing/widget state
        std::vector<MuscleOutput> m_AvailableMuscleOutputs = GenerateMuscleOutputs();
        glm::vec4 m_ComputedPlotLineBaseColor = {1.0f, 1.0f, 1.0f, 1.0f};
        bool m_LegendPopupIsOpen = false;
        bool m_ShowMarkersOnActivePlot = true;
        bool m_ShowMarkersOnOtherPlots = false;
        bool m_SnapCursor = false;
        ImPlotFlags m_PlotFlags = ImPlotFlags_AntiAliased | ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect | ImPlotFlags_NoChild | ImPlotFlags_NoFrame;
        ImPlotLocation m_LegendLocation = ImPlotLocation_NorthWest;
        ImPlotLegendFlags m_LegendFlags = ImPlotLegendFlags_None;
    };
}

// other states
 namespace
 {
    // state in which a user is being prompted to select a coordinate in the model
    class PickCoordinateState final : public MusclePlotState {
    public:
        explicit PickCoordinateState(SharedStateData* shared_) : MusclePlotState{std::move(shared_)}
        {
            // this is what this state is populating
            shared->PlotParams.setCoordinatePath(osc::GetEmptyComponentPath());
        }

        std::unique_ptr<MusclePlotState> draw() override
        {
            std::unique_ptr<MusclePlotState> rv;

            std::vector<OpenSim::Coordinate const*> coordinates;
            for (OpenSim::Coordinate const& coord : shared->Uim->getModel().getComponentList<OpenSim::Coordinate>())
            {
                coordinates.push_back(&coord);
            }
            osc::Sort(coordinates, osc::IsNameLexographicallyLowerThan<OpenSim::Component const*>);

            ImGui::Text("select coordinate:");

            ImGui::BeginChild("MomentArmPlotCoordinateSelection");
            for (OpenSim::Coordinate const* coord : coordinates)
            {
                if (ImGui::Selectable(coord->getName().c_str()))
                {
                    shared->PlotParams.setCoordinatePath(coord->getAbsolutePath());
                    rv = std::make_unique<ShowingPlotState>(shared);
                }
            }
            ImGui::EndChild();

            return rv;
        }
    };

    // state in which a user is being prompted to select a muscle in the model
    class PickMuscleState final : public MusclePlotState {
    public:
        explicit PickMuscleState(SharedStateData* shared_) : MusclePlotState{std::move(shared_)}
        {
            // this is what this state is populating
            shared->PlotParams.setMusclePath(osc::GetEmptyComponentPath());
        }

        std::unique_ptr<MusclePlotState> draw() override
        {
            std::unique_ptr<MusclePlotState> rv;

            std::vector<OpenSim::Muscle const*> muscles;
            for (OpenSim::Muscle const& musc : shared->Uim->getModel().getComponentList<OpenSim::Muscle>())
            {
                muscles.push_back(&musc);
            }
            osc::Sort(muscles, osc::IsNameLexographicallyLowerThan<OpenSim::Component const*>);

            ImGui::Text("select muscle:");

            if (muscles.empty())
            {
                ImGui::TextDisabled("(the model contains no muscles?)");
            }
            else
            {
                ImGui::BeginChild("MomentArmPlotMuscleSelection");
                for (OpenSim::Muscle const* musc : muscles)
                {
                    if (ImGui::Selectable(musc->getName().c_str()))
                    {
                        shared->PlotParams.setMusclePath(musc->getAbsolutePath());
                        rv = std::make_unique<PickCoordinateState>(shared);
                    }
                }
                ImGui::EndChild();
            }

            return rv;
        }
    };
}

// private IMPL for the muscle plot panel
//
// this effectively operates as a state-machine host, where each state (e.g.
// "choose a muscle", "choose a coordinate") is mostly independent
class osc::ModelMusclePlotPanel::Impl final {
public:

    Impl(std::shared_ptr<UndoableModelStatePair> uim, std::string_view panelName) :
        m_SharedData{std::move(uim)},
        m_ActiveState{std::make_unique<PickMuscleState>(&m_SharedData)},
        m_PanelName{std::move(panelName)}
    {
    }

    Impl(std::shared_ptr<UndoableModelStatePair> uim,
         std::string_view panelName,
         OpenSim::ComponentPath const& coordPath,
         OpenSim::ComponentPath const& musclePath) :
        m_SharedData{std::move(uim), coordPath, musclePath},
        m_ActiveState{std::make_unique<ShowingPlotState>(&m_SharedData)},
        m_PanelName{std::move(panelName)}
    {
    }

    std::string const& getName() const
    {
        return m_PanelName;
    }

    bool isOpen() const
    {
        return m_IsOpen;
    }

    void open()
    {
        m_IsOpen = true;
    }

    void close()
    {
        m_IsOpen = false;
    }

    void draw()
    {
        if (m_IsOpen)
        {
            bool isOpen = m_IsOpen;
            if (ImGui::Begin(m_PanelName.c_str(), &isOpen))
            {
                if (auto maybeNextState = m_ActiveState->draw())
                {
                    m_ActiveState = std::move(maybeNextState);
                }
                m_IsOpen = isOpen;
            }
            ImGui::End();

            if (isOpen != m_IsOpen)
            {
                m_IsOpen = isOpen;
            }
        }
    }

private:
    // data that's shared between all states
    SharedStateData m_SharedData;

    // currently active state (this class controls a state machine)
    std::unique_ptr<MusclePlotState> m_ActiveState;

    // name of the panel, as shown in the UI (via ImGui::Begin)
    std::string m_PanelName;

    // if the panel is currently open or not
    bool m_IsOpen = true;
};


// public API (PIMPL)

osc::ModelMusclePlotPanel::ModelMusclePlotPanel(std::shared_ptr<UndoableModelStatePair> uim,
                                                std::string_view panelName) :
    m_Impl{new Impl{std::move(uim), std::move(panelName)}}
{
}

osc::ModelMusclePlotPanel::ModelMusclePlotPanel(std::shared_ptr<UndoableModelStatePair> uim,
                                                std::string_view panelName,
                                                OpenSim::ComponentPath const& coordPath,
                                                OpenSim::ComponentPath const& musclePath) :
    m_Impl{new Impl{std::move(uim), std::move(panelName), coordPath, musclePath}}
{
}

osc::ModelMusclePlotPanel::ModelMusclePlotPanel(ModelMusclePlotPanel&& tmp) noexcept :
    m_Impl{std::exchange(tmp.m_Impl, nullptr)}
{
}

osc::ModelMusclePlotPanel& osc::ModelMusclePlotPanel::operator=(ModelMusclePlotPanel&& tmp) noexcept
{
    std::swap(m_Impl, tmp.m_Impl);
    return *this;
}

osc::ModelMusclePlotPanel::~ModelMusclePlotPanel() noexcept
{
    delete m_Impl;
}

std::string const& osc::ModelMusclePlotPanel::getName() const
{
    return m_Impl->getName();
}

bool osc::ModelMusclePlotPanel::isOpen() const
{
    return m_Impl->isOpen();
}

void osc::ModelMusclePlotPanel::open()
{
    m_Impl->open();
}

void osc::ModelMusclePlotPanel::close()
{
    m_Impl->close();
}

void osc::ModelMusclePlotPanel::draw()
{
    m_Impl->draw();
}
