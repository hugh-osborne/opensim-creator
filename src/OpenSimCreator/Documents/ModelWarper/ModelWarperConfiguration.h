#pragma once

#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/Model/Station.h>

#include <compare>
#include <concepts>
#include <filesystem>
#include <sstream>
#include <string_view>
#include <typeinfo>
#include <unordered_set>
#include <utility>

namespace osc::mow
{
    // describes how closely (if at all) a `ComponentWarpingStrategy` matches a
    // given `OpenSim::Component`
    //
    // used for resolving potentially-ambiguous matches across multiple strategies
    class StrategyMatchQuality final {
    public:
        static constexpr StrategyMatchQuality none() { return StrategyMatchQuality{State::None}; }
        static constexpr StrategyMatchQuality wildcard() { return StrategyMatchQuality{State::Wildcard}; }
        static constexpr StrategyMatchQuality exact() { return StrategyMatchQuality{State::Exact}; }

        constexpr operator bool () const { return _state != State::None; }

        friend constexpr bool operator==(StrategyMatchQuality, StrategyMatchQuality) = default;
        friend constexpr auto operator<=>(StrategyMatchQuality, StrategyMatchQuality) = default;
    private:
        enum class State {
            None,
            Wildcard,
            Exact
        };

        explicit constexpr StrategyMatchQuality(State state) :
            _state{state}
        {}

        State _state = State::None;
    };

    // abstract interface to a component that is capable of warping `n` other
    // components (`StrategyTargets`) during a model warp
    class ComponentWarpingStrategy : public OpenSim::Component {
        OpenSim_DECLARE_ABSTRACT_OBJECT(ComponentWarpingStrategy, OpenSim::Component);
    public:
        OpenSim_DECLARE_LIST_PROPERTY(StrategyTargets, std::string, "a sequence of strategy target strings that this strategy applies to");
    protected:
        ComponentWarpingStrategy()
        {
            constructProperty_StrategyTargets();
        }
        ComponentWarpingStrategy(const ComponentWarpingStrategy&) = default;
        ComponentWarpingStrategy(ComponentWarpingStrategy&&) noexcept = default;
        ComponentWarpingStrategy& operator=(const ComponentWarpingStrategy&) = default;
        ComponentWarpingStrategy& operator=(ComponentWarpingStrategy&&) noexcept = default;
    public:
        ~ComponentWarpingStrategy() noexcept override = default;

        const std::type_info& getTargetComponentTypeInfo() const
        {
            return implGetTargetComponentTypeInfo();
        }

        StrategyMatchQuality calculateMatchQuality(const OpenSim::Component& candidateComponent) const
        {
            if (not implIsMatchForComponentType(candidateComponent)) {
                return StrategyMatchQuality::none();
            }

            const auto componentAbsPath = candidateComponent.getAbsolutePathString();

            // loop through strategy targets and select the best one, throw if any match
            // is ambiguous
            StrategyMatchQuality best = StrategyMatchQuality::none();
            for (int i = 0; i < getProperty_StrategyTargets().size(); ++i) {
                const std::string& target = get_StrategyTargets(i);
                if (target == componentAbsPath) {
                    // you can't do any better than this, and `extendFinalizeFromProperties`
                    // guarantees no other `StrategyTarget`s are going to match exactly, so
                    // exit early
                    return StrategyMatchQuality::exact();
                }
                else if (target == "*") {
                    best = StrategyMatchQuality::wildcard();
                }
            }
            return best;
        }
    private:
        virtual const std::type_info& implGetTargetComponentTypeInfo() const = 0;
        virtual bool implIsMatchForComponentType(const OpenSim::Component&) const = 0;

        void extendFinalizeFromProperties() override
        {
            assertStrategyTargetsNotEmpty();
            assertStrategyTargetsAreUnique();
        }

        void assertStrategyTargetsNotEmpty()
        {
            if (getProperty_StrategyTargets().empty()) {
                OPENSIM_THROW_FRMOBJ(OpenSim::Exception, "The <StrategyTargets> property of this component must be populated with at least one entry");
            }
        }

        void assertStrategyTargetsAreUnique()
        {
            const int numStrategyTargets = getProperty_StrategyTargets().size();
            std::unordered_set<std::string_view> uniqueStrategyTargets;
            uniqueStrategyTargets.reserve(numStrategyTargets);
            for (int i = 0; i < numStrategyTargets; ++i) {
                const std::string& strategyTarget = get_StrategyTargets(i);
                const auto [_, inserted] = uniqueStrategyTargets.emplace(strategyTarget);
                if (not inserted) {
                    std::stringstream ss;
                    ss << strategyTarget << ": duplicate strategy target detected: all strategy targets must be unique";
                    OPENSIM_THROW_FRMOBJ(OpenSim::Exception, std::move(ss).str());
                }
            }
        }
    };

    // abstract interface to a component that is capable of warping `n` other
    // components (`StrategyTargets`) of type `T` during a model warp
    template<std::derived_from<OpenSim::Component> T>
    class ComponentWarpingStrategyFor : public ComponentWarpingStrategy {
    public:
        ComponentWarpingStrategyFor() = default;

    private:
        const std::type_info& implGetTargetComponentTypeInfo() const override
        {
            return typeid(T);
        }

        bool implIsMatchForComponentType(const OpenSim::Component& component) const override
        {
            return dynamic_cast<const T*>(&component) != nullptr;
        }
    };

    // abstract interface to a component that is capable of warping `n`
    // `OpenSim::PhysicalOffsetFrame`s during a model warp
    class OffsetFrameWarpingStrategy : public ComponentWarpingStrategyFor<OpenSim::PhysicalOffsetFrame> {
        OpenSim_DECLARE_ABSTRACT_OBJECT(OffsetFrameWarpingStrategy, ComponentWarpingStrategyFor<OpenSim::PhysicalOffsetFrame>);
    };

    // concrete implementation of an `OffsetFrameWarpingStrategy` in which
    // only the `translation` property of the offset frame is warped but the
    // rotation is left as-is
    class ThinPlateSplineOnlyTranslationOffsetFrameWarpingStrategy final : public OffsetFrameWarpingStrategy {
        OpenSim_DECLARE_CONCRETE_OBJECT(ThinPlateSplineOnlyTranslationOffsetFrameWarpingStrategy, OffsetFrameWarpingStrategy);
    public:
        // PointSources
    private:
    };

    // concrete implementation of an `OffsetFrameWarpingStrategy` in which the
    // implementation should produce a halting error rather than continuing with
    // the model warp
    class ProduceErrorOffsetFrameWarpingStrategy final : public OffsetFrameWarpingStrategy {
        OpenSim_DECLARE_CONCRETE_OBJECT(ProduceErrorOffsetFrameWarpingStrategy, OffsetFrameWarpingStrategy);
    };

    // concrete implementation of an `OffsetFrameWarpingStrategy` in which the implementation
    // simply copies the `translation` and `rotation` of the source `OpenSim::PhysicalOffsetFrame` to
    // the destination model with no modifications
    class IdentityOffsetFrameWarpingStrategy final : public OffsetFrameWarpingStrategy {
        OpenSim_DECLARE_CONCRETE_OBJECT(IdentityOffsetFrameWarpingStrategy, OffsetFrameWarpingStrategy);
    };

    // abstract interface to a component that is capable of warping `n`
    // `OpenSim::Station`s during a model warp
    class StationWarpingStrategy : public ComponentWarpingStrategyFor<OpenSim::Station> {
        OpenSim_DECLARE_ABSTRACT_OBJECT(StationWarpingStrategy, ComponentWarpingStrategy);
    };

    // concrete implementation of a `StationWarpingStrategy` that uses the Thin-Plate
    // Spline (TPS) algorithm to fit correspondences between mesh landmarks (`MeshSources`)
    class ThinPlateSplineStationWarpingStrategy final : public StationWarpingStrategy {
        OpenSim_DECLARE_CONCRETE_OBJECT(ThinPlateSplineStationWarpingStrategy, StationWarpingStrategy);
        // MeshSources
    };

    // concrete implementation of a `StationWarpingStrategy` in which the implementation should
    // produce a halting error rather than continuing with the model warp
    class ProduceErrorStationWarpingStrategy final : public StationWarpingStrategy {
        OpenSim_DECLARE_CONCRETE_OBJECT(ProduceErrorStationWarpingStrategy, StationWarpingStrategy);
    };

    // concrete implementation of a `StationWarpingStrategy` in which the implementation should
    // just copy the station's postion (+parent) without any modification
    class IdentityStationWarpingStrategy final : public StationWarpingStrategy {
        OpenSim_DECLARE_CONCRETE_OBJECT(IdentityStationWarpingStrategy, StationWarpingStrategy);
    };

    // top-level model warping configuration file
    class ModelWarperConfiguration final : public OpenSim::Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(ModelWarperConfiguration, OpenSim::Component);
    public:
        // constructs a blank (default) configuration object
        ModelWarperConfiguration();

        // constructs a `ModelWarperConfiguration` by loading its properties from an XML file
        // at the given filesystem location
        explicit ModelWarperConfiguration(const std::filesystem::path& filePath);

        const ComponentWarpingStrategy* tryMatchStrategy(const OpenSim::Component&) const;
    private:
        void constructProperties();
        void extendFinalizeFromProperties() override;
    };
}
