#pragma once

#include "src/Utils/UID.hpp"

namespace OpenSim
{
	class Component;
	class Model;	
}

namespace SimTK
{
	class State;
}

namespace osc
{
	class VirtualModelStatePair {

		virtual ~VirtualModelStatePair() noexcept = 0;

		virtual OpenSim::Model const& getModel() const = 0;
		virtual OpenSim::Model& updModel() = 0;
		virtual UID getModelVersion() const = 0;

		virtual SimTK::State const& getState() const = 0;
		virtual SimTK::State& updState() = 0;
		virtual UID getStateVersion() const = 0;

		// opt-in virtual API (selection/hover/isolation logic)

		virtual OpenSim::Component const* getSelected() const { return nullptr; }
		virtual OpenSim::Component* updSelected() { return nullptr; }
		virtual void setSelected(OpenSim::Component const*) {}

		virtual OpenSim::Component const* getHovered() const { return nullptr; }
		virtual OpenSim::Component* updHovered() { return nullptr; }
		virtual void setHovered(OpenSim::Component const*) {}

		virtual OpenSim::Component const* getIsolated() const { return nullptr; }
		virtual OpenSim::Component* updIsolated() { return nullptr; }
		virtual void setIsolated(OpenSim::Component const*) {}

		// concrete helper methods that use the above virutal API

		bool hasSelected() const
		{
			return getSelected() != nullptr;
		}

		template<typename T>
		bool selectionIsType() const
		{
			OpenSim::Component const* selected = getSelected();
			return selected && typeid(*selected) == typeid(T);
		}

		template<typename T>
		bool selectionDerivesFrom() const
		{
			return getSelectedAs<T>() != nullptr;
		}

		template<typename T>
		T const* getSelectedAs() const
		{
			return dynamic_cast<T const*>(getSelected());
		}

		template<typename T>
		T* updSelectedAs()
		{
			return dynamic_cast<T*>(updSelected());
		}

		bool hasHovered() const
		{
			return getHovered() != nullptr;
		}

		void setSelectedHoveredAndIsolatedFrom(VirtualModelStatePair const& other)
		{
			setSelected(other.getSelected());
			setHovered(other.getHovered());
			setIsolated(other.getIsolated());
		}
	};
}