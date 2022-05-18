#include "Tab.hpp"

#include "src/Utils/CStringView.hpp"
#include "src/Utils/UID.hpp"

#include <SDL_events.h>

// the impl-forwarding methods are here so that it's easier to hook into
// the API (e.g. for debugging)

// public API

osc::UID osc::Tab::getID() const
{
	return implGetID();
}

osc::CStringView osc::Tab::getName() const
{
	return implGetName();
}

osc::TabHost* osc::Tab::parent() const
{
	return implParent();
}

void osc::Tab::onMount()
{
	implOnMount();
}

void osc::Tab::onUnmount()
{
	implOnUnmount();
}

bool osc::Tab::onEvent(SDL_Event const& e)
{
	return implOnEvent(e);
}

void osc::Tab::onTick()
{
	implOnTick();
}

void osc::Tab::onDrawMainMenu()
{
	implOnDrawMainMenu();
}

void osc::Tab::onDraw()
{
	implOnDraw();
}
