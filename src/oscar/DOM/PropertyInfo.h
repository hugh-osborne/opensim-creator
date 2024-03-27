#pragma once

#include <oscar/Utils/StringName.h>
#include <oscar/Variant/Variant.h>
#include <oscar/Variant/VariantType.h>

#include <string_view>
#include <utility>

namespace osc
{
    class PropertyInfo final {
    public:
        PropertyInfo() = default;

        PropertyInfo(
            std::string_view name_,
            Variant defaultValue_) :

            PropertyInfo{StringName{name_}, std::move(defaultValue_)}
        {
        }

        PropertyInfo(
            StringName name_,
            Variant defaultValue_
        );

        const StringName& getName() const
        {
            return m_Name;
        }

        VariantType getType() const
        {
            return m_DefaultValue.getType();
        }

        const Variant& getDefaultValue() const
        {
            return m_DefaultValue;
        }

        friend bool operator==(const PropertyInfo&, const PropertyInfo&) = default;

    private:
        StringName m_Name;
        Variant m_DefaultValue;
    };
}
