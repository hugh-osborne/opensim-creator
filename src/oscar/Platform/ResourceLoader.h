#pragma once

#include <oscar/Platform/IResourceLoader.h>
#include <oscar/Platform/ResourceDirectoryEntry.h>
#include <oscar/Platform/ResourcePath.h>
#include <oscar/Platform/ResourceStream.h>

#include <concepts>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace osc
{
    class ResourceLoader {
    public:
        operator IResourceLoader& () { return *impl_; }
        operator const IResourceLoader& () { return *impl_; }

        ResourceStream open(const ResourcePath& resource_path)
        {
            return impl_->open(prefix_ / resource_path);
        }

        std::string slurp(const ResourcePath& resource_path)
        {
            return impl_->slurp(prefix_ / resource_path);
        }

        ResourceLoader with_prefix(const ResourcePath& prefix) const
        {
            return ResourceLoader{impl_, prefix_ / prefix};
        }
        ResourceLoader with_prefix(std::string_view str)
        {
            return with_prefix(ResourcePath{str});
        }

        std::function<std::optional<ResourceDirectoryEntry>()> iterate_directory(const ResourcePath& resource_path)
        {
            return impl_->iterate_directory(prefix_ / resource_path);
        }
    private:
        template<std::derived_from<IResourceLoader> TResourceLoader, typename... Args>
        requires std::constructible_from<TResourceLoader, Args&&...>
        friend ResourceLoader make_resource_loader(Args&&...);

        explicit ResourceLoader(
            std::shared_ptr<IResourceLoader> impl,
            const ResourcePath& prefix = ResourcePath{}) :

            impl_{std::move(impl)},
            prefix_{prefix}
        {}

        std::shared_ptr<IResourceLoader> impl_;
        ResourcePath prefix_;
    };

    template<std::derived_from<IResourceLoader> TResourceLoader, typename... Args>
    requires std::constructible_from<TResourceLoader, Args&&...>
    ResourceLoader make_resource_loader(Args&&... args)
    {
        return ResourceLoader{std::make_shared<TResourceLoader>(std::forward<Args>(args)...)};
    }
}
