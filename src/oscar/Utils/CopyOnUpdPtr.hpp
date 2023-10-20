#pragma once

#include <memory>
#include <utility>

namespace osc
{
    template<typename T>
    class CopyOnUpdPtr final {
    private:
        template<typename U, typename... Args>
        friend CopyOnUpdPtr<U> make_cow(Args&&... args);

        explicit CopyOnUpdPtr(std::shared_ptr<T> p) :
            m_Ptr{std::move(p)}
        {
        }
    public:

        T const* get() const noexcept
        {
            return m_Ptr.get();
        }

        T* upd()
        {
            if (m_Ptr.use_count() > 1)
            {
                m_Ptr = std::make_shared<T>(*m_Ptr);
            }
            return m_Ptr.get();
        }

        T const& operator*() const noexcept
        {
            return *get();
        }

        T const* operator->() const noexcept
        {
            return get();
        }

        friend void swap(CopyOnUpdPtr& a, CopyOnUpdPtr& b) noexcept
        {
            swap(a.m_Ptr, b.m_Ptr);
        }

        template<typename U>
        friend bool operator==(CopyOnUpdPtr const& lhs, CopyOnUpdPtr<U> const& rhs) noexcept
        {
            return lhs.m_Ptr == rhs.m_Ptr;
        }

        template<typename U>
        friend bool operator!=(CopyOnUpdPtr const& lhs, CopyOnUpdPtr<U> const& rhs) noexcept
        {
            return lhs.m_Ptr != rhs.m_Ptr;
        }

        template<typename U>
        friend bool operator<(CopyOnUpdPtr const& lhs, CopyOnUpdPtr<U> const& rhs) noexcept
        {
            return lhs.m_Ptr < rhs.m_Ptr;
        }

        template<typename U>
        friend bool operator>(CopyOnUpdPtr const& lhs, CopyOnUpdPtr<U> const& rhs) noexcept
        {
            return lhs.m_Ptr > rhs.m_Ptr;
        }

        template<typename U>
        friend bool operator<=(CopyOnUpdPtr const& lhs, CopyOnUpdPtr<U> const& rhs) noexcept
        {
            return lhs.m_Ptr <= rhs.m_Ptr;
        }

        template<typename U>
        friend bool operator>=(CopyOnUpdPtr const& lhs, CopyOnUpdPtr<U> const& rhs) noexcept
        {
            return lhs.m_Ptr >= rhs.m_Ptr;
        }

    private:
        friend struct std::hash<CopyOnUpdPtr>;

        std::shared_ptr<T> m_Ptr;
    };

    template<typename T, typename... Args>
    CopyOnUpdPtr<T> make_cow(Args&&... args)
    {
        return CopyOnUpdPtr<T>(std::make_shared<T>(std::forward<Args>(args)...));
    }
}

template<typename T>
struct std::hash<osc::CopyOnUpdPtr<T>> final {
    size_t operator()(osc::CopyOnUpdPtr<T> const& cow) const
    {
        return std::hash<std::shared_ptr<T>>{}(cow.m_Ptr);
    }
};
