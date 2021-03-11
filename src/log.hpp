#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace osmv::log {
    // this implementation is a gruesome simplification of spdlog: go read spdlog
    // sources if you want to see good software engineering

    namespace level {
        enum Level_enum { trace = 0, debug, info, warn, err, critical, off };
    }
#define OSMV_LOG_LVL_NAMES                                                                                             \
    { "trace", "debug", "info", "warning", "error", "critical", "off" }

    [[nodiscard]] std::string_view const& to_string_view(level::Level_enum) noexcept;
    [[nodiscard]] char const* to_c_str(level::Level_enum) noexcept;

    struct Log_msg final {
        std::string_view logger_name;
        std::chrono::system_clock::time_point time;
        std::string_view payload;
        level::Level_enum level;

        Log_msg() = default;
        Log_msg(std::string_view _logger_name, std::string_view _payload, level::Level_enum _level) :
            logger_name{_logger_name},
            time{std::chrono::system_clock::now()},
            payload{_payload},
            level{_level} {
        }
    };

    class Sink {
        level::Level_enum level_{level::trace};

    public:
        virtual ~Sink() noexcept = default;
        virtual void log(Log_msg const&) = 0;

        void set_level(level::Level_enum level) noexcept {
            level_ = level;
        }

        [[nodiscard]] level::Level_enum level() const noexcept {
            return level_;
        }

        [[nodiscard]] bool should_log(level::Level_enum level) const noexcept {
            return level >= level_;
        }
    };

    class Logger final {
        std::string name;
        std::vector<std::shared_ptr<Sink>> _sinks;
        level::Level_enum level{level::trace};

    public:
        Logger(std::string _name) : name{std::move(_name)}, _sinks() {
        }

        Logger(std::string _name, std::shared_ptr<Sink> _sink) : name{std::move(_name)}, _sinks{_sink} {
        }

        template<typename... Args>
        void log(level::Level_enum msg_level, char const* fmt, Args const&... args) {
            if (msg_level < level) {
                return;
            }

            // create the log message
            char buf[255];
            size_t n = std::snprintf(buf, sizeof(buf), fmt, args...);
            Log_msg msg{name, std::string_view{buf, n}, msg_level};

            // sink it
            for (auto& sink : _sinks) {
                if (sink->should_log(msg.level)) {
                    sink->log(msg);
                }
            }
        }

        template<typename... Args>
        void trace(char const* fmt, Args const&... args) {
            log(level::trace, fmt, args...);
        }

        template<typename... Args>
        void debug(char const* fmt, Args const&... args) {
            log(level::info, fmt, args...);
        }

        template<typename... Args>
        void info(char const* fmt, Args const&... args) {
            log(level::info, fmt, args...);
        }

        template<typename... Args>
        void warn(char const* fmt, Args const&... args) {
            log(level::warn, fmt, args...);
        }

        template<typename... Args>
        void error(char const* fmt, Args const&... args) {
            log(level::err, fmt, args...);
        }

        template<typename... Args>
        void critical(char const* fmt, Args const&... args) {
            log(level::critical, fmt, args...);
        }

        [[nodiscard]] std::vector<std::shared_ptr<Sink>> const& sinks() const noexcept {
            return _sinks;
        }

        [[nodiscard]] std::vector<std::shared_ptr<Sink>>& sinks() noexcept {
            return _sinks;
        }
    };

    // global logging API
    std::shared_ptr<Logger> default_logger() noexcept;
    Logger* default_logger_raw() noexcept;

    template<typename... Args>
    inline void log(level::Level_enum level, char const* fmt, Args const&... args) {
        default_logger_raw()->log(level, fmt, args...);
    }

    template<typename... Args>
    inline void trace(char const* fmt, Args const&... args) {
        default_logger_raw()->trace(fmt, args...);
    }

    template<typename... Args>
    inline void debug(char const* fmt, Args const&... args) {
        default_logger_raw()->debug(fmt, args...);
    }

    template<typename... Args>
    void info(char const* fmt, Args const&... args) {
        default_logger_raw()->info(fmt, args...);
    }

    template<typename... Args>
    void warn(char const* fmt, Args const&... args) {
        default_logger_raw()->warn(fmt, args...);
    }

    template<typename... Args>
    void error(char const* fmt, Args const&... args) {
        default_logger_raw()->error(fmt, args...);
    }

    template<typename... Args>
    void critical(char const* fmt, Args const&... args) {
        default_logger_raw()->critical(fmt, args...);
    }
}
