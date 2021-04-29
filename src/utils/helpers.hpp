#pragma once

#include <filesystem>
#include <string>

namespace osc {
    inline int num_bits_set_in(int v) {
        unsigned uv = static_cast<unsigned>(v);
        unsigned i = 0;
        while (uv) {
            uv &= (uv - 1);
            ++i;
        }
        return static_cast<int>(i);
    }

    inline int lsb_index(int v) {
        unsigned uv = static_cast<unsigned>(v);
        unsigned i = 0;
        while (!(uv & 0x1)) {
            uv >>= 1;
            ++i;
        }
        return static_cast<int>(i);
    }

    std::string slurp_into_string(std::filesystem::path const&);
}
