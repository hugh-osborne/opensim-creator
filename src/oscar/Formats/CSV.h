#pragma once

#include <iosfwd>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace osc
{
    // returns a vector of column data if a row could be read; otherwise, returns std::nullopt
    std::optional<std::vector<std::string>> readCSVRow(
        std::istream&
    );

    // returns `true` if a row was read from the input and written as columns to the output
    bool readCSVRowIntoVector(
        std::istream&,
        std::vector<std::string>& overwriteOut
    );

    // writes the given columns to the output stream as an ASCII encoded text row
    void writeCSVRow(
        std::ostream&,
        std::span<const std::string> columns
    );
}
