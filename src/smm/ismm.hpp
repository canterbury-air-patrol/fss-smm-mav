#pragma once

#include "../fmu-core-types.hpp"

class ISMM {
public:
    ISMM() = default;
    ISMM(const ISMM&) = delete;
    ISMM(ISMM&&) = delete;
    auto operator=(const ISMM&) -> ISMM& = delete;
    auto operator=(ISMM&&) -> ISMM& = delete;
    virtual ~ISMM() = default;

    virtual void search(Point current_pos) = 0;
};
