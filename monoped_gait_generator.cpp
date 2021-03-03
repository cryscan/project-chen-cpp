//
// Created by cryscan on 3/3/21.
//

#include <cassert>

#include "monoped_gait_generator.h"

towr::GaitGenerator::GaitInfo MonopedGaitGenerator::GetGait(towr::GaitGenerator::Gaits gait) const {
    if (gait == Hop1) {
        auto times = {0.1, 0.3, 0.1};
        auto contacts = {o_, x_, o_};
        return std::make_pair(times, contacts);
    }
    assert(false);
}

void MonopedGaitGenerator::SetCombo(towr::GaitGenerator::Combos combo) {
    if (combo == C0)
        SetGaits({Hop1, Hop1, Hop1, Hop1, Hop1});
    else
        assert(false);
}
