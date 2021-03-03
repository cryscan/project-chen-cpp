//
// Created by cryscan on 3/3/21.
//

#ifndef PLAYGROUND_MONOPED_GAIT_GENERATOR_H
#define PLAYGROUND_MONOPED_GAIT_GENERATOR_H

#include <towr/initialization/gait_generator.h>

class MonopedGaitGenerator : public towr::GaitGenerator {
    [[nodiscard]] GaitInfo GetGait(Gaits gait) const override;

    ContactState o_ = ContactState(1, true);  // stance
    ContactState x_ = ContactState(1, false); // flight

    void SetCombo(Combos combo) override;
};

#endif //PLAYGROUND_MONOPED_GAIT_GENERATOR_H
