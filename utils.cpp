//
// Created by cryscan on 3/21/21.
//

#include <bitset>
#include <towr/nlp_formulation.h>

#include "utils.h"


using namespace towr;

std::vector<int> convert_bound_dims(uint8_t bounds) {
    std::bitset<6> bitset(bounds);
    std::vector<int> result;
    for (auto dim: {AX, AY, AZ, LX, LY, LZ})
        if (bitset[dim]) result.push_back(dim);
    return result;
}