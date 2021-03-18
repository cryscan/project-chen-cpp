//
// Created by cryscan on 3/18/21.
//

#ifndef PLAYGROUND_SPATIAL_HASH_H
#define PLAYGROUND_SPATIAL_HASH_H

#include <cstdint>
#include <vector>


void hilbert_index_to_xy(uint32_t n, uint32_t i, uint32_t& x, uint32_t& y);
uint32_t hilbert_xy_to_index(uint32_t n, uint32_t x, uint32_t y);

uint32_t morton_to_hilbert_3d(uint32_t morton_index, uint32_t bits);
uint32_t hilbert_to_morton_3d(uint32_t hilbert_index, uint32_t bits);


template<typename T>
class HilbertHash {
    uint order;
    std::vector<T> data;

    static constexpr auto order_of = [](uint x) {
        uint y = x;
        uint r = 0;
        while (y >>= 1) ++r;
        if (x > (1 << r)) r += 1;
        return r;
    };

public:
    HilbertHash(uint x, uint y, T t = T())
            : order(std::max(order_of(x), order_of(y))),
              data((1 << 2 * order), t) {
    }

    const T& operator()(uint x, uint y) const {
        auto index = hilbert_xy_to_index(order, x, y);
        return data.at(index);
    }

    T& operator()(uint x, uint y) {
        auto index = hilbert_xy_to_index(order, x, y);
        return data.at(index);
    }
};

#endif //PLAYGROUND_SPATIAL_HASH_H
