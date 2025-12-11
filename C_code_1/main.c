#include <stdint.h>
#include <stdio.h>

#include "classify_model.h"

int main() {
    radar_feature_t f;

    f.range = 12.0;
    f.velocity = 1.2;
    f.snr = 9.0;
    f.azimuth = 5.0;

    uint8_t result = classify_object(f);

    return 0;
}
