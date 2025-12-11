#include <stdio.h>
#include <stdint.h>

// Class labels
#define CLASS_OBJECT     0
#define CLASS_VEHICLE    1
#define CLASS_PEDESTRIAN 2

// Input feature structure
typedef struct {
    float range;       // meters
    float velocity;    // m/s
    float snr;         // Signal-to-noise ratio
    float azimuth;     // angle in degrees
} radar_feature_t;

// Decision Tree Inference Function
uint8_t classify_object(radar_feature_t f)
{
    // Level 1 split on Range
    if (f.range < 15.0)
    {
        // Level 2 split on Velocity
        if (f.velocity < 1.5)
        {
            // Level 3 split on SNR
            if (f.snr < 8.0)
            {
                return CLASS_OBJECT;
            }
            else
            {
                return CLASS_PEDESTRIAN;
            }
        }
        else
        {
            return CLASS_VEHICLE;
        }
    }
    else
    {
        // Far objects
        if (f.velocity > 2.0)
        {
            return CLASS_VEHICLE;
        }
        else
        {
            return CLASS_OBJECT;
        }
    }
}
