static inline uint32_t u32(uint8_t *d)
{
    return (d[0]) | (d[1]<<8) | (d[2]<<16) | (d[3]<<24);
}

static inline uint16_t u16(uint8_t *d)
{
    return (d[0]) | (d[1]<<8);
}

float bytes_to_float(uint8_t *d)
{
    float val;
    memcpy(&val, d, 4);
    return val;
}
