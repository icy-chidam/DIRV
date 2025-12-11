int parse_tlv_frame(uint8_t *buf, uint32_t len, radar_frame_t *frame)
{
    int header_idx = find_magic_word(buf, len);
    if (header_idx < 0) return -1;

    uint32_t *hdr = (uint32_t *)(buf + header_idx);

    uint32_t packet_len     = u32((uint8_t*)&hdr[3]);
    uint32_t num_obj        = u32((uint8_t*)&hdr[7]);
    uint32_t num_tlv        = u32((uint8_t*)&hdr[8]);
    uint32_t frame_number   = u32((uint8_t*)&hdr[5]);

    frame->frame_number = frame_number;
    frame->det_count    = num_obj;

    uint32_t tlv_ptr = header_idx + 40;

    for (uint32_t i=0; i<num_tlv; i++)
    {
        uint32_t tlv_type = u32(&buf[tlv_ptr]);
        uint32_t tlv_len  = u32(&buf[tlv_ptr+4]);

        if (tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS)
        {
            uint32_t offset = tlv_ptr + 8;

            for (uint32_t k=0; k<num_obj; k++)
            {
                radar_obj_t *obj = &frame->objects[k];

                obj->x = bytes_to_float(&buf[offset]);
                obj->y = bytes_to_float(&buf[offset+4]);
                obj->z = bytes_to_float(&buf[offset+8]);
                obj->v = bytes_to_float(&buf[offset+12]);

                obj->range = sqrtf(obj->x*obj->x + obj->y*obj->y + obj->z*obj->z);

                obj->azimuth = (obj->y == 0) ? 90 :
                    atan2f(obj->x, obj->y) * 180 / 3.14159f;

                obj->elev = atan2f(obj->z,
                                sqrtf(obj->x*obj->x + obj->y*obj->y)) * 180 / 3.14159f;

                offset += 16;
            }
        }

        // TLV Type 7: SNR + Noise
        else if (tlv_type == 7)
        {
            uint32_t offset = tlv_ptr + 8;

            for (uint32_t k=0; k<num_obj; k++)
            {
                frame->objects[k].snr   = u16(&buf[offset]);
                frame->objects[k].noise = u16(&buf[offset+2]);
                offset += 4;
            }
        }

        tlv_ptr += tlv_len;
    }

    return 0;  // success
}
