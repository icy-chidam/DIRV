void process_radar_uart()
{
    radar_frame_t frame;

    if (uart_buf_len < MAGIC_WORD_LEN) return;

    if (parse_tlv_frame(uart_buffer, uart_buf_len, &frame) == 0)
    {
        // You NOW have parsed objects in frame.objects[]
        for (int i = 0; i < frame.det_count; i++)
        {
            radar_obj_t *o = &frame.objects[i];

            radar_feature_t f;
            f.range   = o->range;
            f.velocity = o->v;
            f.snr      = (float)o->snr;
            f.azimuth  = o->azimuth;

            uint8_t cls = classify_object(f);

            // Send to PWM / CAN / LED etc
            handle_detection(cls, f.range);
        }
    }

    // Clear buffer after processing
    uart_buf_len = 0;
}
