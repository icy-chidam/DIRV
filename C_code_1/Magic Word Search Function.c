int find_magic_word(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len - MAGIC_WORD_LEN; i++)
    {
        if (memcmp(&buf[i], magic_word, MAGIC_WORD_LEN) == 0)
            return i;
    }
    return -1;
}
