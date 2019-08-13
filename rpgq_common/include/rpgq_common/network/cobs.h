#pragma once

#include <cstdint>
#include <cstdlib>
#include <utility>
#include <tuple>

// consistent overhead byte stuffing

namespace RPGQ
{
    namespace COBS
    {
        const uint8_t COBS_START_SYMBOL             = 0x00;

        const uint8_t COBS_ENCODE_OK                = 0x00;
        const uint8_t COBS_ENCODE_NULLPOINTER       = 0x01;
        const uint8_t COBS_ENCODE_BUFFER_OVERFLOW   = 0x02;

        const uint8_t COBS_DECODE_OK                = 0x00;
        const uint8_t COBS_DECODE_NULLPOINTER       = 0x01;
        const uint8_t COBS_DECODE_BUFFER_OVERFLOW   = 0x02;
        const uint8_t COBS_DECODE_ZERO_INPUT        = 0x04;
        const uint8_t COBS_DECODE_INPUT_TOO_SHORT   = 0x08;

    } // namespace COBS

    std::pair<uint8_t, size_t> COBSEncode(const uint8_t* src, const size_t lengthSrc, uint8_t* dst, const size_t lengthDst);
    std::pair<uint8_t, size_t> COBSDecode(const uint8_t* src, const size_t lengthSrc, uint8_t* dst, const size_t lengthDst);

} // namespace RPGQ