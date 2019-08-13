#include <rpgq_common/network/cobs.h>

using namespace RPGQ;

std::pair<uint8_t, size_t> RPGQ::COBSEncode(const uint8_t* src, const size_t lengthSrc, uint8_t* dst, const size_t lengthDst)
{
    const uint8_t* endSrc = src + lengthSrc;
    const uint8_t* startDst = dst; 
    uint8_t* endDst = dst + lengthDst;
    uint8_t *code = dst++, codeByte = 1;
    uint8_t srcByte = 0;

    // null pointer check
    if (src == nullptr || dst == nullptr)
    {
        return std::make_pair<uint8_t, size_t>((uint8_t) COBS::COBS_ENCODE_NULLPOINTER, 0);
    }

    // process data
    uint8_t ret = 0;
    if (lengthSrc != 0)
    {
        while (1)
        {
            // check for buffer overflow
            if (!(dst < endDst))
            {
                ret = COBS::COBS_ENCODE_BUFFER_OVERFLOW;
                break;
            }

            srcByte = *src++;
            if (srcByte == COBS::COBS_START_SYMBOL)
            {
                // found zero byte
                *code = codeByte;

                code = dst++;
                codeByte = 1;

                if (!(src < endSrc))
                {
                    // reached end
                    break;
                }
            }
            else
            {
                // found non-zero byte
                *dst++ = srcByte;
                
                codeByte++;
                if (codeByte == 0xFF)
                {
                    // too long of non-zero bytes
                    *code = codeByte;

                    code = dst++;
                    codeByte = 1;
                }

                if (!(src < endSrc))
                {
                    // reached end
                    break;
                }
            }
        }
    }

    // finish processing data
    if (dst < endDst)
    {
        // write last code (length) byte
        *code = codeByte;
    }
    else
    {
        ret = COBS::COBS_ENCODE_BUFFER_OVERFLOW;
        dst = endDst;
    }

    return std::make_pair<uint8_t, size_t>((uint8_t) ret, dst - startDst);
}

std::pair<uint8_t, size_t> RPGQ::COBSDecode(const uint8_t* src, const size_t lengthSrc, uint8_t* dst, const size_t lengthDst)
{
    const uint8_t* endSrc = src + lengthSrc;
    const uint8_t* startDst = dst; 
    const uint8_t* endDst = dst + lengthDst;
    uint8_t codeByte;
    uint8_t srcByte;
    size_t remainingBytes;

    // null pointer check
    if (src == nullptr || dst == nullptr)
    {
        return std::make_pair<uint8_t, size_t>((uint8_t) COBS::COBS_DECODE_NULLPOINTER, 0);
    }

    // process data
    uint8_t ret = 0;
    if (lengthSrc != 0)
    {
        while (1)
        {
            codeByte = *src++;
            if (codeByte == COBS::COBS_START_SYMBOL)
            {
                ret = COBS::COBS_DECODE_ZERO_INPUT;
                break;
            }
            codeByte--;

            // check code byte against remaining bytes
            remainingBytes = endSrc - src;
            if (codeByte > remainingBytes)
            {
                ret |= COBS::COBS_DECODE_INPUT_TOO_SHORT;
                codeByte = (uint8_t) remainingBytes;
            }

            // check code byte against remaining output buffer space
            remainingBytes = endDst - dst;
            if (codeByte > remainingBytes)
            {
                ret |= COBS::COBS_DECODE_BUFFER_OVERFLOW;
                codeByte = (uint8_t) remainingBytes;
            } 

            // check for buffer overflow
            if (!(dst < endDst))
            {
                ret |= COBS::COBS_ENCODE_BUFFER_OVERFLOW;
                break;
            }

            // copy all bytes until next code byte
            for (uint8_t i = codeByte; i != 0; i--)
            {
                srcByte = *src++;
                if (srcByte == COBS::COBS_START_SYMBOL)
                {
                    ret |= COBS::COBS_DECODE_ZERO_INPUT;
                }
                *dst++ = srcByte;
            }

            if (!(src < endSrc))
            {
                // reached end
                break;
            }

            // append zero
            if (codeByte != 0xFE)
            {
                if (dst < endDst)
                {
                    *dst++ = 0;
                }
                else
                {
                    ret |= COBS::COBS_DECODE_BUFFER_OVERFLOW;
                    break;
                }
            }
        }
    }

    // finish
    return std::pair<uint8_t, size_t>((uint8_t) ret, dst - startDst);
}