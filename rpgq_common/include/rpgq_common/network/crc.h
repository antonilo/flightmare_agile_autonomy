#pragma once

#include <cstdio>
#include <cstdint>

namespace RPGQ
{
    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue,
        bool reflectData, bool reflectRemainder>
    class BitwiseCRC
    {
    public:
        // constructor & destructor
        BitwiseCRC(void);
        virtual ~BitwiseCRC(void);

        // public get functions
        inline uint8_t GetChecksumLength(void);
        inline virtual CRC GetChecksum(const uint8_t* src, const size_t length);

    protected:
        // general CRC checksum variables
        const uint8_t crcLength_;
        const CRC topBit_;
    };

    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue,
        bool reflectData, bool reflectRemainder>
    class BitwiseCRCWithLookUpTable : public BitwiseCRC<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>
    {
    public:
        // constructor & destructor
        BitwiseCRCWithLookUpTable(void);
        virtual ~BitwiseCRCWithLookUpTable(void);

        // public get functions
        inline virtual CRC ComputeChecksum(const uint8_t* src, const size_t length);

    private:
        // general CRC with look-up table variables & functions
        CRC crcTable_[256];
        inline void InitializeTable(void);
    };


    // auxiliary function
    inline uint32_t Reflect(uint32_t data, uint8_t length){

        uint32_t  reflection = 0x00000000;

        // reflect the data about the center bit
        for (uint8_t bit = 0; bit < length; ++bit){

            // if LSB bit is set, set the reflection of it
            if (data & 0x01){
                reflection |= (1 << ((length - 1) - bit));
            }

            data = (data >> 1);
        }

        return (reflection);
    };


    // Bitwise CRC functions
    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue, bool reflectData, bool reflectRemainder>
    BitwiseCRC<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>::BitwiseCRC(void):
        crcLength_(8 * sizeof(CRC)),
        topBit_(1 << (crcLength_ - 1))
    {};

    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue, bool reflectData, bool reflectRemainder>
    BitwiseCRC<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>::~BitwiseCRC(void)
    {};

    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue, bool reflectData, bool reflectRemainder>
    uint8_t BitwiseCRC<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>::GetChecksumLength(void)
    {
        return sizeof(CRC);  
    };

    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue, bool reflectData, bool reflectRemainder>
    CRC BitwiseCRC<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>::GetChecksum(const uint8_t* src, const size_t length)
    {
        CRC remainder = initialRemainder;

        // perform bytewise modulo-2 division
        for (size_t idx = 0; idx < length; idx++)
        {
            // bring next byte into remainder
            if (reflectData == false)
            {
                remainder ^= (src[idx] << (crcLength_ - 8));
            }
            else
            {
                remainder ^= (Reflect(src[idx], 8) << (crcLength_ -8));
            }

            // perform bitwise modulo-2 division
            for (uint8_t bit = 8; bit > 0; bit--)
            {
                if (remainder & topBit_)
                {
                    remainder = (remainder << 1) ^ polynomial;
                }
                else
                {
                    remainder = (remainder << 1);
                }
            }
        }

        // return final remainder
        if (reflectRemainder == false)
        {
            return (remainder ^ finalXORValue);
        }
        else
        {
            return (Reflect(remainder, crcLength_) ^ finalXORValue);
        }
    };


    // Bitwise CRC with look-up table functions
    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue, bool reflectData, bool reflectRemainder>
    BitwiseCRCWithLookUpTable<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>::BitwiseCRCWithLookUpTable(void):
        BitwiseCRC<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>::BitwiseCRC()
    {
        InitializeTable();
    }

    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue, bool reflectData, bool reflectRemainder>
    BitwiseCRCWithLookUpTable<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>::~BitwiseCRCWithLookUpTable(void)
    {};

    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue, bool reflectData, bool reflectRemainder>
    CRC BitwiseCRCWithLookUpTable<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>::ComputeChecksum(const uint8_t* src, const size_t length)
    {
        CRC remainder = initialRemainder;

        // perform bytewise division with polynomial
        uint8_t data;
        for (size_t idx = 0; idx < length; idx++)
        {
            if (reflectData == false)
            {
                data = src[idx] ^ (remainder >> (this->crcLength_ - 8));
            }
            else
            {
                data = Reflect(src[idx], 8) ^ (remainder >> (this->crcLength_ - 8));
            }

            remainder = crcTable_[data] ^(remainder << 8);
        }

        // return final remainder
        if (reflectRemainder == false)
        {
            return (remainder ^ finalXORValue);
        }
        else
        {
            return (Reflect(remainder, this->crcLength_) ^ finalXORValue);
        }
    };

    template <typename CRC, CRC polynomial, CRC initialRemainder, CRC finalXORValue, bool reflectData, bool reflectRemainder>
    void BitwiseCRCWithLookUpTable<CRC, polynomial, initialRemainder, finalXORValue, reflectData, reflectRemainder>::InitializeTable(void)
    {
        CRC remainder;

        // compute remainder for each possible dividend
        for (unsigned int dividend = 0; dividend < 256; dividend++)
        {
            remainder = dividend << (this->crcLength_ - 8);

            for (uint8_t bit = 8; bit > 0; bit--)
            {
                if (remainder & this->topBit_)
                {
                    remainder = (remainder << 1) ^ polynomial;
                }
                else
                {
                    remainder = (remainder << 1);
                }
            }

            // store result into table
            crcTable_[dividend] = remainder;
        }
    };


    // specializations of the bit-wise crc algorithm
    typedef BitwiseCRC<uint16_t, 0x1021, 0xffff, 0x0000, false, false> CRC16_CCITT;
    typedef BitwiseCRCWithLookUpTable<uint16_t, 0x1021, 0xffff, 0x0000, false, false> CRC16_CCITT_Table;
    typedef BitwiseCRC<uint32_t, 0x04c11db7, 0xffffffff, 0xffffffff, false, false> CRC32_ANSI;
    typedef BitwiseCRCWithLookUpTable<uint32_t, 0x04c11db7, 0xffffffff, 0xffffffff, false, false> CRC32_ANSI_Table;
    typedef BitwiseCRC<uint32_t, 0xBA0DC66B, 0x00000000, 0x00000000, false, false> CRC32_Koopman;
    typedef BitwiseCRCWithLookUpTable<uint32_t, 0xBA0DC66B, 0x00000000, 0x00000000, false, false> CRC32_Koopman_Table;

} // namespace RPGQ