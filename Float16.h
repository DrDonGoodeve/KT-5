// Float16 class
/// Reduced implementation - simply converts to/from 16-bit float from
/// 32-bit float.
//-----------------------------------------------------------------------------
class float16 {
    private:
        typedef union {
            uint16_t uWord16;
            uint8_t uByte[2];
            struct {
                uint16_t frac : 10;	    // mantissa
                uint16_t exp : 5;		// exponent
                uint16_t sign : 1;		// sign
            }IEEE;
        }F16;

        F16 mcValue;

    public:
        float16(float fValue) {
            float fFraction(0.0f);
            int iExponent(0);
            fFraction = std::frexp(fValue, &iExponent);
            mcValue.IEEE.sign = (fValue < 0) ? 1 : 0;

            if (true == isnan(fValue)) {    // Not a number
                mcValue.IEEE.frac = 0x1;
                mcValue.IEEE.exp = 0x1f;

            }
            else if (iExponent > 15) {
                mcValue.IEEE.frac = 0x0;    // Infinity (out of range)
                mcValue.IEEE.exp = 0x1f;

            }
            else if (iExponent <= -15) {  // Sub-normal numbers
                mcValue.IEEE.exp = 0x0;
                mcValue.IEEE.frac = (uint16_t)roundf(fFraction*65535.0f);

            }
            else {    // Normal numbers - implicit leading '1' for fractional part, unless exp is 00000
                float fEncodedFraction((fFraction*2.0f) - 1.0f);
                mcValue.IEEE.frac = (uint16_t)roundf(fEncodedFraction*(float)(1<<10));
                mcValue.IEEE.exp = (uint16_t)(iExponent + 15);
            }
        }

        float getFloat32(void) const {
            if (0x1f == mcValue.IEEE.exp) {
                // Handle non-finite results
                if (0 == mcValue.IEEE.frac) {
                    return std::numeric_limits<float>::infinity();
                }
                else {
                    return std::numeric_limits<float>::quiet_NaN();
                }
            }
            else {
                if (0x0 == mcValue.IEEE.exp) {  // Subnormal
                    return std::ldexp(((float)mcValue.IEEE.frac / (float)((1 << 10) - 1)), -15);
                }
                else {    // Normal
                    float fEncodedFraction((float)mcValue.IEEE.frac / (float)(1 << 10));
                    float fFraction((fEncodedFraction + 1.0f) / 2.0f);
                    return std::ldexp(fFraction, (int)mcValue.IEEE.exp - 15);
                }
            }
        }

        float16(uint8_t uByte1, uint8_t uByte2) {
            mcValue.uByte[0] = uByte1;
            mcValue.uByte[1] = uByte2;
        }

        void getBytes(uint8_t &uByte1, uint8_t &uByte2) {
            uByte1 = mcValue.uByte[0];
            uByte2 = mcValue.uByte[1];
        }
};