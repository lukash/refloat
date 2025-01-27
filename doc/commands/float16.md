# float16 Floating Point Number Representation

`float16` is a 16-bit floating point representation compatible with the IEEE-754 FP16 format, with the difference that it doesn't support the `NaN` and `Inf` special numbers and instead uses the bit for doubling its range of representable numbers.

It's used on the package interface to represent floating point numbers in a generic way at half the size of the standard 32-bit float, to save on bandwidth. Pretty much all realtime values in the package can be encoded in this representation without a significant precision impact (in case of needing very low or very high numbers, they should be scaled accordingly at the source).

The format and the code are adopted from https://stackoverflow.com/a/60047308.

Characteristics:
- Dynamic range: +-131008.0
- Normal numbers closest to zero: +-6.1035156E-5
- Subnormal numbers closest to zero: +-5.9604645E-8
- Precision: 3.311 decimal digits

JavaScript snippet for decoding the `float16` representation encoded in an integer into a floating point value:

```js
function float16_to_float(number) {
    var e = (number&0x7C00)>>10; // exponent
    var m16 = number&0x03FF; // original float16 mantissa
    var m = m16<<13; // new float 32 mantissa
    var v = m16>0 ? 140+Math.floor(Math.log2(m16)) : 0;
    // sign | normalized | denormalized
    var r = (number&0x8000)<<16 | (e!=0)*((e+112)<<23|m) | ((e==0)&(m!=0))*((v-37)<<23|((m<<(150-v))&0x007FE000));
    return new Float32Array(new Uint32Array([r]).buffer)[0];
}
```
