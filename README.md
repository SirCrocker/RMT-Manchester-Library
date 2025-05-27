# kronecker-chenc-for-arduino

- [About](#about)
- [Installation](#installation)
- [Usage](#usage)
- [Support](#support)
- [License](#license)
- [More information](#more-information)

## About

kronecker-chenc-for-arduino is an Arduino library that implements the channel encoding algorithm described in [the paper by F. Asim et al.](https://ieeexplore.ieee.org/document/9146283) for the case of BPSK. The implementation assumes the channel is a binary symmetric channel, that only digital data is sent and received, and that the packets' smallest unit is bytes (8 bits).

Currently, only TPMD-4 with Scheme 2 is implemented, and the decoder implemented is the rank-one detector which has a coding rate of 1/4 due to the need of training symbols.

## Installation

Drag and drop the files `kronecker-chenc.cc` and `kronecker-chenc.h` to the directory where your libraries are installed.

## Usage

A basic usage example is shown in the next subsection.

### Transmitter

```c++
#include <Arduino.h>
#include "kronecker-chenc.h"

#define TRAINING_SYMBOLS 0b10101010

void setup() {
    // Begin serial monitor
    Serial.begin(115200);
    
    /* Do other stuff */
}

void loop() {
    uint8_t msg_buffer[1];
    // Check for messages
    if (Serial.readBytesUntil(0x00, msg_buffer, 1) > 0) {
        uint8_t encoded[2] = encode_kronecker_tpmd4s2(msg_buffer[0]);

        uint8_t decoded = rank_one_detector_tpmd4s2(encoded[0], encoded[1], TRAINING_SYMBOLS);
        Serial.println(decoded, BIN);
    }

    /* Do other stuff */
}
```

## Support

For support or submitting bugs open an issue, I'll check them as soon as I am available.

## License

kronecker-chenc-for-arduino is licensed under the [MIT](./LICENSE) license.

## More Information

For more information about the algorithm and its performance read the original paper by Fazal E-Asim et al. at https://ieeexplore.ieee.org/document/9146283.

### Implementation details 

- Since the modulation with Manchester/BPSK is a bijective function, the encoding is applied in the soon-to-be-sent bytes, not on the manchester modulated ones (contrary to what is done on the original paper).
- When using BPSK each symbol is directly linked to 1 bit of information, this makes it possible to use bytes to represent vectors of 8 symbols. The code uses this idea and does every calculation using only bytes (`uint8_t`/`char`/`int8_t`); however, BPSK symbols have values in {-1, 1} and bits in {0, 1}, so the transformation $F(•)$ is applied where

    - $F(-1) = 0$
    - $F(1)  = 1$
    - $F(a * b) = F(a) \odot F(b) \quad a,b \in \\{-1, 1\\}$.

    The product is replaced by the negation of the XOR product of the transformed symbols, which has the following truth table and allows to maintain the property that $-1 * -1 = 1$.

    | A | B | XNOR |
    | :-: | :-: | :--------: |
    | 0   | 0   | 1          |
    | 0   | 1   | 0          |
    | 1   | 0   | 0          |
    | 1   | 1   | 1          |

    **Table 1:** truth table of the XNOR operation
