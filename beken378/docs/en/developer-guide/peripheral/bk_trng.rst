
:link_to_translation:`zh_CN:[中文]`

TRNG user guide
=====================


Overview
-----------------

- The true random number generator (TRNG), integrated inside the Beken chip, generates true random numbers through random noise and does not depend on other modules.
- The TRNG is used to create keys, initialization vectors and random numbers required for cryptographic operations.

How to use TRNG
------------------

 - To obtain a random number, call bk_rand().

