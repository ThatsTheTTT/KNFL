

# KNLF Image Format

KNLF is a memory-efficient lossless image compression format optimized for a low RAM footprint and line-by-line decoding. It is designed for embedded systems, microcontrollers, and low-resource environments.

This repository contains the C++ reference encoder and decoder library.

---

## Key Features

* Low RAM Footprint: Operates without large lookup tables during decoding, keeping memory usage minimal.
* Streaming Pipeline: Supports line-by-line decoding directly from a byte stream.
* Bit-Level Packing: Combines Paeth spatial prediction with bitstream packing to process pixel data efficiently.

---

## Benchmark Comparison

| Format | File Size Density | Decoding RAM Usage | Main Advantage |
| :--- | :--- | :--- | :--- |
| **KNLF** | Moderate | Minimal (< 1 KB) | Streams line-by-line, tiny RAM footprint |
| **QOI** | Compact | Minimal (< 1 KB) | Blazing fast decoding speed |
| **PNG** | Very Compact | > 32 KB (Window + Huffman) | High compression density, industry standard |

---

## Building the Library

### Prerequisites
* C++17 compliant compiler (GCC, Clang, MSVC)

### Build Steps

## Using the Library

KNLF is a header-only library — no build step required.

### Prerequisites
- C++17 compliant compiler (GCC, Clang, MSVC)

### Usage
Copy `knlf.hpp` into your project and include it:

```bash
git clone https://github.com/ThatsTheTTT/Kinda-Nice-Lossless-Format..git
```

```cpp
#include "knlf.hpp"
```
