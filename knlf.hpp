#ifndef KNLF_HPP
#define KNLF_HPP

#include <iostream>
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>

#pragma pack(push, 1)
struct KNLFHeader {
    char magic[4];
    uint16_t width;
    uint16_t height;
    uint8_t channels;
    uint8_t flags;
};
#pragma pack(pop)

enum PredictorType : uint8_t {
    PRED_NONE = 0,
    PRED_SUB = 1,
    PRED_UP = 2,
    PRED_AVERAGE = 3,
    PRED_PAETH = 4
};

class StreamBitWriter {
private:
    std::ostream& out;
    uint8_t currentByte = 0;
    uint8_t bitPos = 0;

public:
    StreamBitWriter(std::ostream& os) : out(os) {}

    void writeBits(uint32_t val, uint8_t numBits) {
        for (int i = numBits - 1; i >= 0; --i) {
            uint8_t bit = (val >> i) & 1;
            currentByte |= (bit << (7 - bitPos));
            bitPos++;
            if (bitPos == 8) {
                out.put(static_cast<char>(currentByte));
                currentByte = 0;
                bitPos = 0;
            }
        }
    }

    void align() {
        if (bitPos > 0) {
            out.put(static_cast<char>(currentByte));
            currentByte = 0;
            bitPos = 0;
        }
    }
};

class StreamBitReader {
private:
    std::istream& in;
    uint8_t currentByte = 0;
    uint8_t bitPos = 8;

public:
    StreamBitReader(std::istream& is) : in(is) {}

    void align() {
        if (bitPos < 8) {
            bitPos = 8;
        }
    }

    bool readBit(uint8_t& outBit) {
        if (bitPos == 8) {
            int c = in.get();
            if (c == EOF) return false;
            currentByte = static_cast<uint8_t>(c);
            bitPos = 0;
        }
        outBit = (currentByte >> (7 - bitPos)) & 1;
        bitPos++;
        return true;
    }

    bool readBits(uint32_t& outVal, uint8_t numBits) {
        outVal = 0;
        for (uint8_t i = 0; i < numBits; ++i) {
            uint8_t bit = 0;
            if (!readBit(bit)) return false;
            outVal = (outVal << 1) | bit;
        }
        return true;
    }
};

class KNLFStream {
private:
    static uint8_t paethPredictor(uint8_t a, uint8_t b, uint8_t c) {
        int p = (int)a + (int)b - (int)c;
        int pa = std::abs(p - (int)a);
        int pb = std::abs(p - (int)b);
        int pc = std::abs(p - (int)c);
        if (pa <= pb && pa <= pc) return a;
        if (pb <= pc) return b;
        return c;
    }

    static uint8_t getPredictor(PredictorType type, uint8_t left, uint8_t up, uint8_t upLeft) {
        switch (type) {
            case PRED_SUB:     return left;
            case PRED_UP:      return up;
            case PRED_AVERAGE: return (static_cast<uint16_t>(left) + up) / 2;
            case PRED_PAETH:   return paethPredictor(left, up, upLeft);
            default:           return 0;
        }
    }

    static uint8_t encodeZigZag(int8_t val) {
        return (val >= 0) ? static_cast<uint8_t>(val << 1)
                          : static_cast<uint8_t>((-static_cast<int>(val) << 1) - 1);
    }

    static int8_t decodeZigZag(uint8_t val) {
        return (val & 1) ? static_cast<int8_t>(-(static_cast<int>(val >> 1) + 1))
                         : static_cast<int8_t>(val >> 1);
    }

public:
    class Encoder {
    private:
        std::ostream& out;
        StreamBitWriter bw;
        uint16_t width, height;
        uint8_t channels;
        std::vector<uint8_t> prevRow;

    public:
        Encoder(std::ostream& outputStream, uint16_t w, uint16_t h, uint8_t ch)
            : out(outputStream), bw(outputStream), width(w), height(h), channels(ch) {
            
            out.write("KNLF", 4);
            uint8_t headerData[6];
            headerData[0] = static_cast<uint8_t>(width & 0xFF);
            headerData[1] = static_cast<uint8_t>(width >> 8);
            headerData[2] = static_cast<uint8_t>(height & 0xFF);
            headerData[3] = static_cast<uint8_t>(height >> 8);
            headerData[4] = channels;
            headerData[5] = 1;
            out.write(reinterpret_cast<char*>(headerData), 6);
            
            prevRow.resize(width * channels, 0);
        }

        bool encodeScanline(const uint8_t* currRow) {
            size_t stride = width * channels;
            
            PredictorType bestPred = PRED_NONE;
            uint64_t minCost = 0xFFFFFFFFFFFFFFFFULL;

            for (int p = 0; p <= 4; ++p) {
                PredictorType testPred = static_cast<PredictorType>(p);
                uint64_t cost = 0;
                for (size_t i = 0; i < stride; ++i) {
                    uint8_t left = (i >= channels) ? currRow[i - channels] : 0;
                    uint8_t up = prevRow[i];
                    uint8_t upLeft = (i >= channels) ? prevRow[i - channels] : 0;
                    uint8_t pred = getPredictor(testPred, left, up, upLeft);
                    
                    int8_t diff = static_cast<int8_t>(static_cast<uint8_t>(currRow[i] - pred));
                    cost += std::abs(static_cast<int>(diff));
                }
                if (cost < minCost) {
                    minCost = cost;
                    bestPred = testPred;
                }
            }

            std::vector<uint8_t> residualLine(stride);
            for (size_t i = 0; i < stride; ++i) {
                uint8_t left = (i >= channels) ? currRow[i - channels] : 0;
                uint8_t up = prevRow[i];
                uint8_t upLeft = (i >= channels) ? prevRow[i - channels] : 0;
                uint8_t pred = getPredictor(bestPred, left, up, upLeft);
                
                int8_t diff = static_cast<int8_t>(static_cast<uint8_t>(currRow[i] - pred));
                residualLine[i] = encodeZigZag(diff);
            }

            bw.writeBits(static_cast<uint32_t>(bestPred), 3);

            size_t idx = 0;
            while (idx < stride) {
                if (residualLine[idx] == 0) {
                    size_t zeroRun = 0;
                    while (idx + zeroRun < stride && residualLine[idx + zeroRun] == 0 && zeroRun < 64) {
                        zeroRun++;
                    }
                    bw.writeBits(0, 2);
                    bw.writeBits(static_cast<uint32_t>(zeroRun - 1), 6);
                    idx += zeroRun;
                } else {
                    if (residualLine[idx] <= 15) {
                        bw.writeBits(1, 2);
                        bw.writeBits(residualLine[idx], 4);
                    } else {
                        bw.writeBits(2, 2);
                        bw.writeBits(residualLine[idx], 8);
                    }
                    idx++;
                }
            }

            bw.align(); 
            std::copy(currRow, currRow + stride, prevRow.begin());
            return out.good();
        }
    };

    class Decoder {
    private:
        std::istream& in;
        StreamBitReader br;
        KNLFHeader header;
        std::vector<uint8_t> prevRow;
        bool validHeader = false;

    public:
        Decoder(std::istream& inputStream) : in(inputStream), br(inputStream) {
            char magic[4];
            if (in.read(magic, 4) && magic[0] == 'K' && magic[1] == 'N' && magic[2] == 'L' && magic[3] == 'F') {
                uint8_t headerData[6];
                if (in.read(reinterpret_cast<char*>(headerData), 6)) {
                    header.magic[0] = 'K'; 
                    header.magic[1] = 'N'; 
                    header.magic[2] = 'L'; 
                    header.magic[3] = 'F';
                    header.width = headerData[0] | (headerData[1] << 8);
                    header.height = headerData[2] | (headerData[3] << 8);
                    header.channels = headerData[4];
                    header.flags = headerData[5];
                    validHeader = true;
                    prevRow.resize(header.width * header.channels, 0);
                }
            }
        }

        bool isValid() const { return validHeader; }
        const KNLFHeader& getHeader() const { return header; }

        bool decodeScanline(uint8_t* outRow) {
            if (!validHeader) return false;

            size_t stride = header.width * header.channels;

            uint32_t predBits = 0;
            if (!br.readBits(predBits, 3)) return false;
            PredictorType predType = static_cast<PredictorType>(predBits);

            size_t idx = 0;
            while (idx < stride) {
                uint32_t mode = 0;
                if (!br.readBits(mode, 2)) return false;

                if (mode == 0) {
                    uint32_t runVal = 0;
                    if (!br.readBits(runVal, 6)) return false;
                    size_t run = runVal + 1;
                    if (idx + run > stride) return false;

                    for (size_t r = 0; r < run; ++r) {
                        uint8_t left = (idx >= header.channels) ? outRow[idx - header.channels] : 0;
                        uint8_t up = prevRow[idx];
                        uint8_t upLeft = (idx >= header.channels) ? prevRow[idx - header.channels] : 0;
                        uint8_t pred = getPredictor(predType, left, up, upLeft);
                        outRow[idx] = pred;
                        idx++;
                    }
                } else if (mode == 1) {
                    uint32_t val = 0;
                    if (!br.readBits(val, 4)) return false;
                    int8_t diff = decodeZigZag(static_cast<uint8_t>(val));

                    uint8_t left = (idx >= header.channels) ? outRow[idx - header.channels] : 0;
                    uint8_t up = prevRow[idx];
                    uint8_t upLeft = (idx >= header.channels) ? prevRow[idx - header.channels] : 0;
                    uint8_t pred = getPredictor(predType, left, up, upLeft);
                    outRow[idx] = static_cast<uint8_t>(pred + diff);
                    idx++;
                } else if (mode == 2) {
                    uint32_t val = 0;
                    if (!br.readBits(val, 8)) return false;
                    int8_t diff = decodeZigZag(static_cast<uint8_t>(val));

                    uint8_t left = (idx >= header.channels) ? outRow[idx - header.channels] : 0;
                    uint8_t up = prevRow[idx];
                    uint8_t upLeft = (idx >= header.channels) ? prevRow[idx - header.channels] : 0;
                    uint8_t pred = getPredictor(predType, left, up, upLeft);
                    outRow[idx] = static_cast<uint8_t>(pred + diff);
                    idx++;
                } else {
                    return false;
                }
            }

            br.align(); 
            std::copy(outRow, outRow + stride, prevRow.begin());
            return true;
        }
    };
};

#endif
