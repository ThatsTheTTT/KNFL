#ifndef KNLF_HPP
#define KNLF_HPP

// Recommended file extension: .knlf

#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <stdexcept>

#pragma pack(push, 1)
struct KNLFHeader {
    char magic[4];
    uint16_t width;
    uint16_t height;
    uint8_t channels;
    uint8_t formatVersion;
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
    explicit StreamBitWriter(std::ostream& os) : out(os) {}
    StreamBitWriter(const StreamBitWriter&) = delete;
    StreamBitWriter& operator=(const StreamBitWriter&) = delete;

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
    explicit StreamBitReader(std::istream& is) : in(is) {}
    StreamBitReader(const StreamBitReader&) = delete;
    StreamBitReader& operator=(const StreamBitReader&) = delete;

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
    static constexpr size_t MAX_DECODE_STRIDE = 64 * 1024 * 1024;
    static constexpr uint8_t FORMAT_VERSION = 1;
    static constexpr size_t HEADER_BODY_SIZE = 6;
    static constexpr uint32_t CRC32_POLY = 0xEDB88320u;
    static constexpr uint32_t CRC32_INIT = 0xFFFFFFFFu;
    static constexpr uint32_t CRC32_XOROUT = 0xFFFFFFFFu;

    static uint8_t paethPredictor(uint8_t a, uint8_t b, uint8_t c) {
        int p = static_cast<int>(a) + static_cast<int>(b) - static_cast<int>(c);
        int pa = std::abs(p - static_cast<int>(a));
        int pb = std::abs(p - static_cast<int>(b));
        int pc = std::abs(p - static_cast<int>(c));
        if (pa <= pb && pa <= pc) return a;
        if (pb <= pc) return b;
        return c;
    }

    static uint8_t getPredictor(PredictorType type, uint8_t left, uint8_t up, uint8_t upLeft) {
        switch (type) {
            case PRED_SUB:     return left;
            case PRED_UP:      return up;
            case PRED_AVERAGE: return static_cast<uint8_t>((static_cast<uint16_t>(left) + up + 1) / 2);
            case PRED_PAETH:   return paethPredictor(left, up, upLeft);
            default:           return 0;
        }
    }

    static int8_t computeDiff(uint8_t curr, uint8_t pred) {
        uint8_t u = static_cast<uint8_t>(curr - pred);
        int8_t result;
        std::memcpy(&result, &u, 1);
        return result;
    }

    static uint8_t encodeZigZag(int8_t val) {
        return (val >= 0) ? static_cast<uint8_t>(val << 1)
                          : static_cast<uint8_t>((-static_cast<int>(val) << 1) - 1);
    }

    static int8_t decodeZigZag(uint8_t val) {
        return (val & 1) ? static_cast<int8_t>(-(static_cast<int>(val >> 1) + 1))
                         : static_cast<int8_t>(val >> 1);
    }

    static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            crc ^= data[i];
            for (int j = 0; j < 8; ++j) {
                uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(crc & 1u));
                crc = (crc >> 1) ^ (CRC32_POLY & mask);
            }
        }
        return crc;
    }

public:
    class Encoder {
    private:
        std::ostream& out;
        StreamBitWriter bw;
        uint16_t width, height;
        uint8_t channels;
        std::vector<uint8_t> prevRow;
        uint32_t rowsWritten = 0;
        uint32_t crcState = CRC32_INIT;
        bool crcFooterWritten = false;

    public:
        Encoder(std::ostream& outputStream, uint16_t w, uint16_t h, uint8_t ch)
            : out(outputStream), bw(outputStream), width(w), height(h), channels(ch) {

            if (width == 0 || height == 0 || channels == 0) {
                throw std::invalid_argument("KNLFStream::Encoder: width, height and channels must be non-zero");
            }

            size_t stride = static_cast<size_t>(width) * static_cast<size_t>(channels);
            if (stride > MAX_DECODE_STRIDE) {
                throw std::invalid_argument("KNLFStream::Encoder: stride exceeds maximum allowed size");
            }

            out.write("KNLF", 4);
            uint8_t headerData[HEADER_BODY_SIZE];
            headerData[0] = static_cast<uint8_t>(width & 0xFF);
            headerData[1] = static_cast<uint8_t>(width >> 8);
            headerData[2] = static_cast<uint8_t>(height & 0xFF);
            headerData[3] = static_cast<uint8_t>(height >> 8);
            headerData[4] = channels;
            headerData[5] = FORMAT_VERSION;
            out.write(reinterpret_cast<char*>(headerData), HEADER_BODY_SIZE);

            if (!out.good()) {
                throw std::ios_base::failure("KNLFStream::Encoder: failed to write header");
            }

            prevRow.resize(stride, 0);
        }

        Encoder(const Encoder&) = delete;
        Encoder& operator=(const Encoder&) = delete;

        ~Encoder() {
            flush();
        }

        void flush() {
            bw.align();
            if (!crcFooterWritten && rowsWritten >= height) {
                uint32_t finalCrc = crcState ^ CRC32_XOROUT;
                uint8_t footer[4];
                footer[0] = static_cast<uint8_t>(finalCrc & 0xFF);
                footer[1] = static_cast<uint8_t>((finalCrc >> 8) & 0xFF);
                footer[2] = static_cast<uint8_t>((finalCrc >> 16) & 0xFF);
                footer[3] = static_cast<uint8_t>((finalCrc >> 24) & 0xFF);
                out.write(reinterpret_cast<char*>(footer), 4);
                crcFooterWritten = true;
            }
        }

        [[nodiscard]] bool encodeScanline(const uint8_t* currRow, size_t rowSize) {
            size_t stride = static_cast<size_t>(width) * static_cast<size_t>(channels);

            if (currRow == nullptr || rowSize != stride) return false;
            if (rowsWritten >= height) return false;

            crcState = crc32Update(crcState, currRow, stride);

            PredictorType bestPred = PRED_NONE;
            uint64_t minCost = 0xFFFFFFFFFFFFFFFFULL;
            std::vector<uint8_t> bestResidual(stride);
            std::vector<uint8_t> testResidual(stride);

            for (int p = 0; p <= 4; ++p) {
                PredictorType testPred = static_cast<PredictorType>(p);
                uint64_t cost = 0;
                for (size_t i = 0; i < stride; ++i) {
                    uint8_t left = (i >= channels) ? currRow[i - channels] : 0;
                    uint8_t up = prevRow[i];
                    uint8_t upLeft = (i >= channels) ? prevRow[i - channels] : 0;
                    uint8_t pred = getPredictor(testPred, left, up, upLeft);

                    int8_t diff = computeDiff(currRow[i], pred);
                    cost += std::abs(static_cast<int>(diff));
                    testResidual[i] = encodeZigZag(diff);
                }
                if (cost < minCost) {
                    minCost = cost;
                    bestPred = testPred;
                    bestResidual.swap(testResidual);
                }
            }

            const std::vector<uint8_t>& residualLine = bestResidual;

            bw.writeBits(static_cast<uint32_t>(bestPred), 3);

            size_t idx = 0;
            while (idx < stride) {
                if (residualLine[idx] == 0) {
                    size_t zeroRun = 0;
                    while (idx + zeroRun < stride && residualLine[idx + zeroRun] == 0 && zeroRun < 64) {
                        zeroRun++;
                    }
                    if (zeroRun >= 2) {
                        bw.writeBits(0, 2);
                        bw.writeBits(static_cast<uint32_t>(zeroRun - 1), 6);
                        idx += zeroRun;
                        continue;
                    }
                }

                if (residualLine[idx] <= 15) {
                    bw.writeBits(1, 2);
                    bw.writeBits(residualLine[idx], 4);
                } else {
                    bw.writeBits(2, 2);
                    bw.writeBits(residualLine[idx], 8);
                }
                idx++;
            }

            std::copy(currRow, currRow + stride, prevRow.begin());
            rowsWritten++;

            if (rowsWritten >= height) {
                flush();
            }

            return out.good();
        }
    };

    class Decoder {
    private:
        std::istream& in;
        StreamBitReader br;
        KNLFHeader header{};
        std::vector<uint8_t> prevRow;
        bool validHeader = false;
        uint32_t rowsRead = 0;
        uint32_t crcState = CRC32_INIT;
        bool crcChecked = false;
        bool crcOk = false;

    public:
        explicit Decoder(std::istream& inputStream) : in(inputStream), br(inputStream) {
            std::memset(&header, 0, sizeof(header));

            char magic[4];
            if (in.read(magic, 4) && std::memcmp(magic, "KNLF", 4) == 0) {
                uint8_t headerData[HEADER_BODY_SIZE];
                if (in.read(reinterpret_cast<char*>(headerData), HEADER_BODY_SIZE)) {
                    std::memcpy(header.magic, "KNLF", 4);
                    header.width = static_cast<uint16_t>(headerData[0] | (headerData[1] << 8));
                    header.height = static_cast<uint16_t>(headerData[2] | (headerData[3] << 8));
                    header.channels = headerData[4];
                    header.formatVersion = headerData[5];

                    size_t stride = static_cast<size_t>(header.width) * static_cast<size_t>(header.channels);

                    if (header.width > 0 && header.height > 0 && header.channels > 0 &&
                        header.formatVersion == FORMAT_VERSION &&
                        stride <= MAX_DECODE_STRIDE) {
                        validHeader = true;
                        prevRow.resize(stride, 0);
                    }
                }
            }
        }

        Decoder(const Decoder&) = delete;
        Decoder& operator=(const Decoder&) = delete;

        [[nodiscard]] bool isValid() const { return validHeader; }
        const KNLFHeader& getHeader() const { return header; }
        [[nodiscard]] bool isChecksumChecked() const { return crcChecked; }
        [[nodiscard]] bool isChecksumValid() const { return crcOk; }

        [[nodiscard]] bool decodeScanline(uint8_t* outRow, size_t rowSize) {
            if (!validHeader) return false;

            size_t stride = static_cast<size_t>(header.width) * static_cast<size_t>(header.channels);

            if (outRow == nullptr || rowSize != stride) return false;
            if (rowsRead >= header.height) return false;

            uint32_t predBits = 0;
            if (!br.readBits(predBits, 3)) return false;
            if (predBits > static_cast<uint32_t>(PRED_PAETH)) return false;
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
                        outRow[idx] = getPredictor(predType, left, up, upLeft);
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

            std::copy(outRow, outRow + stride, prevRow.begin());
            crcState = crc32Update(crcState, outRow, stride);
            rowsRead++;

            if (rowsRead >= header.height) {
                br.align();
                uint8_t footer[4];
                if (in.read(reinterpret_cast<char*>(footer), 4)) {
                    uint32_t storedCrc = static_cast<uint32_t>(footer[0]) |
                                         (static_cast<uint32_t>(footer[1]) << 8) |
                                         (static_cast<uint32_t>(footer[2]) << 16) |
                                         (static_cast<uint32_t>(footer[3]) << 24);
                    uint32_t computedCrc = crcState ^ CRC32_XOROUT;
                    crcOk = (storedCrc == computedCrc);
                } else {
                    crcOk = false;
                }
                crcChecked = true;
            }

            return true;
        }
    };
};

#endif
