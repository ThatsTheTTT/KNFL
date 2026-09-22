#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <fstream>
#include <chrono>

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

inline uint8_t paethPredictor(uint8_t a, uint8_t b, uint8_t c) {
    int p = (int)a + (int)b - (int)c;
    int pa = std::abs(p - (int)a);
    int pb = std::abs(p - (int)b);
    int pc = std::abs(p - (int)c);

    if (pa <= pb && pa <= pc) return a;
    if (pb <= pc) return b;
    return c;
}

inline uint8_t encodeZigZag(int8_t val) {
    return static_cast<uint8_t>((val << 1) ^ (val >> 7));
}

inline int8_t decodeZigZag(uint8_t val) {
    return static_cast<int8_t>((val >> 1) ^ (-(val & 1)));
}

class BitWriter {
public:
    std::vector<uint8_t> buffer;
    uint8_t currentByte = 0;
    uint8_t bitPos = 0;

    void writeBits(uint32_t val, uint8_t numBits) {
        for (int i = numBits - 1; i >= 0; --i) {
            uint8_t bit = (val >> i) & 1;
            currentByte |= (bit << (7 - bitPos));
            bitPos++;
            if (bitPos == 8) {
                buffer.push_back(currentByte);
                currentByte = 0;
                bitPos = 0;
            }
        }
    }

    void flush() {
        if (bitPos > 0) {
            buffer.push_back(currentByte);
            currentByte = 0;
            bitPos = 0;
        }
    }
};

class BitReader {
private:
    const uint8_t* data;
    size_t size;
    size_t bytePos = 0;
    uint8_t bitPos = 0;

public:
    BitReader(const uint8_t* src, size_t srcSize) : data(src), size(srcSize) {}

    bool readBit(uint8_t& outBit) {
        if (bytePos >= size) return false;
        outBit = (data[bytePos] >> (7 - bitPos)) & 1;
        bitPos++;
        if (bitPos == 8) {
            bitPos = 0;
            bytePos++;
        }
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

    size_t getBytesRead() const { return bytePos + (bitPos > 0 ? 1 : 0); }
};

class KNLFEncoder {
private:
    static uint8_t getPredictorValue(PredictorType type, uint8_t left, uint8_t up, uint8_t upLeft) {
        switch (type) {
            case PRED_SUB:     return left;
            case PRED_UP:      return up;
            case PRED_AVERAGE: return (static_cast<uint16_t>(left) + up) / 2;
            case PRED_PAETH:   return paethPredictor(left, up, upLeft);
            default:           return 0;
        }
    }

    static uint64_t evaluateLineCost(PredictorType type, const uint8_t* currRow, 
                                     const uint8_t* prevRow, uint16_t width, uint8_t channels) {
        uint64_t cost = 0;
        size_t rowSize = width * channels;

        for (size_t i = 0; i < rowSize; ++i) {
            uint8_t left = (i >= channels) ? currRow[i - channels] : 0;
            uint8_t up = prevRow ? prevRow[i] : 0;
            uint8_t upLeft = (prevRow && i >= channels) ? prevRow[i - channels] : 0;

            uint8_t pred = getPredictorValue(type, left, up, upLeft);
            int8_t diff = static_cast<int8_t>(currRow[i] - pred);
            cost += std::abs(diff);
        }
        return cost;
    }

public:
    static bool compress(const uint8_t* rawPixels, uint16_t width, uint16_t height, 
                         uint8_t channels, std::vector<uint8_t>& outBuffer) {
        if (!rawPixels || width == 0 || height == 0 || (channels != 3 && channels != 4)) {
            return false;
        }

        KNLFHeader header;
        header.magic[0] = 'K'; header.magic[1] = 'N'; 
        header.magic[2] = 'L'; header.magic[3] = 'F';
        header.width = width;
        header.height = height;
        header.channels = channels;
        header.flags = 0;

        outBuffer.clear();
        uint8_t* headerPtr = reinterpret_cast<uint8_t*>(&header);
        outBuffer.insert(outBuffer.end(), headerPtr, headerPtr + sizeof(KNLFHeader));

        BitWriter bw;
        size_t stride = width * channels;
        std::vector<uint8_t> residualLine(stride);

        for (uint16_t y = 0; y < height; ++y) {
            const uint8_t* currRow = rawPixels + (y * stride);
            const uint8_t* prevRow = (y > 0) ? (rawPixels + ((y - 1) * stride)) : nullptr;

            PredictorType bestPred = PRED_NONE;
            uint64_t minCost = evaluateLineCost(PRED_NONE, currRow, prevRow, width, channels);

            for (int p = 1; p <= 4; ++p) {
                PredictorType testPred = static_cast<PredictorType>(p);
                uint64_t cost = evaluateLineCost(testPred, currRow, prevRow, width, channels);
                if (cost < minCost) {
                    minCost = cost;
                    bestPred = testPred;
                }
            }

            bw.writeBits(static_cast<uint32_t>(bestPred), 3);

            for (size_t i = 0; i < stride; ++i) {
                uint8_t left = (i >= channels) ? currRow[i - channels] : 0;
                uint8_t up = prevRow ? prevRow[i] : 0;
                uint8_t upLeft = (prevRow && i >= channels) ? prevRow[i - channels] : 0;

                uint8_t pred = getPredictorValue(bestPred, left, up, upLeft);
                int8_t diff = static_cast<int8_t>(currRow[i] - pred);
                residualLine[i] = encodeZigZag(diff);
            }

            size_t idx = 0;
            while (idx < stride) {
                if (residualLine[idx] == 0) {
                    size_t zeroRun = 0;
                    while (idx + zeroRun < stride && residualLine[idx + zeroRun] == 0 && zeroRun < 63) {
                        zeroRun++;
                    }
                    bw.writeBits(0, 2); 
                    bw.writeBits(static_cast<uint32_t>(zeroRun), 6);
                    idx += zeroRun;
                } else {
                    uint8_t val = residualLine[idx];
                    if (val <= 15) {
                        bw.writeBits(1, 2);
                        bw.writeBits(val, 4);
                    } else {
                        bw.writeBits(2, 2);
                        bw.writeBits(val, 8);
                    }
                    idx++;
                }
            }
        }

        bw.flush();
        outBuffer.insert(outBuffer.end(), bw.buffer.begin(), bw.buffer.end());
        return true;
    }
};

class KNLFDecoder {
private:
    static uint8_t getPredictorValue(PredictorType type, uint8_t left, uint8_t up, uint8_t upLeft) {
        switch (type) {
            case PRED_SUB:     return left;
            case PRED_UP:      return up;
            case PRED_AVERAGE: return (static_cast<uint16_t>(left) + up) / 2;
            case PRED_PAETH:   return paethPredictor(left, up, upLeft);
            default:           return 0;
        }
    }

public:
    static bool decompress(const uint8_t* compressedData, size_t compressedSize,
                           std::vector<uint8_t>& outPixels, uint16_t& outWidth, 
                           uint16_t& outHeight, uint8_t& outChannels) {
        if (!compressedData || compressedSize < sizeof(KNLFHeader)) {
            return false;
        }

        const KNLFHeader* header = reinterpret_cast<const KNLFHeader*>(compressedData);
        if (header->magic[0] != 'K' || header->magic[1] != 'N' ||
            header->magic[2] != 'L' || header->magic[3] != 'F') {
            return false;
        }

        outWidth = header->width;
        outHeight = header->height;
        outChannels = header->channels;

        size_t stride = outWidth * outChannels;
        outPixels.resize(stride * outHeight);

        BitReader br(compressedData + sizeof(KNLFHeader), compressedSize - sizeof(KNLFHeader));

        for (uint16_t y = 0; y < outHeight; ++y) {
            uint32_t predBits = 0;
            if (!br.readBits(predBits, 3)) return false;
            PredictorType predType = static_cast<PredictorType>(predBits);

            uint8_t* currRow = outPixels.data() + (y * stride);
            const uint8_t* prevRow = (y > 0) ? (outPixels.data() + ((y - 1) * stride)) : nullptr;

            size_t idx = 0;
            while (idx < stride) {
                uint32_t mode = 0;
                if (!br.readBits(mode, 2)) return false;

                if (mode == 0) {
                    uint32_t runLen = 0;
                    if (!br.readBits(runLen, 6)) return false;
                    for (size_t r = 0; r < runLen && idx < stride; ++r) {
                        uint8_t left = (idx >= outChannels) ? currRow[idx - outChannels] : 0;
                        uint8_t up = prevRow ? prevRow[idx] : 0;
                        uint8_t upLeft = (prevRow && idx >= outChannels) ? prevRow[idx - outChannels] : 0;

                        uint8_t pred = getPredictorValue(predType, left, up, upLeft);
                        currRow[idx] = pred;
                        idx++;
                    }
                } else if (mode == 1) {
                    uint32_t val = 0;
                    if (!br.readBits(val, 4)) return false;
                    int8_t diff = decodeZigZag(static_cast<uint8_t>(val));

                    uint8_t left = (idx >= outChannels) ? currRow[idx - outChannels] : 0;
                    uint8_t up = prevRow ? prevRow[idx] : 0;
                    uint8_t upLeft = (prevRow && idx >= outChannels) ? prevRow[idx - outChannels] : 0;

                    uint8_t pred = getPredictorValue(predType, left, up, upLeft);
                    currRow[idx] = static_cast<uint8_t>(pred + diff);
                    idx++;
                } else if (mode == 2) {
                    uint32_t val = 0;
                    if (!br.readBits(val, 8)) return false;
                    int8_t diff = decodeZigZag(static_cast<uint8_t>(val));

                    uint8_t left = (idx >= outChannels) ? currRow[idx - outChannels] : 0;
                    uint8_t up = prevRow ? prevRow[idx] : 0;
                    uint8_t upLeft = (prevRow && idx >= outChannels) ? prevRow[idx - outChannels] : 0;

                    uint8_t pred = getPredictorValue(predType, left, up, upLeft);
                    currRow[idx] = static_cast<uint8_t>(pred + diff);
                    idx++;
                }
            }
        }
        return true;
    }
};

std::vector<uint8_t> generateTestPattern(uint16_t w, uint16_t h, uint8_t channels) {
    std::vector<uint8_t> img(w * h * channels);
    for (uint16_t y = 0; y < h; ++y) {
        for (uint16_t x = 0; x < w; ++x) {
            size_t idx = (y * w + x) * channels;
            img[idx + 0] = static_cast<uint8_t>((x * 255) / w);
            img[idx + 1] = static_cast<uint8_t>((y * 255) / h);
            img[idx + 2] = static_cast<uint8_t>(((x + y) * 128) / (w + h));
            if (channels == 4) {
                img[idx + 3] = 255;
            }
        }
    }
    return img;
}

int main() {
    const uint16_t WIDTH = 512;
    const uint16_t HEIGHT = 512;
    const uint8_t CHANNELS = 3;

    std::vector<uint8_t> originalPixels = generateTestPattern(WIDTH, HEIGHT, CHANNELS);

    std::vector<uint8_t> compressedKNLF;
    bool encSuccess = KNLFEncoder::compress(originalPixels.data(), WIDTH, HEIGHT, CHANNELS, compressedKNLF);

    if (!encSuccess) {
        return 1;
    }

    const char* filename = "output.knlf";
    {
        std::ofstream outFile(filename, std::ios::binary);
        outFile.write(reinterpret_cast<const char*>(compressedKNLF.data()), compressedKNLF.size());
    }

    std::vector<uint8_t> loadedKNLF;
    {
        std::ifstream inFile(filename, std::ios::binary | std::ios::ate);
        std::streamsize fsize = inFile.tellg();
        inFile.seekg(0, std::ios::beg);
        loadedKNLF.resize(fsize);
        inFile.read(reinterpret_cast<char*>(loadedKNLF.data()), fsize);
    }

    std::vector<uint8_t> decompressedPixels;
    uint16_t decW = 0, decH = 0;
    uint8_t decChannels = 0;

    bool decSuccess = KNLFDecoder::decompress(loadedKNLF.data(), loadedKNLF.size(), 
                                               decompressedPixels, decW, decH, decChannels);

    if (!decSuccess) {
        return 1;
    }

    bool isExactMatch = (originalPixels == decompressedPixels);

    if (isExactMatch) {
        return 0;
    } else {
        return 2;
    }
}

