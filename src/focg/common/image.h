#pragma once
#include <focg/common/linalg.h>
#include <vector>

struct Image {
    Image() = default;
    explicit Image(size_t width, size_t height) : width(width), height(height), buffer(width*height), packed(width*height){
        std::fill(buffer.begin(), buffer.end(), Vector3(1.0,1.0,1.0));
    }
    
    inline Vector3 getPixel(int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height) return Vector3();
        return buffer[y*width + x];
    }
    
    inline void setPixel(int x, int y, const Vector3& rgb) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        buffer[y*width + x] = rgb;
    }
    
    inline void setPixel(const Vector2& pos, const Vector3& rgb) {
        setPixel(pos.x(), pos.y(), rgb);
    }

    inline uint32_t packPixel(const Vector3& rgb) {
        const uint32_t r = static_cast<uint8_t>(0xFF * clampToNormal(rgb.x()));
        const uint32_t g = static_cast<uint8_t>(0xFF * clampToNormal(rgb.y()));
        const uint32_t b = static_cast<uint8_t>(0xFF * clampToNormal(rgb.z()));
        const uint32_t pixel = 0xFF000000 | r | (g << 8) | (b << 16);
        return pixel;
    }

    inline Vector3 unpackPixel(const uint32_t pixel) {
        return Vector3((pixel & 0xFF)/255.0, ((pixel & 0xFF00)>> 8)/255.0, ((pixel & 0xFF0000)>> 16)/255.0);
    }

    void pack() {
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                packed[j*width + i] = packPixel(buffer[j*width+i]);
            }
        }
    }

    uint32_t* data() {
        return packed.data();
    }
    
    const uint32_t* data() const {
        return packed.data();
    }
    
    void clear() {
        std::fill(buffer.begin(), buffer.end(), Vector3(1.0,1.0,1.0));
    }

    size_t getWidth() const {
        return width;
    }
    
    size_t getHeight() const {
        return height;
    }
    
private:
    size_t width{0};
    size_t height{0};
    std::vector<Vector3> buffer; // rgba_uint8
    std::vector<uint32_t> packed; // rgba_uint8
};
