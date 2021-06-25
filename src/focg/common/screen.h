#pragma once
#include <focg/common/linalg.h>
#include <vector>

struct Screen {
    Screen() = delete;
    explicit Screen(size_t width, size_t height) : width(width), height(height), buffer(width*height) {
        std::fill(buffer.begin(), buffer.end(), 0xFFFFFFFF);
    }
    
    inline void setPixel(size_t x, size_t y, const Vector3& rgb) {
        const uint32_t r = static_cast<uint8_t>(0xFF * clampToNormal(rgb.x()));
        const uint32_t g = static_cast<uint8_t>(0xFF * clampToNormal(rgb.y()));
        const uint32_t b = static_cast<uint8_t>(0xFF * clampToNormal(rgb.z()));
        const uint32_t pixel = 0xFF000000 | r | (g << 8) | (b << 16);
        buffer[y*width + x] = pixel;
    }
    
    inline void setPixel(const Vector2& pos, const Vector3& rgb) {
        setPixel(pos.x(), pos.y(), rgb);
    }
    
    uint32_t* data() {
        return buffer.data();
    }
    
    const uint32_t* data() const {
        return buffer.data();
    }
    
    size_t getWidth() const {
        return width;
    }
    
    size_t getHeight() const {
        return height;
    }
    
private:
    size_t width;
    size_t height;
    std::vector<uint32_t> buffer; // rgba_uint8
};
