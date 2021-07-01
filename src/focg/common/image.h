#pragma once
#include <focg/common/linalg.h>
#include <vector>

struct Image {
    Image() = default;
    explicit Image(size_t width, size_t height) : width(width), height(height), buffer(width*height) {
        std::fill(buffer.begin(), buffer.end(), 0xFFFFFFFF);
    }
    
    inline void setPixel(int x, int y, const Vector3& rgb) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        const uint32_t r = static_cast<uint8_t>(0xFF * clampToNormal(rgb.x()));
        const uint32_t g = static_cast<uint8_t>(0xFF * clampToNormal(rgb.y()));
        const uint32_t b = static_cast<uint8_t>(0xFF * clampToNormal(rgb.z()));
        const uint32_t pixel = 0xFF000000 | r | (g << 8) | (b << 16);
        buffer[y*width + x] = pixel;
    }
    
    inline Vector3 readPixel(int x, int y) {
        const uint32_t pixel = buffer[y*width + x];
        return Vector3((pixel & 0xFF)/255.0, ((pixel & 0xFF00)>> 8)/255.0, ((pixel & 0xFF0000)>> 16)/255.0);
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
    
    Image antialias() {
        Image out(width, height);
        for (int i = 1; i < width-1; ++i) {
            for (int j = 1; j < height-1; ++j) {
                Vector3 color;
                color += readPixel(i-1,j-1)/9.0;
                color += readPixel(i,j-1)/9.0;
                color += readPixel(i+1,j-1)/9.0;
                color += readPixel(i-1,j)/9.0;
                color += readPixel(i,j)/9.0;
                color += readPixel(i+1,j)/9.0;
                color += readPixel(i-1,j+1)/9.0;
                color += readPixel(i,j+1)/9.0;
                color += readPixel(i+1,j+1)/9.0;
                out.setPixel(i,j,color);
            }
        }
        return out;
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
    std::vector<uint32_t> buffer; // rgba_uint8
};
