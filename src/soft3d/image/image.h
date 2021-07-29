#pragma once
#include <soft3d/common/util.h>
#include <soft3d/math/linalg.h>

#include <vector>

struct Image {
    Image() = default;
    explicit Image(size_t width, size_t height)
        : width(width), height(height), buffer(width * height), packed(width * height) {
        std::fill(buffer.begin(), buffer.end(), Vector3(1.0, 1.0, 1.0));
    }

    inline Vector3 getPixel(int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return Vector3();
        return buffer[y * width + x];
    }

    inline Vector3 getPixelWrap(int x, int y) {
        x %= width;
        y %= height;
        return getPixel(x, y);
    }

    inline Vector3 getPixelClamp(int x, int y) {
        x = std::max(0, std::min(x, (int)width - 1));
        y = std::max(0, std::min(y, (int)height - 1));
        return getPixel(x, y);
    }

    void sigmoidToneMap(Float alpha) {
        Float lsum = 0.0;
        Float del = 0.00001; // prevent 0
        Float factor = 1.0f / (width * height);
        Image lumi(width, height);
        //ITU BT.601
        const Vector3 c(0.299, 0.587, 0.114);
        for (size_t j = 0; j < height; ++j) {
            for (size_t i = 0; i < width; ++i) {
                Float l= getPixel(i, j).dot(c);
                if (isnan(l)) {
                    printf("NAN\n");
                    lumi.setPixel(i, j, Vector3(0, 0, 0));
                } else {
                    lsum += factor*log(del + l);
                    lumi.setPixel(i, j, Vector3(l, 0, 0));
                }
            }
        }
        Float gavg = exp(lsum);
        for (size_t j = 0; j < height; ++j) {
            for (size_t i = 0; i < width; ++i) {
                Vector3 pixel = getPixel(i, j);
                Float fxy = alpha * lumi.getPixel(i, j).x() + (1 - alpha) * gavg;
                Float r = pixel.x() / (pixel.x() + fxy);
                Float g = pixel.y() / (pixel.y() + fxy);
                Float b = pixel.z() / (pixel.z() + fxy);
                setPixel(i, j, Vector3(r, g, b));
            }
        }
    }

    void flimToneMap() {
        const auto hable = [](Vector3 x) {
            float A = 0.15;
            float B = 0.50;
            float C = 0.10;
            float D = 0.20;
            float E = 0.02;
            float F = 0.30;

            return ((x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F)) - E / F;
        };
        Vector3 W(11.2f);
        float exposure_bias = 2.0f;
        for (size_t j = 0; j < height; ++j) {
            for (size_t i = 0; i < width; ++i) {
                Vector3 pixel = getPixel(i, j);
                Vector3 curr = hable(pixel * exposure_bias);
                Vector3 white_scale = Vector3(1.0f) / hable(W);
                
                setPixel(i, j, curr * white_scale);
            }
        }
    }

    inline void setPixel(int x, int y, const Vector3& rgb) {
        if (x < 0 || x >= width || y < 0 || y >= height)
            return;
        buffer[y * width + x] = rgb;
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
        return Vector3((pixel & 0xFF) / 255.0, ((pixel & 0xFF00) >> 8) / 255.0,
                       ((pixel & 0xFF0000) >> 16) / 255.0);
    }

    void pack() {
        for (int j = 0; j < height; ++j) {
            for (int i = 0; i < width; ++i) {
                packed[j * width + i] = packPixel(buffer[j * width + i]);
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
        std::fill(buffer.begin(), buffer.end(), Vector3(1.0, 1.0, 1.0));
    }

    int getWidth() const {
        return width;
    }

    int getHeight() const {
        return height;
    }

  private:
    size_t width{ 0 };
    size_t height{ 0 };
    std::vector<Vector3> buffer;   // rgba_uint8
    std::vector<uint32_t> packed;  // rgba_uint8
};
