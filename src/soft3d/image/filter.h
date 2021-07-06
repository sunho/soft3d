#pragma once
#include <soft3d/image/image.h>
#include <soft3d/math/linalg.h>

#include <vector>

struct Sequence1 {
    Sequence1() = default;
    explicit Sequence1(size_t size) : data(size), radius(size / 2) {
    }

    Float& operator[](size_t index) {
        return data[index];
    }

    const Float& operator[](size_t index) const {
        return data[index];
    }

    size_t getRadius() const {
        return radius;
    }

    size_t getSize() const {
        return data.size();
    }

  private:
    size_t radius{ 0 };
    std::vector<Float> data;
};

struct Sequence2Index {
    int i;
    int j;
};

using si2 = Sequence2Index;

struct Sequence2 {
    Sequence2() = default;
    explicit Sequence2(size_t width, size_t height)
        : width(width), height(height), data(width * height), radius(width / 2) {
    }

    Float& operator[](si2 index) {
        return data[index.j * width + index.i];
    }

    const Float& operator[](si2 index) const {
        return data[index.j * width + index.i];
    }

    size_t getHeight() const {
        return height;
    }

    size_t getWidth() const {
        return width;
    }

    size_t getRadius() const {
        return radius;
    }

  private:
    size_t radius{ 0 };
    size_t width;
    size_t height;
    std::vector<Float> data;
};

struct ContFilter1 {
    ContFilter1() = default;
    ContFilter1(size_t radius, std::function<Float(Float x)> func) : radius(radius), func(func) {
    }

    Float operator()(Float x) const {
        return func(x);
    }

    size_t getRadius() const {
        return radius;
    }

  private:
    size_t radius{ 0 };
    std::function<Float(Float x)> func{ nullptr };
};

struct ContFilter2 {
    ContFilter2() = default;
    ContFilter2(size_t radius, std::function<Float(Float x, Float y)> func)
        : radius(radius), func(func) {
    }

    Float operator()(Float x, Float y) const {
        return func(x, y);
    }

    size_t getRadius() const {
        return radius;
    }

  private:
    size_t radius{ 0 };
    std::function<Float(Float x, Float y)> func{ nullptr };
};

static Float reconstruct(Sequence1 seq, ContFilter1 filter, Float x) {
    Float s = 0;
    const int start = ceil(x - filter.getRadius());
    const int end = floor(x + filter.getRadius());
    for (int i = start; i <= end; ++i) {
        if (i < 0 || i >= seq.getSize())
            continue;
        s += seq[i] * filter(x - i);
    }
    return s;
}

static Float reconstruct(Sequence2 seq, ContFilter2 filter, Float x, Float y) {
    Float s = 0;
    const int xstart = ceil(x - filter.getRadius());
    const int xend = floor(x + filter.getRadius());
    const int ystart = ceil(y - filter.getRadius());
    const int yend = floor(y + filter.getRadius());
    for (int i = xstart; i <= xend; ++i) {
        for (int j = ystart; j <= yend; ++j) {
            if (i < 0 || i >= seq.getWidth())
                continue;
            if (j < 0 || j >= seq.getHeight())
                continue;
            s += seq[si2{ i, j }] * filter(x - i, y - j);
        }
    }
    return s;
}

static Float convolve(Sequence1 seq, Sequence1 filter, size_t x) {
    Float s = 0;
    const int start = x - filter.getRadius();
    const int end = x + filter.getRadius();
    for (int i = start; i <= end; ++i) {
        if (i < 0 || i >= seq.getSize())
            continue;
        s += seq[i] * filter[x - i + filter.getRadius()];  // TODO: what's more ideal way?
    }
    return s;
}

static Float convolve(Sequence2 seq, Sequence2 filter, size_t x, size_t y) {
    Float s = 0;
    const int xstart = x - filter.getRadius();
    const int xend = x + filter.getRadius();
    const int ystart = y - filter.getRadius();
    const int yend = y + filter.getRadius();
    for (int i = xstart; i <= xend; ++i) {
        for (int j = ystart; j <= yend; ++j) {
            if (i < 0 || i >= seq.getWidth())
                continue;
            if (j < 0 || j >= seq.getHeight())
                continue;
            int fx = x - i + filter.getRadius();
            int fy = y - j + filter.getRadius();
            s += seq[si2{ i, j }] * filter[si2{ fx, fy }];
        }
    }
    return s;
}

// Box filter
// "uniform" filter
static Sequence1 boxFilter1(size_t radius) {
    Sequence1 out(radius * 2 + 1);
    for (int i = 0; i < out.getSize(); ++i) {
        out[i] = 1.0 / out.getSize();
    }
    return out;
}

// it's equivalent to box * box * box * box
// box * box = tent
// natural radius: 2.0
static Float bspline1(Float x) {
    if (-1.0 <= x && x <= 1.0) {
        auto out =
            (-3 * pow(1.0 - fabs(x), 3.0) + 3 * pow(1.0 - fabs(x), 2.0) + 3 * (1.0 - fabs(x)) + 1) /
            6.0;
        return out;
    } else {
        return (pow((2 - fabs(x)), 3.0)) / 6.0;
    }
}

static ContFilter1 bsplineFilter1() {
    return ContFilter1(2.0, bspline1);
}

// Catmull-Rom filter
// it iterpolates the samples
// natural radius: 2.0
static Float catmullRom1(Float x) {
    if (-1.0 <= x && x <= 1.0) {
        auto out =
            (-3 * pow(1.0 - fabs(x), 3.0) + 4 * pow(1.0 - fabs(x), 2.0) + (1.0 - fabs(x))) / 2.0;
        return out;
    } else {
        return (pow(2 - fabs(x), 3.0) - pow(2 - fabs(x), 2.0)) / 2.0;
    }
}

static ContFilter1 catmullRomFilter1() {
    return ContFilter1(2.0, catmullRom1);
}

// Mitchell-Netravali filter
// it's weighted average of bspline and catmull filter
// natural radius: 2.0
static Float mitchell1(Float x) {
    if (-1.0 <= x && x <= 1.0) {
        auto out = (-21 * pow(1.0 - fabs(x), 3.0) + 27 * pow(1.0 - fabs(x), 2.0) +
                    9 * (1.0 - fabs(x)) + 1) /
                   18.0;
        return out;
    } else {
        return (7 * pow(2 - fabs(x), 3.0) - 6 * pow(2 - fabs(x), 2.0)) / 18.0;
    }
}

static ContFilter1 mitchellFilter1() {
    return ContFilter1(2.0, mitchell1);
}

// it's equivalent to box * box
// natural radius: 1.0
static Float tent1(Float x) {
    return 1.0 - fabs(x);
}

static ContFilter1 tentFilter1() {
    return ContFilter1(1.0, tent1);
}

// Gaussian filter
// it's just normal distribution pdf
// natural radius: infinity (needs to be specfied with sane value)
static ContFilter1 normalFilter1(Float std, Float r) {
    const Float c = 1.0 / (sqrt(2 * PI) * std);
    const Float d = 1.0 / 2 * std * std;
    return ContFilter1(r, [=](Float x) { return c * exp(-x * x * d); });
}

// Box filter
// "uniform" filter
static Sequence2 boxFilter2(size_t radius) {
    Sequence2 out(radius * 2 + 1, radius * 2 + 1);
    for (int i = 0; i < out.getWidth(); ++i) {
        for (int j = 0; j < out.getHeight(); ++j) {
            out[si2{ i, j }] = 1.0 / (out.getWidth() * out.getHeight());
        }
    }
    return out;
}

static Float tent2(Float x, Float y) {
    return (1.0 - fabs(x)) * (1.0 - fabs(y));
}

static ContFilter2 tentFilter2() {
    return ContFilter2(1.0, tent2);
}

// Gaussian filter
// it's "scaled" version
static ContFilter2 normalFilter2(Float std, Float r) {
    const Float c = 1.0 / sqrt(2 * PI);
    return ContFilter2(r, [=](Float x, Float y) { return c * exp(-(x * x + y * y) / 2); });
}

static Image filterImage(Image image, Sequence1 filter) {
    Image out(image.getWidth(), image.getHeight());
    std::vector<Vector3> cache(image.getHeight());
    for (int x = filter.getRadius(); x < image.getWidth() - filter.getRadius(); ++x) {
        for (int y = 0; y < image.getHeight(); ++y) {
            cache[y] = Vector3();
            for (int i = x - filter.getRadius(); i <= x + filter.getRadius(); ++i) {
                if (i < 0 || i >= image.getWidth())
                    continue;
                cache[y] += image.getPixel(i, y) * filter[x - i + filter.getRadius()];
            }
        }
        for (int y = filter.getRadius(); y < image.getHeight() - filter.getRadius(); ++y) {
            Vector3 pixel;
            for (int j = y - filter.getRadius(); j <= y + filter.getRadius(); ++j) {
                if (j < 0 || j >= image.getHeight())
                    continue;
                pixel += cache[j] * filter[y - j + filter.getRadius()];
            }
            out.setPixel(x, y, pixel);
        }
    }
    return out;
}

static Sequence1 discretify(ContFilter1 filter) {
    Sequence1 out(filter.getRadius() * 2 + 1);
    for (int i = 0; i < out.getSize(); ++i) {
        out[i] = filter(i - static_cast<int>(filter.getRadius()));
    }
    return out;
}

static ContFilter1 scaleFilter(ContFilter1 filter, int n) {
    return ContFilter1(filter.getRadius() * n, [=](Float x) { return filter(x / n) / n; });
}
