#include <gtest/gtest.h>
#include <focg/common/linalg.h>

TEST(matrix, dword_aligned_to_two_gpr) {
    GMatrix<double> m(3,3);
    m[mi{0,0}] = 10.0;
    m[mi{1,0}] = 5.0;
    ASSERT_EQ(10.0, (m[mi{0,0}]));
    Vector3 a(1.0,0.0,0.0);
    Vector3 b = m.mul<Vector3>(a);
    ASSERT_EQ(b[0], 10.0);
    ASSERT_EQ(b[1], 5.0);
    ASSERT_EQ(b[2], 0.0);
}
