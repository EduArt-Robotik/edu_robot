/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */

#include "edu_robot/angle.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <array>
#include <tuple>

using namespace eduart::robot;

// Helper function for floating point comparison
constexpr double EPSILON = 1e-9;

// ============================================================================
// Struktur für Testtabellen-Einträge
// ============================================================================

struct AngleTestCase {
  double input;
  double expected;
  std::string description;
};

// ============================================================================
// Tests for base Angle class
// ============================================================================

TEST(AngleTest, DefaultConstructor) {
  Angle angle;
  EXPECT_NEAR(angle.radian(), 0.0, EPSILON);
  EXPECT_NEAR(angle.degree(), 0.0, EPSILON);
}

TEST(AngleTest, ConversionTable) {
  const std::array<AngleTestCase, 8> testCases = {{
    {0.0,           0.0,           "Zero"},
    {M_PI,          180.0,         "π radians = 180 degrees"},
    {M_PI / 2.0,    90.0,          "π/2 radians = 90 degrees"},
    {M_PI / 4.0,    45.0,          "π/4 radians = 45 degrees"},
    {2.0 * M_PI,    360.0,         "2π radians = 360 degrees"},
    {M_PI / 6.0,    30.0,          "π/6 radians = 30 degrees"},
    {M_PI / 3.0,    60.0,          "π/3 radians = 60 degrees"},
    {3.0 * M_PI / 4.0, 135.0,      "3π/4 radians = 135 degrees"},
  }};

  for (const auto& testCase : testCases) {
    Angle angle(testCase.input);
    EXPECT_NEAR(angle.degree(), testCase.expected, EPSILON)
        << "Failed for: " << testCase.description;
  }
}

TEST(AngleTest, OperatorsTable) {
  const std::array<std::tuple<double, double, double, std::string>, 6> testCases = {{
    // {operand1, operand2, expected_result, description}
    {M_PI / 4.0, M_PI / 4.0, M_PI / 2.0, "Addition: π/4 + π/4 = π/2"},
    {M_PI,       M_PI / 2.0, M_PI / 2.0, "Subtraction: π - π/2 = π/2"},
    {M_PI / 4.0, 2.0,        M_PI / 2.0, "Multiplication: π/4 * 2 = π/2"},
    {M_PI,       2.0,        M_PI / 2.0, "Division: π / 2 = π/2"},
    {M_PI / 2.0, M_PI / 4.0, 3.0 * M_PI / 4.0, "Addition: π/2 + π/4 = 3π/4"},
    {0.0,        M_PI,       M_PI,       "Addition with zero: 0 + π = π"},
  }};

  for (const auto& testCase : testCases) {
    double op1 = std::get<0>(testCase);
    double op2 = std::get<1>(testCase);
    double expected = std::get<2>(testCase);
    const auto& desc = std::get<3>(testCase);

    if (desc.find("Addition:") != std::string::npos) {
      Angle a(op1);
      Angle result = a + op2;
      EXPECT_NEAR(result.radian(), expected, EPSILON) << desc;
    } else if (desc.find("Subtraction:") != std::string::npos) {
      Angle a(op1);
      Angle result = a - op2;
      EXPECT_NEAR(result.radian(), expected, EPSILON) << desc;
    } else if (desc.find("Multiplication:") != std::string::npos) {
      Angle a(op1);
      Angle result = a * op2;
      EXPECT_NEAR(result.radian(), expected, EPSILON) << desc;
    } else if (desc.find("Division:") != std::string::npos) {
      Angle a(op1);
      Angle result = a / op2;
      EXPECT_NEAR(result.radian(), expected, EPSILON) << desc;
    }
  }
}

// ============================================================================
// Tests for AnglePiToPi (normalized to ]-π, π])
// ============================================================================

TEST(AnglePiToPiTest, NormalizationTable) {
  const std::array<AngleTestCase, 13> testCases = {{
    {0.0,          0.0,          "Zero stays zero"},
    {M_PI / 2.0,   M_PI / 2.0,   "π/2 in range"},
    {-M_PI / 2.0,  -M_PI / 2.0,  "-π/2 in range"},
    {M_PI,         M_PI,         "π at upper boundary (inclusive)"},
    {-M_PI,        M_PI,         "-π normalized to π (boundary case)"},
    {3.0 * M_PI,   M_PI,         "3π normalized to π"},
    {-3.0 * M_PI,  M_PI,         "-3π normalized to π"},
    {5.0 * M_PI,   M_PI,         "5π normalized to π (via remainder)"},
    {3.0 * M_PI / 2.0, -M_PI / 2.0, "3π/2 normalized to -π/2"},
    {-3.0 * M_PI / 2.0, M_PI / 2.0, "-3π/2 normalized to π/2"},
    {100.0 * M_PI, 0.0,          "100π normalized to 0 (even multiple)"},
    {-100.0 * M_PI, 0.0,         "-100π normalized to 0 (even multiple)"},
    {7.0 * M_PI / 4.0, -M_PI / 4.0, "7π/4 normalized to -π/4"},
  }};

  for (const auto& testCase : testCases) {
    AnglePiToPi angle(testCase.input);
    EXPECT_NEAR(angle.radian(), testCase.expected, EPSILON)
        << "Failed for: " << testCase.description
        << " (input: " << testCase.input << ")";
  }
}

TEST(AnglePiToPiTest, OperationsTable) {
  const std::array<std::tuple<double, double, double, std::string>, 6> testCases = {{
    {M_PI / 2.0,    M_PI,         -M_PI / 2.0, "π/2 + π = 3π/2 → -π/2"},
    {-M_PI / 2.0,   -M_PI,        M_PI / 2.0,  "-π/2 - π = -3π/2 → π/2"},
    {M_PI / 4.0,    M_PI / 4.0,   M_PI / 2.0,  "π/4 + π/4 = π/2"},
    {M_PI / 3.0,    M_PI / 6.0,   M_PI / 2.0,  "π/3 + π/6 = π/2"},
    {M_PI / 2.0,    M_PI / 4.0,   M_PI / 4.0,  "π/2 - π/4 = π/4"},
    {0.0,           0.0,          0.0,         "0 + 0 = 0"},
  }};

  for (const auto& testCase : testCases) {
    double initial = std::get<0>(testCase);
    double offset = std::get<1>(testCase);
    double expected = std::get<2>(testCase);
    const auto& desc = std::get<3>(testCase);

    if (desc.find("+") != std::string::npos) {
      AnglePiToPi angle(initial);
      angle += offset;
      EXPECT_NEAR(angle.radian(), expected, EPSILON) << desc;
    } else if (desc.find("-") != std::string::npos) {
      AnglePiToPi angle(initial);
      angle -= offset;
      EXPECT_NEAR(angle.radian(), expected, EPSILON) << desc;
    }
  }
}

// ============================================================================
// Tests for Angle0To2Pi (normalized to [0, 2π[)
// ============================================================================

TEST(Angle0To2PiTest, NormalizationTable) {
  const std::array<AngleTestCase, 12> testCases = {{
    {0.0,          0.0,            "Zero stays zero"},
    {M_PI / 4.0,   M_PI / 4.0,     "π/4 in range"},
    {M_PI,         M_PI,           "π in range"},
    {2.0 * M_PI,   0.0,            "2π normalized to 0 (upper boundary)"},
    {4.0 * M_PI,   0.0,            "4π normalized to 0"},
    {3.0 * M_PI,   M_PI,           "3π normalized to π"},
    {-M_PI / 2.0,  3.0 * M_PI / 2.0, "-π/2 normalized to 3π/2"},
    {-M_PI,        M_PI,           "-π normalized to π"},
    {3.0 * M_PI / 2.0, 3.0 * M_PI / 2.0, "3π/2 in range"},
    {100.5 * M_PI, M_PI / 2.0,     "100.5π normalized to π/2 (via remainder)"},
    {-100.0 * M_PI, 0.0,           "-100π normalized to 0 or 2π (fmod boundary)"},
    {5.0 * M_PI / 2.0, M_PI / 2.0, "5π/2 normalized to π/2"},
  }};

  for (const auto& testCase : testCases) {
    Angle0To2Pi angle(testCase.input);
    if (testCase.description.find("fmod boundary") != std::string::npos) {
      // Special case: fmod can result in 2π or 0
      double result = angle.radian();
      EXPECT_TRUE(std::abs(result - 0.0) < EPSILON || std::abs(result - 2.0 * M_PI) < EPSILON)
          << "Failed for: " << testCase.description;
    } else {
      EXPECT_NEAR(angle.radian(), testCase.expected, EPSILON)
          << "Failed for: " << testCase.description
          << " (input: " << testCase.input << ")";
    }
  }
}

TEST(Angle0To2PiTest, OperationsTable) {
  const std::array<std::tuple<double, double, double, std::string>, 5> testCases = {{
    {3.0 * M_PI / 2.0, M_PI,         M_PI / 2.0,         "3π/2 + π = 5π/2 → π/2"},
    {M_PI / 2.0,       M_PI,         3.0 * M_PI / 2.0,   "π/2 - π = -π/2 → 3π/2"},
    {M_PI / 4.0,       M_PI / 4.0,   M_PI / 2.0,         "π/4 + π/4 = π/2"},
    {M_PI,             M_PI / 2.0,   M_PI / 2.0,         "π - π/2 = π/2"},
    {0.0,              0.0,          0.0,                "0 + 0 = 0"},
  }};

  for (const auto& testCase : testCases) {
    double initial = std::get<0>(testCase);
    double offset = std::get<1>(testCase);
    double expected = std::get<2>(testCase);
    const auto& desc = std::get<3>(testCase);

    if (desc.find("+") != std::string::npos) {
      Angle0To2Pi angle(initial);
      angle += offset;
      EXPECT_NEAR(angle.radian(), expected, EPSILON) << desc;
    } else if (desc.find("-") != std::string::npos) {
      Angle0To2Pi angle(initial);
      angle -= offset;
      EXPECT_NEAR(angle.radian(), expected, EPSILON) << desc;
    }
  }
}

// ============================================================================
// Tests for AnglePi2ToPi2 (normalized to [-π/2, π/2])
// ============================================================================

TEST(AnglePi2ToPi2Test, NormalizationTable) {
  const std::array<AngleTestCase, 12> testCases = {{
    {0.0,              0.0,           "Zero stays zero"},
    {M_PI / 4.0,       M_PI / 4.0,    "π/4 in range"},
    {-M_PI / 4.0,      -M_PI / 4.0,   "-π/4 in range"},
    {M_PI / 2.0,       M_PI / 2.0,    "π/2 at upper boundary"},
    {-M_PI / 2.0,      M_PI / 2.0,    "-π/2 normalized to π/2 (boundary)"},
    {3.0 * M_PI / 4.0, -M_PI / 4.0,   "3π/4 normalized to -π/4"},
    {-3.0 * M_PI / 4.0, M_PI / 4.0,   "-3π/4 normalized to π/4"},
    {5.0 * M_PI / 4.0, M_PI / 4.0,    "5π/4 normalized to π/4"},
    {-3.0 * M_PI / 4.0, M_PI / 4.0,   "-3π/4 normalized to π/4"},
    {50.5 * M_PI,      -M_PI / 2.0,   "50.5π → ±π/2 (via remainder)"},
    {-50.5 * M_PI,     -M_PI / 2.0,   "-50.5π → ±π/2 (via remainder)"},
    {7.0 * M_PI / 4.0, -M_PI / 4.0,   "7π/4 normalized to -π/4"},
  }};

  for (const auto& testCase : testCases) {
    AnglePi2ToPi2 angle(testCase.input);
    if (testCase.description.find("±π/2") != std::string::npos) {
      // Special case: can be π/2 or -π/2 due to remainder behavior
      double result = angle.radian();
      EXPECT_TRUE(std::abs(result - M_PI / 2.0) < EPSILON || std::abs(result + M_PI / 2.0) < EPSILON)
          << "Failed for: " << testCase.description;
    } else {
      EXPECT_NEAR(angle.radian(), testCase.expected, EPSILON)
          << "Failed for: " << testCase.description
          << " (input: " << testCase.input << ")";
    }
  }
}

TEST(AnglePi2ToPi2Test, OperationsTable) {
  const std::array<std::tuple<double, double, double, std::string>, 4> testCases = {{
    {M_PI / 4.0,   M_PI / 2.0, -M_PI / 4.0, "π/4 + π/2 = 3π/4 → -π/4"},
    {-M_PI / 4.0,  -M_PI / 2.0, M_PI / 4.0, "-π/4 - π/2 = -3π/4 → π/4"},
    {M_PI / 6.0,   M_PI / 6.0, M_PI / 3.0,  "π/6 + π/6 = π/3 (in range)"},
    {0.0,          0.0,        0.0,         "0 + 0 = 0"},
  }};

  for (const auto& testCase : testCases) {
    double initial = std::get<0>(testCase);
    double offset = std::get<1>(testCase);
    double expected = std::get<2>(testCase);
    const auto& desc = std::get<3>(testCase);

    if (desc.find("+") != std::string::npos) {
      AnglePi2ToPi2 angle(initial);
      angle += offset;
      EXPECT_NEAR(angle.radian(), expected, EPSILON) << desc;
    } else if (desc.find("-") != std::string::npos) {
      AnglePi2ToPi2 angle(initial);
      angle -= offset;
      EXPECT_NEAR(angle.radian(), expected, EPSILON) << desc;
    }
  }
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
