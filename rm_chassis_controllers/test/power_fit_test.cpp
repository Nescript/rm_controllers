#include <gtest/gtest.h>
#include <rm_chassis_controllers/bipedal_wheel_controller/powerID.h>
#include <random>
#include <vector>

using Vector = Eigen::Matrix<double, 12, 1>;
static Vector features(double v, double w, double a, double z)
{
  Vector f;
  f << 1, v * a, w * z, a * a, z * z, std::abs(v), std::abs(w), v * v, w * w, std::abs(a), std::abs(z), std::abs(v * w);
  return f;
}
TEST(PowerFit, GlobalCurrentCoefficientsMatchExplicitHistory)
{
  PowerModel model;
  EXPECT_TRUE(std::isnan(model.predict(1, 1, 1, 1)));
  model.init(0.97, 0.001);
  Vector truth;
  truth << 10, 20, 2, 1, 2, 3, 4, 2, 1, 2, 1, 3;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> draw(-3, 3);
  std::vector<Vector> xs;
  std::vector<double> ys;
  for (int i = 0; i < 1000; ++i)
  {
    const double v = draw(rng), w = draw(rng), a = draw(rng), z = draw(rng);
    auto f = features(v, w, a, z);
    // Change the model halfway through: global score must include old samples.
    double y = f.dot(truth) + (i > 500 ? 8 : 0) + 0.1 * draw(rng);
    ASSERT_TRUE(model.updateD(0, v, w, a, z, y));
    xs.push_back(f);
    ys.push_back(y);
    if (i == 499 || i == 999)
    {
      ASSERT_TRUE(model.solve());
      auto fit = model.evaluate();
      double mean = 0, sse = 0, bias = 0, sst = 0;
      for (double y0 : ys)
        mean += y0 / ys.size();
      for (size_t j = 0; j < ys.size(); ++j)
      {
        const double e = xs[j].dot(model.getC()) - ys[j];
        sse += e * e;
        bias += e;
        sst += (ys[j] - mean) * (ys[j] - mean);
      }
      EXPECT_TRUE(fit.metrics_valid);
      EXPECT_TRUE(fit.r2_valid);
      EXPECT_EQ(fit.sample_count, ys.size());
      EXPECT_NEAR(fit.rmse, std::sqrt(sse / ys.size()), 1e-8);
      EXPECT_NEAR(fit.bias, bias / ys.size(), 1e-9);
      EXPECT_NEAR(fit.r2, 1 - sse / sst, 1e-10);
    }
  }
  auto count = model.evaluate().sample_count;
  EXPECT_FALSE(model.updateD(0, NAN, 0, 0, 0, 1));
  EXPECT_FALSE(model.updateD(0, 1, 0, 0, 0, INFINITY));
  EXPECT_FALSE(model.updateD(0, 1e200, 0, 0, 0, 1));
  EXPECT_EQ(model.evaluate().sample_count, count);
  const Vector old = model.getC();
  model.setAlphaRidge(-1e100);  // Force rejected solve, retain applied coefficients.
  EXPECT_FALSE(model.solve());
  EXPECT_TRUE(model.getC().isApprox(old));
  EXPECT_TRUE(model.evaluate().metrics_valid);
  model.init(0.999, 0.001);
  EXPECT_EQ(model.evaluate().sample_count, 0u);
  EXPECT_FALSE(model.evaluate().model_valid);
  EXPECT_FALSE(model.evaluate().metrics_valid);
}
TEST(PowerFit, ConstantPowerAndClampedCoefficients)
{
  PowerModel model;
  // Rank deficient but positive raw c[1,2]; applied values are clamped.
  for (int i = 0; i < 100; ++i)
    ASSERT_TRUE(model.updateD(0, 1, 1, 1, 1, 12));
  ASSERT_TRUE(model.solve());
  EXPECT_DOUBLE_EQ(model.getC()(1), 5.0);
  auto fit = model.evaluate();
  EXPECT_TRUE(fit.metrics_valid);
  EXPECT_FALSE(fit.r2_valid);
  EXPECT_TRUE(std::isnan(fit.r2));
  EXPECT_DOUBLE_EQ(model.predict(1, 1, 1, 1), features(1, 1, 1, 1).dot(model.getC()));
  EXPECT_DOUBLE_EQ(model.predict(0, 0, 0, 0), model.getC()(0));
  EXPECT_TRUE(std::isnan(model.predict(NAN, 0, 0, 0)));
  const double error = features(1, 1, 1, 1).dot(model.getC()) - 12;
  EXPECT_NEAR(fit.rmse, std::abs(error), 1e-10);
  EXPECT_NEAR(fit.bias, error, 1e-10);
}

TEST(PowerFit, RejectsInsufficientExcitationAndAcceptsConfiguredModel)
{
  PowerModel model;
  Vector configured;
  configured << 10, 20, 2, 1, 2, 3, 4, 2, 1, 2, 1, 3;
  EXPECT_TRUE(model.setC(configured));
  EXPECT_TRUE(model.getC().isApprox(configured));
  configured(5) = -1;
  EXPECT_FALSE(model.setC(configured));

  for (int i = 0; i < 1000; ++i)
    ASSERT_TRUE(model.updateD(0, 1, 1, 1, 1, 12));
  EXPECT_FALSE(model.hasSufficientExcitation(500, 1e-6));

  model.init(0.999, 0.001);
  configured << 10, 20, 2, 1, 2, 3, 4, 2, 1, 2, 1, 3;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> draw(-3, 3);
  for (int i = 0; i < 2000; ++i)
  {
    const double v = draw(rng), w = draw(rng), a = draw(rng), z = draw(rng);
    ASSERT_TRUE(model.updateD(0, v, w, a, z, features(v, w, a, z).dot(configured)));
  }
  EXPECT_TRUE(model.hasSufficientExcitation(500, 1e-6));
  EXPECT_FALSE(model.solve(3000));
  EXPECT_TRUE(model.solve(500));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
