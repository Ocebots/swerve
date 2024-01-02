package frc.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

public class CustomSlewRateLimiter {
  private double rateLimit;
  private double prevVal;
  private double prevTime;

  public CustomSlewRateLimiter(double rateLimit, double initialVal) {
    this.rateLimit = rateLimit;
    this.prevVal = initialVal;
    this.prevTime = MathSharedStore.getTimestamp();
  }

  public void setRateLimit(double rateLimit) {
    this.rateLimit = rateLimit;
  }

  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;
    prevVal += MathUtil.clamp(input - prevVal, -rateLimit * elapsedTime, rateLimit * elapsedTime);
    prevTime = currentTime;
    return prevVal;
  }
}
