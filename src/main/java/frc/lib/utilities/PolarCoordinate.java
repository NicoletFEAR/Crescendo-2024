package frc.lib.utilities;

import java.util.*;

public class PolarCoordinate {
    private double theta;
    private double r;

    public PolarCoordinate(double theta, double r) {
        this.theta = theta;
        this.r = r;
    }

    // Implement equals and hashCode methods for proper comparison
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        PolarCoordinate thetaR = (PolarCoordinate) obj;
        return Double.compare(thetaR.theta, theta) == 0 && Double.compare(thetaR.r, r) == 0;
    }

    public double getTheta() {
        return theta;
    }

    public double getR() {
        return r;
    }

    @Override
    public int hashCode() {
        return Objects.hash(theta, r);
    }
}