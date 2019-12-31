package org.firstinspires.ftc.teamcode.Universal.Math;

public class Vector2D {
    public static final double EPSILON = 0.00001;

    private final double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D copy() {
        return new Vector2D(x, y);
    }

    public Vector2D normalized() {
        double norm = norm();
        if (norm < EPSILON) {
            return new Vector2D(1, 0);
        } else {
            return multiplied(1.0 / norm());
        }
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public Vector2D normalize(Vector2D other){
        double x = (this.x + other.x) / 2.0;
        double y = (this.y + other.y) / 2.0;

        return new Vector2D(x, y);
    }

    public double dot(Vector2D other) {
        return x * other.x() + y * other.y();
    }

    public Vector2D multiplied(double scalar) {
        return new Vector2D(scalar * x, scalar * y);
    }

    public Vector2D added(Vector2D other) {
        return new Vector2D(x + other.x, y + other.y);
    }

    public Vector2D negated() {
        return new Vector2D(-x, -y);
    }

    public Vector2D rotated(double angle) {
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Vector2D(newX, newY);
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof Vector2D) {
            Vector2D otherVector = (Vector2D) other;
            return Math.abs(x - otherVector.x) < EPSILON && Math.abs(y - otherVector.y) < EPSILON;
        }
        return false;
    }

    @Override
    public String toString() {
        return "<" + x + ", " + y + ">";
    }

    public static double getCosAngle(Vector2D v1, Vector2D v2) {
        double dot = v1.x * v2.x + v1.y * v2.y;
        return dot / (v1.norm() * v2.norm());
    }

    public static double distance(Vector2D v1, Vector2D v2) {
        return v1.added(v2.negated()).norm();
    }
}
