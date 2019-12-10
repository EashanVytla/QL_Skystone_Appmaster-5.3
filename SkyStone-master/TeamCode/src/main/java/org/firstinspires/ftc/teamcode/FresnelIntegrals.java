package org.firstinspires.ftc.teamcode;

public class FresnelIntegrals {
    private static final double S_CUTOFF = 2.2538, C_CUTOFF = 2.303;
    private static final double[] S_COEFFICIENTS = new double[]{
            0.656233747738434,
            -0.115656562228975,
            0.00907999046492011,
            -0.000391180576348680,
            0.0000105833265842972,
            -1.96107874116241E-7,
            2.64225214635068E-9,
            -2.70393837225454E-11,
            2.17250752056569E-13,
            -1.40662513593184E-15,
    };

    private static final double[] C_COEFFICIENTS = new double[]{
            1.25331413731550,
            -0.309242868139914,
            0.0353252867175832,
            -0.00201142272264001,
            0.0000677718763722813,
            -0.00000150409885692290,
            2.36168238376633E-8,
            -2.76014480494450E-10,
            2.49370133245328E-12,
            -1.79339097842879E-14,
    };

    public static double taylorS(double x){
        double out = 0;
        for (int i = 0; i < S_COEFFICIENTS.length; i++) {
            out += S_COEFFICIENTS[i]*Math.pow(x, 4*i+3);
        }
        return out;
    }

    public static double taylorC(double x){
        double out = 0;
        for (int i = 0; i < C_COEFFICIENTS.length; i++) {
            out += C_COEFFICIENTS[i]*Math.pow(x, 4*i+1);
        }
        return out;
    }

    public static double approxS(double x){
        double out = 0.015467;
        double PI = Math.PI;
        double halfPI = Math.PI/2;
        double absX = Math.abs(x);
        double sqrtX = Math.sqrt(absX);
        double xSquare = x*x;
        double absCubeX = absX*x*x;
        out -= Math.cos(halfPI*xSquare)/(PI*(absX+16.731277*PI*Math.exp(-1.576388*PI*sqrtX)));
        out += 8/25.0*(1-Math.exp(-.6087077*PI*absCubeX));
        out += 2/25.0*(1-Math.exp(-1.71402838*PI*xSquare));
        out += 1/10.0*(1-Math.exp(-9/10.0*PI*x));
        return out*1.2;
    }

    public static double approxC(double x){
        double out = 0.01146;
        double PI = Math.PI;
        double halfPI = PI/2;
        double squareX = x*x;
        double absX = Math.abs(x);
        double sqrtX = Math.sqrt(x);
        double absCubeX = absX*x*x;
        out += Math.sin(halfPI*squareX)/(PI*(absX+20.0*PI*Math.exp(-200.0*PI*sqrtX)));
        out += 8/25.0*(1-Math.exp(-69/100.0*PI*absCubeX));
        out += 2/25.0*(1-Math.exp(-9/2.0*PI*squareX));
        out += 1/10.0*(1-Math.exp(-1.5529406*PI*absX));
        return out*1.25;
    }

    public static double S(double x){
        int polarity = x<0?-1:1;
        double scale = (0.0011184 * Math.abs(x)) + 0.79688;
        if(x*polarity<S_CUTOFF){
            return taylorS(x) * scale;
        }
        return approxS(x*polarity)*polarity * scale;
    }

    public static double C(double x){
        int polarity = x<0?-1:1;
        double scale = (-0.00198 * Math.abs(x)) + 0.798;
        if(x*polarity<C_CUTOFF){
            return taylorC(x) * scale;
        }
        return approxC(x*polarity)*polarity * scale;
    }
}