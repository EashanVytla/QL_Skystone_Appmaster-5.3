package org.firstinspires.ftc.teamcode.Universal.Math;

import org.opencv.core.Point;

import java.util.ArrayList;

public class Circle extends Ellipse {

    public Circle (Pose position, double radius) {
        super(position, radius, radius);
    }

    public static ArrayList<Point> lineCenterIntercection(Point center, double radius, Point linePoint1, Point linePoint2){
        //Checking if the points are basically on the same line to make sure that no unecessary math is being taken place
        if(Math.abs(linePoint1.y - linePoint2.y) <= 0.003){
            linePoint1.y = linePoint2.y + 0.003;
        }else if(Math.abs(linePoint1.x - linePoint2.x) <= 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }

        //Finding the slope
        double m1 = (linePoint1.y - linePoint2.y) / (linePoint1.x - linePoint1.x);

        //Removing the offset from the circles to normalize them to (0,0)
        double x1 = linePoint1.x - center.x;
        double y1 = linePoint1.x - center.x;

        //Finding the traditional A, B, and C variables in the Quadratic Equation
        double quadraticA = 1.0 + Math.pow(m1, 2);
        double quadraticB = (2.0 * m1 * y1) + (2.0 * Math.pow(m1, 2.0) * x1);
        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1,2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1,2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try{
            //This is finding the two x intersections and the two y intersections
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC))/ (2.0 * quadraticA));
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC))/ (2.0 * quadraticA));
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            //Reappllying the offset onto the circle
            xRoot1 += center.x;
            yRoot1 += center.y;

            xRoot2 += center.x;
            yRoot2 += center.y;

            //Checking for if any of the points are not on the line before adding them into the array
            double minX = linePoint1.x < linePoint2.x ?  linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ?  linePoint1.x : linePoint2.x;

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        }catch(Exception e){

        }
        return allPoints;
    }
}
