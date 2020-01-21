package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

import java.util.ArrayList;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import static org.firstinspires.ftc.teamcode.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.MathFunctions.lineCircleIntersection;

public class RobotMovement {
    static int index = 0;
    static int previous_index = 0;

    public static Pose2d followPath(ArrayList<CurvePoint> allPoints, double followAngle, Pose2d p){
        previous_index = index;

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(p.getX(), p.getY()), allPoints.get(0).followDistance); //todo: optimize lookahead computation

        index = getCurrentLine(followMe.toPoint(), allPoints);

        //System.out.println("Size: " + allPoints.size());
        if (Math.hypot(allPoints.get(allPoints.size() - 1).x - p.getX(), allPoints.get(allPoints.size() - 1).y - p.getY()) > allPoints.get(0).followDistance + 10 || index < allPoints.size() - 2) {
            //goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
            return new Pose2d(followMe.x, followMe.y, getFollowAngle(p, followMe, followAngle));
        }
        else{
            return new Pose2d(allPoints.get(allPoints.size() - 1).x, allPoints.get(allPoints.size() - 1).y, getFollowAngle(p, allPoints.get(allPoints.size() - 1), followAngle));
            //goToPoint(allPoints.get(allPoints.size() - 1).x, allPoints.get(allPoints.size() - 1).y, allPoints.get(allPoints.size() - 1).moveSpeed, followAngle, allPoints.get(allPoints.size() - 1).turnSpeed);
        }
    }

    public static double getFollowAngle(Pose2d p, CurvePoint followMe, double preferredAngle){
        double absoluteAngleToTarget = Math.atan2(followMe.y-p.getY(),followMe.x-p.getX());
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (p.getHeading() - Math.toRadians(90)));

        return relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
    }

    public static Pose2d followPath(ArrayList<CurvePoint> allPoints, double followAngle, double holdAngle, Pose2d p){
        previous_index = index;

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(p.getX(), p.getY()), allPoints.get(0).followDistance); //todo: optimize lookahead computation

        index = getCurrentLine(followMe.toPoint(), allPoints);

        //System.out.println("Size: " + allPoints.size());
        if (Math.hypot(allPoints.get(allPoints.size() - 1).x - p.getX(), allPoints.get(allPoints.size() - 1).y - p.getX()) > allPoints.get(0).followDistance + 10 || index < allPoints.size() - 2) {
            return new Pose2d(followMe.x, followMe.y, holdAngle);
        }
        else{
            return new Pose2d(allPoints.get(allPoints.size() - 1).x, allPoints.get(allPoints.size() - 1).y, holdAngle);
        }
    }

    public static double distToLine(Point p1, Point p2, Point robotLocation){

        double m = (p2.x == p1.x ? 999 : (p2.y - p1.y) / (p2.x - p1.x));
        double m2 = (m == 0 ? -999 : -1 / m);
        double b2 = p1.y - (m2 * p2.x);
        double b = p1.y - (m * p1.x);
        double x = (b2 - b) / (m - m2);
        double y = (m * x) + b;

        return Math.hypot(robotLocation.x - x, robotLocation.y - y);
    }

    public static int clip(int num, int low, int high){
        if (num > high){
            return high;
        }
        else if (num < low){
            return low;
        }
        else{
            return num;
        }
    }

    public static int getCurrentLine(Point intersection, ArrayList<CurvePoint> allPoints){
        int currentline = 0;

        for (int i = 0; i < allPoints.size() - 1; i++){
            double startx = allPoints.get(i).x < allPoints.get(i + 1).x ? allPoints.get(i).x : allPoints.get(i + 1).x;
            double starty = allPoints.get(i).y < allPoints.get(i + 1).y ? allPoints.get(i).y : allPoints.get(i + 1).y;
            double endx = allPoints.get(i).x > allPoints.get(i + 1).x ? allPoints.get(i).x : allPoints.get(i + 1).x;
            double endy = allPoints.get(i).y > allPoints.get(i + 1).y ? allPoints.get(i).y : allPoints.get(i + 1).y;

            double m = (endx == startx ? Double.MAX_VALUE : (endy - starty) / (endx - startx));
            double b = starty - (m * startx);

            //System.out.println("M: " + m + " B: " + b);

            if (intersection.x >= startx - 2 && intersection.x <= endx + 2 && intersection.y >= starty - 2 && intersection.y <= endy + 2){
                //if (distToLine(allPoints.get(i).toPoint(), allPoints.get(i + 1).toPoint(), intersection) < 10) {
                System.out.println("Index Jump: " + Math.abs(i - previous_index));
                if (Math.abs(i - previous_index) < 2) {
                    currentline = i;
                    break;
                }
                //System.out.println("Distance To Line: " + distToLine(allPoints.get(i).toPoint(), allPoints.get(i + 1).toPoint(), intersection))
                //}
            }
        }
        return currentline;
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        double runningDistance = 0;
        double previous_dist = 0;

        //System.out.println("Index: " + index);
        //System.out.println("Previous Index: " + previous_index);
        //System.out.println("Lower Bound: " + Math.min(Math.max(0, index), pathPoints.size() - 3) + " Upper Bound: " + Math.min(pathPoints.size() - 1, index + 2));


        for (int i = Math.min(Math.max(0, index), pathPoints.size() - 3); i < Math.min(index + 2, pathPoints.size() - 1); i++){
            CurvePoint start = pathPoints.get(i);
            CurvePoint end = pathPoints.get(i + 1);
            runningDistance += previous_dist;

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, start.toPoint(), end.toPoint());

            double closestDistance = Double.MAX_VALUE;

            for (Point thisIntersection : intersections){
                double dist = Math.hypot(thisIntersection.x - pathPoints.get(i + 1).x, thisIntersection.y - pathPoints.get(i + 1).y);

                if (dist < closestDistance){
                    closestDistance = dist;
                    followMe.setPoint(thisIntersection);
                }
            }
            previous_dist = Math.hypot(end.x - start.x, end.y - start.y);
        }
        //System.out.println("X: " + followMe.x + " Y: " + followMe.y);
        return followMe;
    }

    /**
     *
     * @param x
     * @param y
     * @param movementSpeed
     */
}
