/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

public class PurePursuitHandler {
    private static ArrayList<Point> path = new ArrayList<>();
    private static float lookaheadDistance = Constants.lookaheadDistance;
    private static boolean finished = false;
    private static float finishedThreshold = 0.5f;

    /**
     * An all in one function which gives you the next velocities in a path, given a few parameters.
     * This function exists purely so that the end user does not have to call multiple methods to get this
     * velocity, but rather just this one method.
     *
     * @return
     */
    public static double[] getNextVelocities(){

        return null;
    }

    /**
     * Generate the furthest lookahead point on the path that is distance r from the point (x, y).
     *
     * @param robotPosition The point of the origin. (In this case, the robot position).
     * @return A Point object if the lookahead point exists, or null.
     */
    public static Point getLookaheadPoint(Point robotPosition){
        Point lookahead = null;

        for (int i = 0; i < path.size() - 1; i++){
            Point segmentStart = path.get(i);
            Point segmentEnd = path.get(i + 1);

            Point p1 = new Point(segmentStart.x - robotPosition.x, segmentStart.y - robotPosition.y);
            Point p2 = new Point(segmentEnd.x - robotPosition.x, segmentEnd.y - robotPosition.y);

            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            float distance = distanceFormula(dx, dy);
            float determinant = p1.x * p2.y - p1.y * p2.x;
            float discriminant = (float) (Math.pow(lookaheadDistance, 2) * Math.pow(distance, 2) - Math.pow(determinant, 2));

            if (discriminant < 0 || p1.equals(p2)) continue;

            float x1 = (float) ((determinant * dy + signum(dy) * dx * Math.sqrt(discriminant)) / (Math.pow(distance, 2)));
            float x2 = (float) ((determinant * dy - signum(dy) * dx * Math.sqrt(discriminant)) / (Math.pow(distance, 2)));

            float y1 = (float) ((-determinant * dx + Math.abs(dy) * Math.sqrt(discriminant)) / Math.pow(distance, 2));
            float y2 = (float) ((-determinant * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (Math.pow(distance, 2)));

            boolean validIntersection1 = Math.min(p1.x, p2.x) < x1 && x1 < Math.max(p1.x, p2.x)
                    || Math.min(p1.y, p2.y) < y1 && y1 < Math.max(p1.y, p2.y);
            boolean validIntersection2 = Math.min(p1.x, p2.x) < x2 && x2 < Math.max(p1.x, p2.x)
                    || Math.min(p1.y, p2.y) < y2 && y2 < Math.max(p1.y, p2.y);

            if (validIntersection1 || validIntersection2) lookahead = null;

            // select the first one if it's valid
            if (validIntersection1) {
                lookahead = new Point(x1 + robotPosition.x, y1 + robotPosition.y);
            }

            // select the second one if it's valid and either lookahead is none,
            // or it's closer to the end of the segment than the first intersection
            if (validIntersection2) {
                if (lookahead == null || Math.abs(x1 - p2.x) > Math.abs(x2 - p2.x) || Math.abs(y1 - p2.y) > Math.abs(y2 - p2.y)) {
                    lookahead = new Point(x2 + robotPosition.x, y2 + robotPosition.y);
                }
            }
        }

        if (path.size() > 0) {
            Point lastPoint = path.get(path.size() - 1);

            float endX = lastPoint.x;
            float endY = lastPoint.y;

            // if we are closer than lookahead distance to the end, set it as the lookahead
            if (Math.sqrt((endX - robotPosition.x) * (endX - robotPosition.x) + (endY - robotPosition.y) * (endY - robotPosition.y)) <= lookaheadDistance) {
                lookahead = new Point(endX, endY);
            }
        }

        if (lookahead == null){

        }
        if (distanceFormula(robotPosition, lookahead) < finishedThreshold){

            finished = true;
        }

        
        return lookahead;
    }

    /**
     * A helper method to calculate the curvature between two points, notably the robot position and lookahead point.
     *
     * @param robotAngle Angle of the robot.
     * @param robot Position of the robot.
     * @param lookahead Point towards which the robot should travel.
     * @return The curvature between two distinct Point objects.
     */
    public static double calculateCurvature(float robotAngle, Point robot, Point lookahead) {
        // if the robot has strayed off its path, just go forward until we find something again
        if (lookahead == null)
            return 0;

        /* curvature = 2x/L^2 */
        /*
         * a = − tan(robot angle)
         * b = 1
         * c = tan(robot angle) * robot x − robot y The
         * point-line distance formula is: d = |ax + by + c| /sqrt(a^2 + b^2) Plugging
         * in our coefficients and the coordinates of the lookahead point gives:
         * x = |a * lookahead x + b * lookahead y + c| /sqrt(a^2 + b^2)
         */
        /*
         * side = signum(sin(robot angle) * (Lx - Rx) - cos(robot angle) * (Ly - Ry))
         * Signed curvature = curvature * sign
         */
        double a = -1 * Math.tan(robotAngle);
        double b = 1;
        double c = Math.tan(robotAngle) * robot.x - robot.y;
        double x = Math.abs(a * lookahead.x + b * lookahead.y + c) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        double curvature = (2 * x) / ((Math.pow(lookahead.x - robot.x, 2) + Math.pow(lookahead.y - robot.y, 2)));
        double side = Math.signum((Math.sin(robotAngle) * (lookahead.x - robot.x)) - (Math.cos(robotAngle) * (lookahead.y - robot.y)));
        return (curvature * side);
    }

    /**
     * Given a few parameters, this method will get the velocity each wheel must follow.
     *
     * @param curvature The curvature from robot to lookahead point.
     * @param maxVelocity Max velocity the robot can follow.
     * @param wheelBaseWidth Wheel base of the robot.
     * @return An array containing the left wheel velocity in arr[0] and right wheel in arr[1]
     */
    public static double[] getTargetVelocities(double curvature, float maxVelocity, float wheelBaseWidth) {

        double[] velocities = {0, 0};

        double velocity = maxVelocity;
        /*
         * Target wheel velocities are given by L = V * (2 + CT)/2; R = V * (2 - CT)/2
         */

        /* Left Wheel */
        double targetLeftVelocity = velocity * (2 + (curvature * wheelBaseWidth)) / 2;
        velocities[0] = targetLeftVelocity;

        /* Right Wheel */
        double targetRightVelocity = velocity * (2 - (curvature * wheelBaseWidth)) / 2;
        velocities[1] = targetRightVelocity;

        return velocities;
    }

    /**
     * Add a new point to the path for the robot to follow.
     *
     * @param point the new Point to add to the path.
     */
    public static void addPoint(Point point){
        path.add(point);
    }

    /**
     * Gets the path with all the Points.
     *
     * @return An ArrayList consisting of all Point objects in the path.
     */
    public static ArrayList<Point> getPath(){
        return path;
    }

    /**
     * Gets the lookahead distance.
     *
     * @return The lookahead distance of the robot.
     */
    public static float getLookaheadDistance(){
        return lookaheadDistance;
    }

    /**
     * Whether or not the algorithm is complete.
     *
     * @return A boolean indicating if the robot has reached its destination.
     */
    public static boolean isFinished(){
        return finished;
    }

    /**
     * Changes the lookahead distance.
     *
     * @param add the distance to add to the currrent lookahead distance.
     */
    public static void addToLookaheadDistance(float add){
        lookaheadDistance += add;
    }

    /**
     * Returns the sign of the input number n. Note that the function returns 1 for n = 0 to satisfy the requirements
     * set forth by the line-circle intersection formula.
     *
     * @param n The number to return the sign of.
     * @return A float value of the sign of the number (-1.0f for n < 0, else 1.0f).
     */
    private static float signum(float n) {
        if (n == 0) return 1;
        else return Math.signum(n);
    }

    /**
     * Calculates the distance between two Point objects.
     *
     * @param a The first point
     * @param b The second point
     * @return The distance between two Point objects
     */
    private static float distanceFormula(Point a, Point b){
        try {
            return (float) Math.sqrt(Math.pow(a.y - b.y, 2) + Math.pow(a.x - b.x, 2));
        }
        catch (java.lang.NullPointerException e){
            System.out.println("Error Encountered:" + e.getMessage());

            e.printStackTrace();
            return 0;
        }
    }

    /**
     * Calculates the distance using the dx and dy of two points.
     *
     * @param dx x distance between two points
     * @param dy y distance between two points
     * @return The distance using the dx and dy of two points.
     */
    private static float distanceFormula(float dx, float dy){
        return (float) Math.sqrt(Math.pow(dy, 2) + Math.pow(dx, 2));
    }
}
