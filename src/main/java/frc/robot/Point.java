package frc.robot;

public class Point {
    public float x;
    public float y;

    public Point(float x, float y){
        this.x = x;
        this.y = y;
    }

    public boolean equals(Point other){
        if (x == other.x && y == other.y)
            return true;
        return false;
    }
}
