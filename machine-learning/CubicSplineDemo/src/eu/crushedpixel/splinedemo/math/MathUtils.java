package eu.crushedpixel.splinedemo.math;
import java.awt.Point;


public class MathUtils {

	public static double getAngle(Point p1, Point p2) {
		double xDiff = p2.x - p1.x;
        double yDiff = p2.y - p1.y;
        return 180-(Math.toDegrees(Math.atan2(yDiff, xDiff))-90);
	}
	
	public static Point movePoint(Point p, double angle, int distance) {
		p = new Point(p);
	    p.x += distance * Math.sin(angle);
	    p.y += distance * Math.cos(angle);
	    
	    return p;
	}
}
