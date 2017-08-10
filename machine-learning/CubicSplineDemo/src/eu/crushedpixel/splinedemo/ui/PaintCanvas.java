package eu.crushedpixel.splinedemo.ui;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;
import java.util.List;

import eu.crushedpixel.splinedemo.math.BasicSpline;


@SuppressWarnings("serial")
public class PaintCanvas extends DoubleBuffer {

	private List<Point> points;
	private BasicSpline spline = new BasicSpline();

	private Point grabbedPoint = null;

	private Point grabClick = null;

	private Point getNearestPoint(int x, int y, int radius) {
		for(Point pos : points) {
			double dist = Math.sqrt(Math.pow(x-pos.x, 2) + Math.pow(y-pos.y, 2));
			if(dist <= radius) {
				return pos;
			}
		}
		return null;
	}

	public PaintCanvas() {
		points = new ArrayList<>();

		this.addMouseListener(new MouseAdapter() {
			@Override
			public void mousePressed(MouseEvent e) {
				switch (e.getButton()) {
				case 1: //Linke Maustaste
					grabbedPoint = getNearestPoint(e.getX(), e.getY(), POINT_RADIUS+BORDER);
					
					if(grabbedPoint != null) {
						grabClick = new Point(e.getX()-grabbedPoint.x, e.getY()-grabbedPoint.y);
						
					} else {
						points.add(new Point(e.getX(), e.getY()));
						recalcSpline();
						PaintCanvas.this.paint(PaintCanvas.this.getGraphics());
					}
					break;
					
				case 3: //Rechte Maustaste
					Point toRemove = getNearestPoint(e.getX(), e.getY(), POINT_RADIUS+BORDER);
					if(toRemove != null) {
						points.remove(toRemove);	
						recalcSpline();
						PaintCanvas.this.paint(PaintCanvas.this.getGraphics());
					}
					break;
				}
			}

			@Override
			public void mouseReleased(MouseEvent e) {
				if(e.getButton() == 1) {
					if(grabbedPoint != null) {
						grabbedPoint = null;
						
						recalcSpline();
						PaintCanvas.this.paint(PaintCanvas.this.getGraphics());
					}
				}
			}
		});

		this.addMouseMotionListener(new MouseMotionListener() {
			@Override
			public void mouseMoved(MouseEvent e) {}

			@Override
			public void mouseDragged(MouseEvent e) {
				if(grabbedPoint != null) {
					grabbedPoint.x = e.getX()-grabClick.x;
					grabbedPoint.y = e.getY()-grabClick.y;
					
					recalcSpline();
					PaintCanvas.this.paint(PaintCanvas.this.getGraphics());
				}
			}
		});
		
		this.addKeyListener(new KeyAdapter() {

			@Override
			public void keyTyped(KeyEvent e) {
				if(e.getKeyChar() == 'c') {
					points = new ArrayList<>();
					recalcSpline();
					PaintCanvas.this.paint(PaintCanvas.this.getGraphics());
				}
			}
			
		});
	}

	public void recalcSpline() {
		spline = new BasicSpline();
		if(points.size() > 2) {
			for(Point p : points) {
				spline.addPoint(p);
			}
			spline.calcSpline();
		}
	}

	private static final int POINT_RADIUS = 15;
	private static final int BORDER = 3;
	private static final int LINE_RADIUS = 10;

	@Override
	public void paintBuffer(Graphics g) {
		Graphics2D g2D = (Graphics2D)g;
		g2D.setColor(Color.LIGHT_GRAY);
		g2D.fillRect(0, 0, this.getWidth(), this.getHeight());

		Point pointBefore = null;
		if(points.size() > 2) {
			for(float f = 0; f<=1.1; f+=0.01) {
				Point p = spline.getPoint(f);
				Point pnt = new Point(p.x, p.y);
				drawConnectionPoint(pnt, g2D);

				if(pointBefore != null) drawConnection(pnt, pointBefore, g2D);

				pointBefore = pnt;
			}
		}

		for(Point p : points) {
			drawPoint(p, g2D);
		}
	}

	private void drawConnection(Point a, Point b, Graphics2D g2D) {
		g2D.setColor(Color.RED);
		g2D.setStroke(new BasicStroke(LINE_RADIUS*2));
		g2D.drawLine(a.x, a.y, b.x, b.y);
	}

	private void drawConnectionPoint(Point p, Graphics2D g2D) {
		g2D.setColor(Color.RED);
		g2D.fillOval(p.x-LINE_RADIUS, p.y-LINE_RADIUS, LINE_RADIUS*2, LINE_RADIUS*2);
	}

	private void drawPoint(Point p, Graphics2D g2D) {
		g2D.setColor(Color.BLACK);
		g2D.fillOval(p.x-(POINT_RADIUS+BORDER), p.y-(POINT_RADIUS+BORDER), (POINT_RADIUS+BORDER)*2, (POINT_RADIUS+BORDER)*2);

		g2D.setColor(Color.GRAY);
		g2D.fillOval(p.x-POINT_RADIUS, p.y-POINT_RADIUS, POINT_RADIUS*2, POINT_RADIUS*2);
	}


}
