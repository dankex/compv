package eu.crushedpixel.splinedemo;
import java.awt.Canvas;

import javax.swing.JFrame;

import eu.crushedpixel.splinedemo.ui.PaintCanvas;

public class Main {
	
	public static void main(String[] args) {
		JFrame frame = new JFrame();
		frame.setSize(800, 800);
		frame.setResizable(false);
		frame.setTitle("Cubic Spline Demo");
		frame.setVisible(true);
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		
		Canvas canvas = new PaintCanvas();
		canvas.setSize(800, 800);
		canvas.setVisible(true);
		
		frame.add(canvas);
	}
}
