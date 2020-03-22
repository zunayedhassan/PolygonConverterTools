package zunayedhassan.PolygonConverterExperiment.Model;

import java.util.ArrayList;
import javafx.scene.shape.ArcTo;
import javafx.scene.shape.Circle;
import javafx.scene.shape.ClosePath;
import javafx.scene.shape.CubicCurveTo;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.Shape;
import javafx.scene.shape.Ellipse;
import javafx.scene.shape.HLineTo;
import javafx.scene.shape.LineTo;
import javafx.scene.shape.MoveTo;
import javafx.scene.shape.Path;
import javafx.scene.shape.PathElement;
import javafx.scene.shape.QuadCurveTo;
import javafx.scene.shape.VLineTo;

/**
 *
 * @author Zunayed Hassan
 */
public class ShapeToPolygonConverter {
    public ArrayList<Double> GetConvertedPolygonPoints(Shape shape) {
        ArrayList<Double> points = new ArrayList<>();
        
        // Rectangle
        if (shape instanceof Rectangle) {
            Rectangle rectangle = (Rectangle) shape;
            
            if (this.IsRounedRectangle(rectangle)) {
                points = this.GetRectanglePolygon(rectangle);
            }
            else {
                points = this.GetRoundedRectanglePolygon(rectangle);
            }
        }
        // Circle
        else if (shape instanceof Circle) {
            Circle circle  = (Circle) shape;
            points = this.GetCirclePolygon(circle);
        }
        // Ellipse
        else if (shape instanceof Ellipse) {
            Ellipse ellipse  = (Ellipse) shape;
            points = this.GetEllipsePolygon(ellipse);
        }
        // Path
        else if (shape instanceof Path) {
            Path path = (Path) shape;
            
            for (PathElement pathElement : path.getElements()) {
                // Move To
                if (pathElement instanceof MoveTo) {
                    MoveTo currentPathElement = (MoveTo) pathElement;

                    points.add(currentPathElement.getX());
                    points.add(currentPathElement.getY());
                }
                // Line to
                else if (pathElement instanceof LineTo) {
                    LineTo currentPathElement = (LineTo) pathElement;

                    points.add(currentPathElement.getX());
                    points.add(currentPathElement.getY());
                }
                // Horizontal Line
                else if (pathElement instanceof HLineTo) {
                    HLineTo currentPathElement = (HLineTo) pathElement;

                    points.add(currentPathElement.getX());
                    points.add(points.get(points.size() - 2));
                }
                // Vertical Line
                else if (pathElement instanceof VLineTo) {
                    VLineTo currentPathElement = (VLineTo) pathElement;

                    points.add(points.get(points.size() - 1));
                    points.add(currentPathElement.getY());
                }
                // Arc To
                // NOTE: This is buggy
                else if (pathElement instanceof ArcTo) {
                    ArcTo  arcTo   = (ArcTo) pathElement;
                    double radiusX = arcTo.getRadiusX() * 2;
                    double radiusY = arcTo.getRadiusY() * 2;
                    double centerX = 0;
                    double centerY = 0;
                    
                    if (points.get(points.size() - 2) > arcTo.getX()) {
                        centerX = points.get(points.size() - 2) - radiusX;
                    }
                    else {
                        centerX = points.get(points.size() - 2) + radiusX;
                    }
                    
                    centerY = points.get(points.size() - 1);

                    Ellipse           arcToEllipse    = new Ellipse(centerX, centerY, radiusX, radiusY);
                    ArrayList<Double> tempArcToPoints = this.GetEllipsePolygon(arcToEllipse);
                    
                    double startX = points.get(points.size() - 2);
                    double startY = points.get(points.size() - 1);
                    double endX   = arcTo.getX();
                    double endY   = arcTo.getY();
                    
                    double[] tempArcToVerticesX = new double[tempArcToPoints.size() / 2];
                    double[] tempArcToVerticesY = new double[tempArcToPoints.size() / 2];
                    
                    for (int i = 0, j = 0; i < tempArcToPoints.size(); i += 2, j++) {
                        tempArcToVerticesX[j] = tempArcToPoints.get(i);
                        tempArcToVerticesY[j] = tempArcToPoints.get(i + 1);
                    }
                    
                    int startIndex = this._getNearestPointIndex(tempArcToVerticesX, tempArcToVerticesY, startX, startY);
                    int endIndex   = this._getNearestPointIndex(tempArcToVerticesX, tempArcToVerticesY, endX,   endY  );
                    
                    if (startIndex < endIndex) {
                        for (int i = startIndex; i <= endIndex; i++) {
                            points.add(tempArcToVerticesX[i]);
                            points.add(tempArcToVerticesY[i]);
                        }
                    }
                    else {
                        for (int i = startIndex; i >= endIndex; i--) {
                            points.add(tempArcToVerticesX[i]);
                            points.add(tempArcToVerticesY[i]);
                        }
                    }
                }
                // Qaud Curve To
                else if (pathElement instanceof QuadCurveTo) {
                    QuadCurveTo currentQuadCurveTo = (QuadCurveTo) pathElement;
                    
                    double startX   = points.get(points.size() - 2);
                    double startY   = points.get(points.size() - 1);
                    double controlX = currentQuadCurveTo.getControlX();
                    double controlY = currentQuadCurveTo.getControlY();
                    double endX     = currentQuadCurveTo.getX();
                    double endY     = currentQuadCurveTo.getY();
                    
                    for (double t = 0; t <= 1; t += 0.1) {
                        double[] currentPoint = this._getQuadraticBezierCurvePoint(startX, startY, controlX, controlY, endX, endY, t);
                        
                        points.add(currentPoint[0]);
                        points.add(currentPoint[1]);
                    }
                }
                // Cubic Curve To
                else if (pathElement instanceof CubicCurveTo) {
                    CubicCurveTo currentCubicCurveTo = (CubicCurveTo) pathElement;
                    
                    double startX    = points.get(points.size() - 2);
                    double startY    = points.get(points.size() - 1);
                    double controlX1 = currentCubicCurveTo.getControlX1();
                    double controlY1 = currentCubicCurveTo.getControlY1();
                    double controlX2 = currentCubicCurveTo.getControlX2();
                    double controlY2 = currentCubicCurveTo.getControlY2();
                    double endX      = currentCubicCurveTo.getX();
                    double endY      = currentCubicCurveTo.getY();
                    
                    for (double t = 0; t <= 1; t += 0.1) {
                        double[] currentPoint = this._getCubicBezierCurvePoint(startX, startY, controlX1, controlY1, controlX2, controlY2, endX, endY, t);
                        
                        points.add(currentPoint[0]);
                        points.add(currentPoint[1]);
                    }
                }
                // Close Path
                else if (pathElement instanceof ClosePath) {
                    points.add(points.get(0));
                    points.add(points.get(1));
                }
            }
        }
        
        return points;
    }
    
    public ArrayList<Double> GetRectanglePolygon(Rectangle rectangle) {
        ArrayList<Double> points = new ArrayList<>();
        
        points.add(rectangle.getX());
        points.add(rectangle.getY());

        points.add(rectangle.getX() + rectangle.getWidth());
        points.add(rectangle.getY());

        points.add(rectangle.getX() + rectangle.getWidth());
        points.add(rectangle.getY() + rectangle.getHeight());

        points.add(rectangle.getX());
        points.add(rectangle.getY() + rectangle.getHeight());
        
        return points;
    }
    
    public ArrayList<Double> GetRoundedRectanglePolygon(Rectangle rectangle) {
        ArrayList<Double> points = new ArrayList<>();
        
        double arcWidth  = rectangle.getArcWidth() / 2.0;
        double arcHeight = rectangle.getArcHeight() / 2.0;
        double x         = rectangle.getX();
        double y         = rectangle.getY();
        double width     = rectangle.getWidth();
        double height    = rectangle.getHeight();
        
        points.add(x + arcWidth);
        points.add(y);
        
        this._setRoundedCorner(points, arcWidth, arcHeight, x + width - arcWidth, y + arcHeight, 90, 180);
       
        points.add(x + width);
        points.add(y + height - arcHeight);
        
        this._setRoundedCorner(points, arcWidth, arcHeight, x + width - arcWidth, y + height - arcHeight, 180, 270);

      
        points.add(x + arcWidth);
        points.add(y + height);
        
        this._setRoundedCorner(points, arcWidth, arcHeight, x + arcWidth, y + height - arcHeight, 270, 360);        
        
        points.add(x);
        points.add(y + arcHeight);
        
        this._setRoundedCorner(points, arcWidth, arcHeight, x + arcWidth, y + arcHeight, 0, 90);
        
        return points;
    }
    
    private void _setRoundedCorner(ArrayList<Double> points, double radiusX, double radiusY, double centerX, double centerY, double minAngle, double maxAngle) {
        ArrayList<Double> roundedCornerPoints = this.GetEllipsePolygon(radiusX, radiusY, centerX, centerY, minAngle, maxAngle);
        
        for (double value : roundedCornerPoints) {
            points.add(value);
        }
    }
    
    public ArrayList<Double> GetCirclePolygon(Circle circle) {
        ArrayList<Double> points = new ArrayList<>();
        
        final double TOTAL_ANGLE_OF_CIRCLE = 360;
        final int    TOTAL_VERTICES        = 36;

        double radius  = circle.getRadius();
        double centerX = circle.getCenterX();
        double centerY = circle.getCenterY();

        double[] unsortedVerticesX = new double[TOTAL_VERTICES];
        double[] unsortedVerticesY = new double[TOTAL_VERTICES];
        double[] angles            = new double[TOTAL_VERTICES];
        int[]    verticesIndex     = new int[TOTAL_VERTICES];

        for (int i = 0, j = 0; i < TOTAL_ANGLE_OF_CIRCLE; i += (TOTAL_ANGLE_OF_CIRCLE / TOTAL_VERTICES), j++) {
            double x = centerX + radius * Math.cos(i);
            double y = centerY + radius * Math.sin(i);

            unsortedVerticesX[j] = x;
            unsortedVerticesY[j] = y;

            double anlgeFromCenter = Math.atan2((y - centerY), (x - centerX));
            angles[j] = anlgeFromCenter;
        }

        for (int i = 0; i < verticesIndex.length; i++) {
            verticesIndex[i] = i;
        }

        // Now sort
        for (int i = 0; i < angles.length; i++) {
            for (int j = 0; j < angles.length; j++) {
                if (angles[i] > angles[j]) {
                    double angleTemp  = angles[i];
                    angles[i]         = angles[j];
                    angles[j]         = angleTemp;

                    int indexTemp    = verticesIndex[i];
                    verticesIndex[i] = verticesIndex[j];
                    verticesIndex[j] = indexTemp;
                }
            }
        }

        for (int i = 0; i < verticesIndex.length; i++) {
            double x = unsortedVerticesX[verticesIndex[i]];
            double y = unsortedVerticesY[verticesIndex[i]];

            points.add(x);
            points.add(y);
        }
        
        return points;
    }
    
    public ArrayList<Double> GetEllipsePolygon(Ellipse ellipse) {
        ArrayList<Double> points = new ArrayList<>();
        
        double radiusX  = ellipse.getRadiusX();
        double radiusY  = ellipse.getRadiusY();
        double centerX = ellipse.getCenterX();
        double centerY = ellipse.getCenterY();
        
        return this.GetEllipsePolygon(radiusX, radiusY, centerX, centerY, 0, 360);
    }
    
    public ArrayList<Double> GetEllipsePolygon(double radiusX, double radiusY, double centerX, double centerY, double minAngle, double maxAngle) {
        minAngle -= 180;
        maxAngle -= 180;
        
        ArrayList<Double> points = new ArrayList<>();

        final double TOTAL_ANGLE_OF_CIRCLE = 360;
        final int    TOTAL_VERTICES        = 36;

        double[] unsortedVerticesX = new double[TOTAL_VERTICES];
        double[] unsortedVerticesY = new double[TOTAL_VERTICES];
        double[] angles            = new double[TOTAL_VERTICES];
        int[]    verticesIndex     = new int[TOTAL_VERTICES];

        for (int i = 0, j = 0; i < TOTAL_ANGLE_OF_CIRCLE; i += (TOTAL_ANGLE_OF_CIRCLE / TOTAL_VERTICES), j++) {
            double x = centerX + radiusX * Math.cos(i);
            double y = centerY + radiusY * Math.sin(i);

            unsortedVerticesX[j] = x;
            unsortedVerticesY[j] = y;

            double anlgeFromCenter = Math.atan2((y - centerY), (x - centerX));
            angles[j] = anlgeFromCenter;
        }

        for (int i = 0; i < verticesIndex.length; i++) {
            verticesIndex[i] = i;
        }

        // Now sort
        for (int i = 0; i < angles.length; i++) {
            for (int j = 0; j < angles.length; j++) {
                if (angles[i] < angles[j]) {
                    double angleTemp = angles[i];
                    angles[i]        = angles[j];
                    angles[j]        = angleTemp;

                    int indexTemp    = verticesIndex[i];
                    verticesIndex[i] = verticesIndex[j];
                    verticesIndex[j] = indexTemp;
                }
            }
        }

        for (int i = 0; i < verticesIndex.length; i++) {
            double currentAngle = (angles[i] * 57.2958);
            
            if ((currentAngle >= minAngle) && (currentAngle <= maxAngle)) {
                double x = unsortedVerticesX[verticesIndex[i]];
                double y = unsortedVerticesY[verticesIndex[i]];

                points.add(x);
                points.add(y);
            }
        }

        return points;
    }
    
    public boolean IsRounedRectangle(Rectangle rectangle) {
        return ((rectangle.getArcWidth() == 0) && (rectangle.getArcHeight() == 0));
    }
    
    private double _getDistanceBetweenTwoPoints(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
    
    private int _getNearestPointIndex(double[] verticesX, double[] verticesY, double testPointX, double testPointY) {
        int      index     = 0;
        double[] distances = new double[verticesX.length];
        
        for (int i = 0; i < verticesX.length; i++) {
            distances[i] = this._getDistanceBetweenTwoPoints(verticesX[i], verticesY[i], testPointX, testPointY);
        }
        
        double min = distances[index];
        
        for (int i = 0; i < distances.length; i++) {
            if (min > distances[i]) {
                min = distances[i];
                index = i;
            }
        }
        
        return index;
    }
    
    private double[] _getQuadraticBezierCurvePoint(double startX, double startY, double controlX, double controlY, double endX, double endY, double t) {
        double[] outputPoint = new double[2];
        
        if ((t >= 0) && (t <= 1)) {
            outputPoint[0] = this._getQuadraticBezierCurvePoint(startX, controlX, endX, t);
            outputPoint[1] = this._getQuadraticBezierCurvePoint(startY, controlY, endY, t);
        }
        
        return outputPoint;
    }
    
    private double _getQuadraticBezierCurvePoint(double s, double c, double e, double t) {
        return ((Math.pow(1 - t, 2) * s) + (2 * (1 - t) * t * c) + (Math.pow(t, 2) * e));
    }
    
    private double[] _getCubicBezierCurvePoint(double startX, double startY, double controlX1, double controlY1, double controlX2, double controlY2, double endX, double endY, double t) {
        double[] outputPoint = new double[2];
        
        if ((t >= 0) && (t <= 1)) {
            outputPoint[0] = this._getCubicBezierCurvePoint(startX, controlX1, controlX2, endX, t);
            outputPoint[1] = this._getCubicBezierCurvePoint(startY, controlY1, controlY2, endY, t);
        }
        
        return outputPoint;
    }
    
    private double _getCubicBezierCurvePoint(double s, double c1, double c2, double e, double t) {
        return ((Math.pow(1 - t, 3) * s) + (3 * Math.pow(1 - t, 2) * t * c1) + (3 * (1 - t) * Math.pow(t, 2) * c2) + (Math.pow(t, 3) * e));
    }
}
