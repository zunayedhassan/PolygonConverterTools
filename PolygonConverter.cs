using System;
using System.Collections.Generic;

namespace com.appiomatic.PolygonConverter
{
    class PolygonConverter
    {
        public List<double> GetRectanglePolygon(double x, double y, double width, double height)
        {
            List<double> points = new List<double>();

            points.Add(x);
            points.Add(y);

            points.Add(x + width);
            points.Add(y);

            points.Add(x + width);
            points.Add(y + height);

            points.Add(x);
            points.Add(y + height);

            return points;
        }

        public List<double> GetRoundedRectanglePolygon(double x, double y, double width, double height, double arcWidth, double arcHeight)
        {
            List<double> points = new List<double>();

            arcWidth /= 2.0;
            arcHeight /= 2.0;

            points.Add(x + arcWidth);
            points.Add(y);

            this._setRoundedCorner(points, arcWidth, arcHeight, x + width - arcWidth, y + arcHeight, 90, 180);

            points.Add(x + width);
            points.Add(y + height - arcHeight);

            this._setRoundedCorner(points, arcWidth, arcHeight, x + width - arcWidth, y + height - arcHeight, 180, 270);


            points.Add(x + arcWidth);
            points.Add(y + height);

            this._setRoundedCorner(points, arcWidth, arcHeight, x + arcWidth, y + height - arcHeight, 270, 360);

            points.Add(x);
            points.Add(y + arcHeight);

            this._setRoundedCorner(points, arcWidth, arcHeight, x + arcWidth, y + arcHeight, 0, 90);

            return points;
        }

        private void _setRoundedCorner(List<double> points, double radiusX, double radiusY, double centerX, double centerY, double minAngle, double maxAngle)
        {
            List<double> roundedCornerPoints = this.GetEllipsePolygon(radiusX, radiusY, centerX, centerY, minAngle, maxAngle);

            foreach (double value in roundedCornerPoints)
            {
                points.Add(value);
            }
        }

        public List<double> GetEllipsePolygon(double radiusX, double radiusY, double centerX, double centerY, double minAngle, double maxAngle)
        {
            minAngle -= 180;
            maxAngle -= 180;

            List<double> points = new List<double>();

            const double TOTAL_ANGLE_OF_CIRCLE = 360;
            const int TOTAL_VERTICES = 36;

            double[] unsortedVerticesX = new double[TOTAL_VERTICES];
            double[] unsortedVerticesY = new double[TOTAL_VERTICES];
            double[] angles = new double[TOTAL_VERTICES];
            int[] verticesIndex = new int[TOTAL_VERTICES];

            for (int i = 0, j = 0; i < TOTAL_ANGLE_OF_CIRCLE; i += (int) (TOTAL_ANGLE_OF_CIRCLE / TOTAL_VERTICES), j++)
            {
                double x = centerX + radiusX * Math.Cos(i);
                double y = centerY + radiusY * Math.Sin(i);

                unsortedVerticesX[j] = x;
                unsortedVerticesY[j] = y;

                double anlgeFromCenter = Math.Atan2((y - centerY), (x - centerX));
                angles[j] = anlgeFromCenter;
            }

            for (int i = 0; i < verticesIndex.Length; i++)
            {
                verticesIndex[i] = i;
            }

            // Now sort
            for (int i = 0; i < angles.Length; i++)
            {
                for (int j = 0; j < angles.Length; j++)
                {
                    if (angles[i] < angles[j])
                    {
                        double angleTemp = angles[i];
                        angles[i] = angles[j];
                        angles[j] = angleTemp;

                        int indexTemp = verticesIndex[i];
                        verticesIndex[i] = verticesIndex[j];
                        verticesIndex[j] = indexTemp;
                    }
                }
            }

            for (int i = 0; i < verticesIndex.Length; i++)
            {
                double currentAngle = (angles[i] * 57.2958);

                if ((currentAngle >= minAngle) && (currentAngle <= maxAngle))
                {
                    double x = unsortedVerticesX[verticesIndex[i]];
                    double y = unsortedVerticesY[verticesIndex[i]];

                    points.Add(x);
                    points.Add(y);
                }
            }

            return points;
        }

        public List<double> GetCirclePolygon(double radius, double centerX, double centerY)
        {
            List<double> points = new List<double>();

            const double TOTAL_ANGLE_OF_CIRCLE = 360;
            const int TOTAL_VERTICES = 36;

            double[] unsortedVerticesX = new double[TOTAL_VERTICES];
            double[] unsortedVerticesY = new double[TOTAL_VERTICES];
            double[] angles = new double[TOTAL_VERTICES];
            int[] verticesIndex = new int[TOTAL_VERTICES];

            for (int i = 0, j = 0; i < TOTAL_ANGLE_OF_CIRCLE; i += (int) (TOTAL_ANGLE_OF_CIRCLE / TOTAL_VERTICES), j++)
            {
                double x = centerX + radius * Math.Cos(i);
                double y = centerY + radius * Math.Sin(i);

                unsortedVerticesX[j] = x;
                unsortedVerticesY[j] = y;

                double anlgeFromCenter = Math.Atan2((y - centerY), (x - centerX));
                angles[j] = anlgeFromCenter;
            }

            for (int i = 0; i < verticesIndex.Length; i++)
            {
                verticesIndex[i] = i;
            }

            // Now sort
            for (int i = 0; i < angles.Length; i++)
            {
                for (int j = 0; j < angles.Length; j++)
                {
                    if (angles[i] > angles[j])
                    {
                        double angleTemp = angles[i];
                        angles[i] = angles[j];
                        angles[j] = angleTemp;

                        int indexTemp = verticesIndex[i];
                        verticesIndex[i] = verticesIndex[j];
                        verticesIndex[j] = indexTemp;
                    }
                }
            }

            for (int i = 0; i < verticesIndex.Length; i++)
            {
                double x = unsortedVerticesX[verticesIndex[i]];
                double y = unsortedVerticesY[verticesIndex[i]];

                points.Add(x);
                points.Add(y);
            }

            return points;
        }

        public double GetDistanceBetweenTwoPoints(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2));
        }

        public int GetNearestPointIndex(double[] verticesX, double[] verticesY, double testPointX, double testPointY)
        {
            int index = 0;
            double[] distances = new double[verticesX.Length];

            for (int i = 0; i < verticesX.Length; i++)
            {
                distances[i] = this.GetDistanceBetweenTwoPoints(verticesX[i], verticesY[i], testPointX, testPointY);
            }

            double min = distances[index];

            for (int i = 0; i < distances.Length; i++)
            {
                if (min > distances[i])
                {
                    min = distances[i];
                    index = i;
                }
            }

            return index;
        }

        public double[] GetQuadraticBezierCurvePoint(double startX, double startY, double controlX, double controlY, double endX, double endY, double t)
        {
            double[] outputPoint = new double[2];

            if ((t >= 0) && (t <= 1))
            {
                outputPoint[0] = this._getQuadraticBezierCurvePoint(startX, controlX, endX, t);
                outputPoint[1] = this._getQuadraticBezierCurvePoint(startY, controlY, endY, t);
            }

            return outputPoint;
        }

        private double _getQuadraticBezierCurvePoint(double s, double c, double e, double t)
        {
            return ((Math.Pow(1 - t, 2) * s) + (2 * (1 - t) * t * c) + (Math.Pow(t, 2) * e));
        }

        public double[] GetCubicBezierCurvePoint(double startX, double startY, double controlX1, double controlY1, double controlX2, double controlY2, double endX, double endY, double t)
        {
            double[] outputPoint = new double[2];

            if ((t >= 0) && (t <= 1))
            {
                outputPoint[0] = this._getCubicBezierCurvePoint(startX, controlX1, controlX2, endX, t);
                outputPoint[1] = this._getCubicBezierCurvePoint(startY, controlY1, controlY2, endY, t);
            }

            return outputPoint;
        }

        private double _getCubicBezierCurvePoint(double s, double c1, double c2, double e, double t)
        {
            return ((Math.Pow(1 - t, 3) * s) + (3 * Math.Pow(1 - t, 2) * t * c1) + (3 * (1 - t) * Math.Pow(t, 2) * c2) + (Math.Pow(t, 3) * e));
        }
    }
}
