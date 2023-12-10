using System.Numerics;
using Avalonia;
using Avalonia.Media;
using dancer_pose_alignment;

namespace GUI;

public static class PreviewDrawer
{
    public static DrawingGroup DrawGeometry(
        Dictionary<int, List<Vector3>> posesByPersonAtFrame,
        Size imgSize,
        int currentLeadIndex,
        int currentFollowIndex,
        int mirrorCurrentLeadIndex,
        int mirrorCurrentFollowIndex,
        List<Tuple<int, bool>> currentSelectedCamerasAndPoseAnchor,
        List<Tuple<int, bool>> mirrorCurrentSelectedCamerasAndPoseAnchor)
    {
        DrawingGroup drawingGroup = new DrawingGroup();

        // HACK: add small dots to the corners of the image to force the overlay to not resize
        SolidColorBrush dotBrush = new SolidColorBrush(Colors.Gray);
        Pen dotPen = new Pen(dotBrush)
        {
            Thickness = .1
        };
        GeometryDrawing lineGeometryDrawingTopCorner = DrawPoint(Vector2.Zero, dotPen);
        drawingGroup.Children.Add(lineGeometryDrawingTopCorner);

        GeometryDrawing lineGeometryDrawingBottomCorner =
            DrawPoint(new Vector2((float)imgSize.Width, (float)imgSize.Height), dotPen);
        drawingGroup.Children.Add(lineGeometryDrawingBottomCorner);

        foreach ((int personIdx, List<Vector3> pose) in posesByPersonAtFrame)
        {
            int role = -1;
            if (currentLeadIndex > -1 && personIdx == currentLeadIndex)
            {
                role = 0;
            }
            else if (currentFollowIndex > -1 && personIdx == currentFollowIndex)
            {
                role = 1;
            }
            else if (mirrorCurrentLeadIndex > -1 && personIdx == mirrorCurrentLeadIndex)
            {
                role = 2;
            }
            else if (mirrorCurrentFollowIndex > -1 && personIdx == mirrorCurrentFollowIndex)
            {
                role = 3;
            }
            else
            {
                foreach (Tuple<int, bool> camAndHand in currentSelectedCamerasAndPoseAnchor)
                {
                    if (personIdx == camAndHand.Item1)
                    {
                        role = 4;
                    }
                }

                foreach (Tuple<int, bool> camAndHand in mirrorCurrentSelectedCamerasAndPoseAnchor)
                {
                    if (personIdx == camAndHand.Item1)
                    {
                        role = 4;
                    }
                }
            }

            foreach (Vector3 joint in pose)
            {
                SolidColorBrush brush = new SolidColorBrush(ColorForConfidence(joint.Z, role));
                Pen pen = new Pen(brush)
                {
                    Thickness = 4
                };
                GeometryDrawing poseGeometry = DrawPoint(new Vector2(joint.X, joint.Y), pen);
                drawingGroup.Children.Add(poseGeometry);
            }

            if (pose.Count == 0)
            {
                continue;
            }

            drawingGroup = DrawHalpe(drawingGroup, pose, role);
        }

        return drawingGroup;
    }

    public static DrawingGroup DrawGeometry(
        Dictionary<int, List<Vector3>> posesByPersonAtFrame,
        Size imgSize,
        int currentLeadIndex,
        int currentFollowIndex,
        int mirrorCurrentLeadIndex,
        int mirrorCurrentFollowIndex,
        List<Vector3> currentSelectedCamerasAndPoseAnchor,
        List<Vector3> mirrorCurrentSelectedCamerasAndPoseAnchor,
        List<Vector2> originCross,
        List<Vector2> otherCameras,
        List<Vector2> leadProjectionsAtFrame,
        List<Vector2> followProjectionsAtFrame)
    {
        DrawingGroup drawingGroup = DrawGeometry(
            posesByPersonAtFrame,
            imgSize,
            currentLeadIndex,
            currentFollowIndex,
            mirrorCurrentLeadIndex,
            mirrorCurrentFollowIndex,
            [],
            []);

        SolidColorBrush originBrush = new SolidColorBrush(Colors.Black);
        Pen originPen = new Pen(originBrush)
        {
            Thickness = 10
        };
        GeometryDrawing originGeometry = DrawPoint(originCross[0], originPen);
        drawingGroup.Children.Add(originGeometry);

        // draw red line to positive xuint
        SolidColorBrush xBrush = new SolidColorBrush(Colors.Red);
        Pen xPen = new Pen(xBrush)
        {
            Thickness = 10
        };
        LineGeometry xLine = new LineGeometry
        {
            StartPoint = new Point(originCross[0].X, originCross[0].Y),
            EndPoint = new Point(originCross[1].X, originCross[1].Y)
        };
        GeometryDrawing xGeometry = new GeometryDrawing
        {
            Pen = xPen,
            Geometry = xLine
        };
        drawingGroup.Children.Add(xGeometry);

        // draw green line to positive y
        SolidColorBrush yBrush = new SolidColorBrush(Colors.Green);
        Pen yPen = new Pen(yBrush)
        {
            Thickness = 10
        };
        LineGeometry yLine = new LineGeometry
        {
            StartPoint = new Point(originCross[0].X, originCross[0].Y),
            EndPoint = new Point(originCross[2].X, originCross[2].Y)
        };
        GeometryDrawing yGeometry = new GeometryDrawing
        {
            Pen = yPen,
            Geometry = yLine
        };
        drawingGroup.Children.Add(yGeometry);

        // draw blue line to positive z
        SolidColorBrush zBrush = new SolidColorBrush(Colors.Blue);
        Pen zPen = new Pen(zBrush)
        {
            Thickness = 10
        };
        LineGeometry zLine = new LineGeometry
        {
            StartPoint = new Point(originCross[0].X, originCross[0].Y),
            EndPoint = new Point(originCross[3].X, originCross[3].Y)
        };
        GeometryDrawing zGeometry = new GeometryDrawing
        {
            Pen = zPen,
            Geometry = zLine
        };
        drawingGroup.Children.Add(zGeometry);

        SolidColorBrush camBrush = new SolidColorBrush(Colors.Green);
        Pen camPen = new Pen(camBrush)
        {
            Thickness = 10
        };
        foreach (Vector2 otherCamera in otherCameras)
        {
            GeometryDrawing poseGeometry = DrawPoint(otherCamera, camPen);
            drawingGroup.Children.Add(poseGeometry);
        }

        SolidColorBrush brush = new SolidColorBrush(Colors.Black);
        Pen pen = new Pen(brush)
        {
            Thickness = 4
        };
        foreach (Vector2 point in leadProjectionsAtFrame)
        {
            GeometryDrawing poseGeometry = DrawPoint(point, pen);
            drawingGroup.Children.Add(poseGeometry);
        }

        drawingGroup = DrawHalpe(drawingGroup, leadProjectionsAtFrame.Select(x => new Vector3(x.X, x.Y, 1f)).ToList(),
            -1);

        SolidColorBrush followBrush = new SolidColorBrush(Colors.Gray);
        Pen followPen = new Pen(followBrush)
        {
            Thickness = 4
        };
        foreach (Vector2 point in followProjectionsAtFrame)
        {
            GeometryDrawing poseGeometry = DrawPoint(point, followPen);
            drawingGroup.Children.Add(poseGeometry);
        }

        drawingGroup = DrawHalpe(drawingGroup, followProjectionsAtFrame.Select(x => new Vector3(x.X, x.Y, 1f)).ToList(),
            -1);

        SolidColorBrush lightCamBrush = new SolidColorBrush(Colors.LightGreen);
        Pen lightCamPen = new Pen(lightCamBrush)
        {
            Thickness = 10
        };

        foreach (Vector3 otherCam in currentSelectedCamerasAndPoseAnchor)
        {
            GeometryDrawing poseGeometry = DrawPoint(new Vector2(otherCam.X, otherCam.Y), lightCamPen);
            drawingGroup.Children.Add(poseGeometry);
        }

        SolidColorBrush lightMirrorCamBrush = new SolidColorBrush(Colors.LightGray);
        Pen lightMirrorCamPen = new Pen(lightMirrorCamBrush)
        {
            Thickness = 10
        };

        foreach (Vector3 otherCam in mirrorCurrentSelectedCamerasAndPoseAnchor)
        {
            GeometryDrawing poseGeometry = DrawPoint(new Vector2(otherCam.X, otherCam.Y), lightMirrorCamPen);
            drawingGroup.Children.Add(poseGeometry);
        }


        return drawingGroup;
    }

    static DrawingGroup DrawHalpe(DrawingGroup drawingGroup, List<Vector3> pose, int role)
    {
        GeometryDrawing leftCalf = DrawLine(pose[(int)Halpe.LAnkle], pose[(int)Halpe.LKnee], role);
        drawingGroup.Children.Add(leftCalf);

        GeometryDrawing leftThigh = DrawLine(pose[(int)Halpe.LKnee], pose[(int)Halpe.LHip], role);
        drawingGroup.Children.Add(leftThigh);

        GeometryDrawing rightCalf = DrawLine(pose[(int)Halpe.RAnkle], pose[(int)Halpe.RKnee], role);
        drawingGroup.Children.Add(rightCalf);

        GeometryDrawing rightThigh = DrawLine(pose[(int)Halpe.RKnee], pose[(int)Halpe.RHip], role);
        drawingGroup.Children.Add(rightThigh);

        GeometryDrawing leftUpperArm = DrawLine(pose[(int)Halpe.LShoulder], pose[(int)Halpe.LElbow], role);
        drawingGroup.Children.Add(leftUpperArm);

        GeometryDrawing leftForearm = DrawLine(pose[(int)Halpe.LElbow], pose[(int)Halpe.LWrist], role);
        drawingGroup.Children.Add(leftForearm);

        GeometryDrawing rightUpperArm = DrawLine(pose[(int)Halpe.RShoulder], pose[(int)Halpe.RElbow], role);
        drawingGroup.Children.Add(rightUpperArm);

        GeometryDrawing rightForearm = DrawLine(pose[(int)Halpe.RElbow], pose[(int)Halpe.RWrist], role);
        drawingGroup.Children.Add(rightForearm);

        GeometryDrawing spine = DrawLine(pose[(int)Halpe.Neck], pose[(int)Halpe.Hip], role);
        drawingGroup.Children.Add(spine);

        GeometryDrawing neck = DrawLine(pose[(int)Halpe.Head], pose[(int)Halpe.Neck], role);
        drawingGroup.Children.Add(neck);

        GeometryDrawing lHip = DrawLine(pose[(int)Halpe.Hip], pose[(int)Halpe.LHip], role);
        drawingGroup.Children.Add(lHip);

        GeometryDrawing rHip = DrawLine(pose[(int)Halpe.Hip], pose[(int)Halpe.RHip], role);
        drawingGroup.Children.Add(rHip);

        GeometryDrawing lShoulder = DrawLine(pose[(int)Halpe.Neck], pose[(int)Halpe.LShoulder], role);
        drawingGroup.Children.Add(lShoulder);

        GeometryDrawing rShoulder = DrawLine(pose[(int)Halpe.Neck], pose[(int)Halpe.RShoulder], role);
        drawingGroup.Children.Add(rShoulder);

        return drawingGroup;
    }

    static GeometryDrawing DrawLine(Vector3 start, Vector3 end, int role)
    {
        LinearGradientBrush linearGradientBrush = new LinearGradientBrush
        {
            StartPoint = new RelativePoint(new Point(start.X, start.Y), RelativeUnit.Absolute),
            EndPoint = new RelativePoint(new Point(end.X, end.Y), RelativeUnit.Absolute)
        };
        linearGradientBrush.GradientStops.Add(new GradientStop(ColorForConfidence(start.Z, role), 0));
        linearGradientBrush.GradientStops.Add(new GradientStop(ColorForConfidence(end.Z, role), 1));
        Pen linePen = new Pen(linearGradientBrush)
        {
            Thickness = 4
        };
        GeometryDrawing poseGeometry = new GeometryDrawing
        {
            Pen = linePen
        };

        LineGeometry lineGeometry = new LineGeometry
        {
            StartPoint = new Point(start.X, start.Y),
            EndPoint = new Point(end.X, end.Y)
        };
        poseGeometry.Geometry = lineGeometry;

        return poseGeometry;
    }

    static Color ColorForConfidence(float confidence, int role)
    {
        byte Avalue = (byte)(255 * confidence);
        byte Rvalue = 0;
        byte Gvalue = 0;
        byte Bvalue = 0;
        switch (role)
        {
            case -1:
                Rvalue = 100;
                Gvalue = 100;
                Bvalue = 100;
                break;
            case 0:
                Rvalue = 255;
                Gvalue = 0;
                Bvalue = 0;
                break;
            case 1:
                Rvalue = 255;
                Gvalue = 0;
                Bvalue = 255;
                break;
            case 2:
                Rvalue = 255;
                Gvalue = 100;
                Bvalue = 100;
                break;
            case 3:
                Rvalue = 255;
                Gvalue = 100;
                Bvalue = 255;
                break;
            case 4:
                Rvalue = 0;
                Gvalue = 255;
                Bvalue = 0;
                break;
        }

        return new Color(Avalue, Rvalue, Gvalue, Bvalue);
    }

    static GeometryDrawing DrawPoint(Vector2 point, Pen pen)
    {
        GeometryDrawing poseGeometry = new GeometryDrawing
        {
            Pen = pen
        };

        EllipseGeometry ellipseGeometry = new EllipseGeometry
        {
            Center = new Avalonia.Point(point.X, point.Y),
            RadiusX = .2,
            RadiusY = .2
        };

        poseGeometry.Geometry = ellipseGeometry;

        return poseGeometry;
    }

    public class IntVector2
    {
        public readonly int x;
        public readonly int y;

        public IntVector2(int passX, int passY)
        {
            x = passX;
            y = passY;
        }
    }
}