using System.Numerics;
using Avalonia;
using Avalonia.Media;
using dancer_pose_alignment;

namespace GUI;

public static class PreviewDrawer
{
    public static DrawingGroup DrawGeometry(
        List<List<Vector3>> poses,
        Size imgSize,
        int currentLeadIndex,
        int currentFollowIndex,
        PoseType poseType,
        List<Vector2> originCross,
        List<Vector2> leadProjectionsAtFrame,
        List<Vector2> followProjectionsAtFrame,
        List<Tuple<Vector2, Vector2>> cameraProjectionsAtFrame)
    {
        DrawingGroup drawingGroup = DrawPoses(
            poses,
            imgSize,
            currentLeadIndex,
            currentFollowIndex,
            poseType);

        drawingGroup = DrawOriginCross(drawingGroup, originCross);

        SolidColorBrush camBrush = new SolidColorBrush(Colors.DarkGreen);
        Pen camPen = new Pen(camBrush)
        {
            Thickness = 10
        };

        SolidColorBrush manualCamBrush = new SolidColorBrush(Colors.Green);
        Pen manualCamPen = new Pen(manualCamBrush)
        {
            Thickness = 10
        };

        foreach (Tuple<Vector2, Vector2> camPosAndManual in cameraProjectionsAtFrame)
        {
            GeometryDrawing camGeometry = DrawPoint(camPosAndManual.Item1, camPen);
            drawingGroup.Children.Add(camGeometry);

            if (camPosAndManual.Item2.X > 0 && camPosAndManual.Item2.Y > 0)
            {
                GeometryDrawing manualCamGeometry = DrawPoint(camPosAndManual.Item2, manualCamPen);
                drawingGroup.Children.Add(manualCamGeometry);

                GeometryDrawing line = DrawLine(camPosAndManual.Item1, camPosAndManual.Item2, manualCamPen);
                drawingGroup.Children.Add(line);
            }
        }

        if (leadProjectionsAtFrame.Count == 0 || followProjectionsAtFrame.Count == 0) return drawingGroup;

        SolidColorBrush brush = new SolidColorBrush(Colors.DarkRed);
        Pen pen = new Pen(brush)
        {
            Thickness = 4
        };
        foreach (Vector2 point in leadProjectionsAtFrame)
        {
            GeometryDrawing poseGeometry = DrawPoint(point, pen);
            drawingGroup.Children.Add(poseGeometry);
        }

        drawingGroup = poseType switch
        {
            PoseType.Coco => DrawCoco(drawingGroup,
                leadProjectionsAtFrame.Select(x => new Vector3(x.X, x.Y, 1f)).ToList(), -1),
            PoseType.Halpe => DrawHalpe(drawingGroup,
                leadProjectionsAtFrame.Select(x => new Vector3(x.X, x.Y, 1f)).ToList(), -1),
            _ => throw new ArgumentOutOfRangeException(nameof(poseType), poseType, null)
        };

        SolidColorBrush followBrush = new SolidColorBrush(Colors.DarkMagenta);
        Pen followPen = new Pen(followBrush)
        {
            Thickness = 4
        };
        foreach (Vector2 point in followProjectionsAtFrame)
        {
            GeometryDrawing poseGeometry = DrawPoint(point, followPen);
            drawingGroup.Children.Add(poseGeometry);
        }

        drawingGroup = poseType switch
        {
            PoseType.Coco => DrawCoco(drawingGroup,
                followProjectionsAtFrame.Select(x => new Vector3(x.X, x.Y, 1f)).ToList(), -1),
            PoseType.Halpe => DrawHalpe(drawingGroup,
                followProjectionsAtFrame.Select(x => new Vector3(x.X, x.Y, 1f)).ToList(), -1),
            _ => throw new ArgumentOutOfRangeException(nameof(poseType), poseType, null)
        };


        return drawingGroup;
    }

    static DrawingGroup DrawPoses(
        List<List<Vector3>> poses,
        Size imgSize,
        int currentLeadIndex,
        int currentFollowIndex,
        PoseType poseType)
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

        int poseCount = 0;
        foreach (List<Vector3> pose in poses)
        {
            int role = -1;
            if (currentLeadIndex > -1 && poseCount == currentLeadIndex)
            {
                role = 0;
            }
            else if (currentFollowIndex > -1 && poseCount == currentFollowIndex)
            {
                role = 1;
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

            drawingGroup = poseType switch
            {
                PoseType.Coco => DrawCoco(drawingGroup, pose, role),
                PoseType.Halpe => DrawHalpe(drawingGroup, pose, role)
            };

            poseCount++;
        }

        return drawingGroup;
    }

    static DrawingGroup DrawCoco(DrawingGroup drawingGroup, List<Vector3> pose, int role)
    {
        GeometryDrawing leftCalf = DrawLine(pose[(int)CocoJoint.L_Ankle], pose[(int)CocoJoint.L_Knee], role);
        drawingGroup.Children.Add(leftCalf);

        GeometryDrawing leftThigh = DrawLine(pose[(int)CocoJoint.L_Knee], pose[(int)CocoJoint.L_Hip], role);
        drawingGroup.Children.Add(leftThigh);

        GeometryDrawing rightCalf = DrawLine(pose[(int)CocoJoint.R_Ankle], pose[(int)CocoJoint.R_Knee], role);
        drawingGroup.Children.Add(rightCalf);

        GeometryDrawing rightThigh = DrawLine(pose[(int)CocoJoint.R_Knee], pose[(int)CocoJoint.R_Hip], role);
        drawingGroup.Children.Add(rightThigh);

        GeometryDrawing leftUpperArm = DrawLine(pose[(int)CocoJoint.L_Shoulder], pose[(int)CocoJoint.L_Elbow], role);
        drawingGroup.Children.Add(leftUpperArm);

        GeometryDrawing leftForearm = DrawLine(pose[(int)CocoJoint.L_Elbow], pose[(int)CocoJoint.L_Wrist], role);
        drawingGroup.Children.Add(leftForearm);

        GeometryDrawing rightUpperArm = DrawLine(pose[(int)CocoJoint.R_Shoulder], pose[(int)CocoJoint.R_Elbow], role);
        drawingGroup.Children.Add(rightUpperArm);

        GeometryDrawing rightForearm = DrawLine(pose[(int)CocoJoint.R_Elbow], pose[(int)CocoJoint.R_Wrist], role);
        drawingGroup.Children.Add(rightForearm);

        GeometryDrawing hip = DrawLine(pose[(int)CocoJoint.L_Hip], pose[(int)CocoJoint.R_Hip], role);
        drawingGroup.Children.Add(hip);

        GeometryDrawing shoulders = DrawLine(pose[(int)CocoJoint.L_Shoulder], pose[(int)CocoJoint.R_Shoulder], role);
        drawingGroup.Children.Add(shoulders);

        return drawingGroup;
    }

    static DrawingGroup DrawHalpe(DrawingGroup drawingGroup, List<Vector3> pose, int role)
    {
        GeometryDrawing leftCalf = DrawLine(pose[(int)HalpeJoint.LAnkle], pose[(int)HalpeJoint.LKnee], role);
        drawingGroup.Children.Add(leftCalf);

        GeometryDrawing leftThigh = DrawLine(pose[(int)HalpeJoint.LKnee], pose[(int)HalpeJoint.LHip], role);
        drawingGroup.Children.Add(leftThigh);

        GeometryDrawing rightCalf = DrawLine(pose[(int)HalpeJoint.RAnkle], pose[(int)HalpeJoint.RKnee], role);
        drawingGroup.Children.Add(rightCalf);

        GeometryDrawing rightThigh = DrawLine(pose[(int)HalpeJoint.RKnee], pose[(int)HalpeJoint.RHip], role);
        drawingGroup.Children.Add(rightThigh);

        GeometryDrawing leftUpperArm = DrawLine(pose[(int)HalpeJoint.LShoulder], pose[(int)HalpeJoint.LElbow], role);
        drawingGroup.Children.Add(leftUpperArm);

        GeometryDrawing leftForearm = DrawLine(pose[(int)HalpeJoint.LElbow], pose[(int)HalpeJoint.LWrist], role);
        drawingGroup.Children.Add(leftForearm);

        GeometryDrawing rightUpperArm = DrawLine(pose[(int)HalpeJoint.RShoulder], pose[(int)HalpeJoint.RElbow], role);
        drawingGroup.Children.Add(rightUpperArm);

        GeometryDrawing rightForearm = DrawLine(pose[(int)HalpeJoint.RElbow], pose[(int)HalpeJoint.RWrist], role);
        drawingGroup.Children.Add(rightForearm);

        GeometryDrawing spine = DrawLine(pose[(int)HalpeJoint.Neck], pose[(int)HalpeJoint.Hip], role);
        drawingGroup.Children.Add(spine);

        GeometryDrawing neck = DrawLine(pose[(int)HalpeJoint.Head], pose[(int)HalpeJoint.Neck], role);
        drawingGroup.Children.Add(neck);

        GeometryDrawing lHip = DrawLine(pose[(int)HalpeJoint.Hip], pose[(int)HalpeJoint.LHip], role);
        drawingGroup.Children.Add(lHip);

        GeometryDrawing rHip = DrawLine(pose[(int)HalpeJoint.Hip], pose[(int)HalpeJoint.RHip], role);
        drawingGroup.Children.Add(rHip);

        GeometryDrawing lShoulder = DrawLine(pose[(int)HalpeJoint.Neck], pose[(int)HalpeJoint.LShoulder], role);
        drawingGroup.Children.Add(lShoulder);

        GeometryDrawing rShoulder = DrawLine(pose[(int)HalpeJoint.Neck], pose[(int)HalpeJoint.RShoulder], role);
        drawingGroup.Children.Add(rShoulder);

        return drawingGroup;
    }

    static DrawingGroup DrawOriginCross(DrawingGroup drawingGroup, List<Vector2> originCross)
    {
        if (originCross.Count == 0) return drawingGroup;

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

    static GeometryDrawing DrawLine(Vector2 start, Vector2 end, Pen linePen)
    {
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
            Center = new Point(point.X, point.Y),
            RadiusX = .2,
            RadiusY = .2
        };

        poseGeometry.Geometry = ellipseGeometry;

        return poseGeometry;
    }
}