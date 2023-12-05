using System.Numerics;
using Avalonia;
using Avalonia.Media;
using dancer_pose_alignment;

namespace GUI;

public static class PreviewDrawer
{
    public static DrawingImage DrawGeometry(Dictionary<int, List<Vector3>> posesByPersonAtFrame, Size imgSize)
    {
        DrawingImage drawingImage = new DrawingImage();
        DrawingGroup drawingGroup = new DrawingGroup();

        Color penColor = Colors.Red;
        Pen pen = new Pen(new SolidColorBrush(penColor))
        {
            Thickness = 1
        };
        
        // HACK: add small dots to the corners of the image to force the overlay to not resize
        GeometryDrawing lineGeometryDrawingTopCorner = DrawPoint(Vector2.Zero, pen);
        drawingGroup.Children.Add(lineGeometryDrawingTopCorner);
        
        GeometryDrawing lineGeometryDrawingBottomCorner = DrawPoint(new Vector2((float) imgSize.Width,(float) imgSize.Height), pen);
        drawingGroup.Children.Add(lineGeometryDrawingBottomCorner);

        foreach ((int personIdx, List<Vector3> pose) in posesByPersonAtFrame)
        {
            foreach (Vector3 joint in pose)
            {
                GeometryDrawing poseGeometry = DrawPoint(new Vector2(joint.X, joint.Y), pen);
                drawingGroup.Children.Add(poseGeometry);
            }
            
            GeometryDrawing leftCalf = DrawLine(pose[(int)Halpe.LAnkle], pose[(int)Halpe.LKnee], pen);
            drawingGroup.Children.Add(leftCalf);
            
            GeometryDrawing leftThigh = DrawLine(pose[(int)Halpe.LKnee], pose[(int)Halpe.LHip], pen);
            drawingGroup.Children.Add(leftThigh);
            
            GeometryDrawing rightCalf = DrawLine(pose[(int)Halpe.RAnkle], pose[(int)Halpe.RKnee], pen);
            drawingGroup.Children.Add(rightCalf);
            
            GeometryDrawing rightThigh = DrawLine(pose[(int)Halpe.RKnee], pose[(int)Halpe.RHip], pen);
            drawingGroup.Children.Add(rightThigh);
            
            GeometryDrawing leftUpperArm = DrawLine(pose[(int)Halpe.LShoulder], pose[(int)Halpe.LElbow], pen);
            drawingGroup.Children.Add(leftUpperArm);
            
            GeometryDrawing leftForearm = DrawLine(pose[(int)Halpe.LElbow], pose[(int)Halpe.LWrist], pen);
            drawingGroup.Children.Add(leftForearm);
            
            GeometryDrawing rightUpperArm = DrawLine(pose[(int)Halpe.RShoulder], pose[(int)Halpe.RElbow], pen);
            drawingGroup.Children.Add(rightUpperArm);
            
            GeometryDrawing rightForearm = DrawLine(pose[(int)Halpe.RElbow], pose[(int)Halpe.RWrist], pen);
            drawingGroup.Children.Add(rightForearm);
            
            GeometryDrawing spine = DrawLine(pose[(int)Halpe.Neck], pose[(int)Halpe.Hip], pen);
            drawingGroup.Children.Add(spine);
            
            GeometryDrawing neck = DrawLine(pose[(int)Halpe.Head], pose[(int)Halpe.Neck], pen);
            drawingGroup.Children.Add(neck);
            
            GeometryDrawing lHip = DrawLine(pose[(int)Halpe.Hip], pose[(int)Halpe.LHip], pen);
            drawingGroup.Children.Add(lHip);
            
            GeometryDrawing rHip = DrawLine(pose[(int)Halpe.Hip], pose[(int)Halpe.RHip], pen);
            drawingGroup.Children.Add(rHip);
            
            GeometryDrawing lShoulder = DrawLine(pose[(int)Halpe.Neck], pose[(int)Halpe.LShoulder], pen);
            drawingGroup.Children.Add(lShoulder);
            
            GeometryDrawing rShoulder = DrawLine(pose[(int)Halpe.Neck], pose[(int)Halpe.RShoulder], pen);
            drawingGroup.Children.Add(rShoulder);
        }


        drawingImage.Drawing = drawingGroup;
        return drawingImage;
    }
    
    static GeometryDrawing DrawLine(Vector3 start, Vector3 end, Pen pen)
    {
        GeometryDrawing poseGeometry = new GeometryDrawing
        {
            Pen = pen
        };

        LineGeometry lineGeometry = new LineGeometry
        {
            StartPoint = new Avalonia.Point(start.X, start.Y),
            EndPoint = new Avalonia.Point(end.X, end.Y)
            
        };
        poseGeometry.Geometry = lineGeometry;

        return poseGeometry;
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