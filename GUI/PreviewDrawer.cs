using System.Numerics;
using Avalonia.Media;

namespace GUI;

public class PreviewDrawer
{
    const int IMAGE_DIMENSION = 512;
    const int IMAGE_PADDING = 8;

    public DrawingImage DrawGeometry()
    {
        DrawingImage drawingImage = new DrawingImage();

        DrawingGroup drawingGroup = new DrawingGroup();

        DrawPackageLines(drawingGroup);

        drawingImage.Drawing = drawingGroup;

        return drawingImage;
    }


    static void DrawPackageLines(DrawingGroup drawingGroup)
    {
        Color penColor = Colors.LightSlateGray;

        GeometryDrawing lineGeometryDrawing = new GeometryDrawing
        {
            Pen = new Pen(new SolidColorBrush(penColor))
        };

        LineGeometry lineGeometry = new LineGeometry();
        
        lineGeometry.StartPoint = new Avalonia.Point(0, 0);
        lineGeometry.EndPoint = new Avalonia.Point(0, 100);

        lineGeometryDrawing.Geometry = lineGeometry;
        drawingGroup.Children.Add(lineGeometryDrawing);
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