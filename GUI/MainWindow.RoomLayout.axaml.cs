using System.Numerics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Shapes;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Media;
using Newtonsoft.Json;
using Path = System.IO.Path;

namespace GUI;

public partial class MainWindow
{
    readonly List<Vector3> cameraPositions = [];

    public MainWindow()
    {
        InitializeComponent();
    }

    void LayoutCanvas_MouseDown(object sender, PointerPressedEventArgs args)
    {
        // Draw the triangle representing the camera position
        PointerPoint point = args.GetCurrentPoint(sender as Control);

        // Create and add the triangle to the canvas
        Polygon triangle = CreateTriangle(point, .2f, LayoutCanvas.Width, LayoutCanvas.Height);
        LayoutCanvas.Children.Add(triangle);

        cameraPositions.Add(new Vector3((float)point.Position.X, float.Parse(HeightInputText.Text),
            (float)point.Position.Y));
    }

    static Polygon CreateTriangle(PointerPoint position, double focalLength, double canvasWidth, double canvasHeight)
    {
        Polygon triangle = new Polygon
        {
            Stroke = Brushes.Black,
            Fill = Brushes.Black,
            StrokeThickness = 2
        };

        // The points of the triangle will be determined based on the position and baseWidth
        const double baseWidth = 30;

        // Calculate the angle to rotate the triangle
        double angle = Math.Atan2(position.Position.Y - canvasHeight / 2, position.Position.X - canvasWidth / 2) -
                       Math.PI / 2;

        // Calculate the rotated points
        Point top = new Point(0, 0);
        Point bottomLeft = new Point(-baseWidth / 2, -focalLength * 100);
        Point bottomRight = new Point(baseWidth / 2, -focalLength * 100);

        triangle.Points.Add(top);
        triangle.Points.Add(RotatePoint(bottomLeft, angle));
        triangle.Points.Add(RotatePoint(bottomRight, angle));

        // Translate the triangle to the position
        Matrix matrix = Matrix.CreateTranslation(position.Position.X, position.Position.Y);
        triangle.RenderTransform = new MatrixTransform(matrix);

        return triangle;
    }

    static Point RotatePoint(Point point, double angle)
    {
        return new Point(
            point.X * Math.Cos(angle) - point.Y * Math.Sin(angle),
            point.X * Math.Sin(angle) + point.Y * Math.Cos(angle)
        );
    }

    void SaveLayoutButton_Click(object sender, RoutedEventArgs e)
    {
        string saveDirectory = VideoInputPath.Text;

        string cameraSavePath = Path.Combine(saveDirectory, $"camera-positions.json");

        File.WriteAllText(cameraSavePath,
            JsonConvert.SerializeObject(cameraPositions, Formatting.Indented));

        Console.WriteLine($"Saved to: {saveDirectory})");
    }
}