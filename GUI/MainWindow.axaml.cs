using System.Numerics;
using Avalonia.Controls;
using Avalonia.Markup.Xaml;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using Image = Avalonia.Controls.Image;

namespace GUI;

public partial class MainWindow : Window
{
    static Image previewImage;
    static Image poseImage;

    public MainWindow()
    {
        AvaloniaXamlLoader.Load(this);

        TextBox videoInputPath = this.Find<TextBox>("VideoInputPath");
        TextBox alphaPoseJsonPath = this.Find<TextBox>("AlphaPoseJsonPath");

        Button nextFrameButton = this.Find<Button>("NextFrameButton");

        nextFrameButton.Click += delegate
        {
            Program.RenderFrame(new Program.Args
                { VideoPath = videoInputPath.Text, AlphaPoseJsonPath = alphaPoseJsonPath.Text });
        };

        previewImage = this.Find<Image>("PreviewImage");
        poseImage = this.Find<Image>("PoseImage");
    }
    
    internal static UpdatePreview updatePreview;

    public static void SetupPreview()
    {
        updatePreview = SetPreview;
    }

    public delegate void UpdatePreview(Bitmap frame, Dictionary<int, List<Vector3>> posesByPersonAtFrame);

    static void SetPreview(Bitmap frame, Dictionary<int, List<Vector3>> posesByPersonAtFrame)
    {
        Dispatcher.UIThread.Post(() =>
        {
            DrawingImage drawingImage = PreviewDrawer.DrawGeometry(posesByPersonAtFrame, frame.Size);

            previewImage.Source = frame;
            poseImage.Source = drawingImage;
        }, DispatcherPriority.Render);

        Dispatcher.UIThread.RunJobs();
    }
}