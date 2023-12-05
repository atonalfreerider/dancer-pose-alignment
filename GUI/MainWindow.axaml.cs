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
    }

    static PreviewDrawer previewDrawer;
    internal static UpdatePreview updatePreview;

    public static void SetupPreview()
    {
        previewDrawer = new PreviewDrawer();
        updatePreview = SetPreview;
    }

    static int frameCount = 0;

    public delegate void UpdatePreview(Bitmap frame);

    static void SetPreview(Bitmap frame)
    {
        frameCount++;
        
        Dispatcher.UIThread.Post(() =>
        {
            DrawingImage drawingImage = previewDrawer.DrawGeometry();
            previewImage.Source = frame;
        }, DispatcherPriority.Render);

        Dispatcher.UIThread.RunJobs();
    }
}