using Avalonia.Controls;
using Avalonia.Markup.Xaml;
using Avalonia.Media;
using Avalonia.Threading;
using Image = Avalonia.Controls.Image;

namespace GUI;

public partial class MainWindow : Window
{
    static Image previewImage;

    public MainWindow()
    {
        AvaloniaXamlLoader.Load(this);

        TextBox inputTextBox = this.Find<TextBox>("CodebaseInputPath");
        TextBox intervalBox = this.Find<TextBox>("Interval");

        Button layoutButton = this.Find<Button>("LayoutButton");
        
        layoutButton.Click += delegate
        {
            interval = Convert.ToInt32(intervalBox.Text);
            Program.Layout(new Program.Args {InputPath = inputTextBox.Text});
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
        
    static int intervalCount = 0;
    static int interval = 100;
        
    public delegate void UpdatePreview();

    static void SetPreview()
    {
        intervalCount++;
        if (intervalCount >= interval)
        {
            intervalCount = 0;
            Dispatcher.UIThread.Post(() =>
            {
                DrawingImage drawingImage = previewDrawer.DrawGeometry();
                previewImage.Source = drawingImage;
                    
            }, DispatcherPriority.Render);
                
            Dispatcher.UIThread.RunJobs();
        }
    }
}