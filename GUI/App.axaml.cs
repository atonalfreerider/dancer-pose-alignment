using Avalonia;
using Avalonia.Markup.Xaml;

namespace GUI;

public class App : Application
{
    public override void Initialize()
    {
        AvaloniaXamlLoader.Load(this);
    }
}