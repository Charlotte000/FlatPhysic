namespace FlatPhysic.SFMLTest;

using FlatPhysic.Bodies;
using FlatPhysic.SFMLDrawer;
using SFML.Graphics;
using SFML.Window;

public static class Program
{
    public static void Main()
    {
        var window = new RenderWindow(new VideoMode(500, 500), "SFML Test");
        window.SetFramerateLimit(60);
        window.Closed += (_, _) => window.Close();

        var physicScene = new PhysicScene();
        physicScene.Bodies.Add(new InverseCircleBody(new(250, 250), 100));

        var drawer = new SFMLDrawer(physicScene);

        while (window.IsOpen)
        {
            window.DispatchEvents();

            if (Keyboard.IsKeyPressed(Keyboard.Key.W))
            {
                physicScene.Bodies.Add(new CircleBody(Mouse.GetPosition(window).ToFlat(), 10, 1));
            }

            physicScene.Update(.016f);

            window.Clear(Color.Black);
            drawer.Draw(window, RenderStates.Default);

            window.Display();
        }
    }
}