namespace FlatPhysic.MonoTest;

using FlatPhysic.Bodies;
using FlatPhysic.MonoDrawer;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

public class Game1 : Game
{
    private readonly GraphicsDeviceManager graphics;

    private readonly PhysicScene physicScene;

    public Game1()
    {
        this.graphics = new GraphicsDeviceManager(this)
        { PreferredBackBufferWidth = 500, PreferredBackBufferHeight = 500 };
        this.IsMouseVisible = true;
        this.physicScene = new() { AllowFreeze = false };
        // this.TargetElapsedTime = TimeSpan.FromSeconds(1d / 30);

        this.physicScene.Bodies.Add(PolygonBody.CreateBox(new(250, 400), new(510, 20)));

        this.Components.Add(new MonoDrawer(this, this.physicScene));
    }

    protected override void Update(GameTime gameTime)
    {
        var keyboard = Keyboard.GetState();
        var mouse = Mouse.GetState();

        if (keyboard.IsKeyDown(Keys.W))
        {
            this.physicScene.Bodies.Add(new CircleBody(mouse.Position.ToFlat(), 10, 1));
        }

        this.physicScene.Update((float)gameTime.ElapsedGameTime.TotalSeconds);

        base.Update(gameTime);
    }

    protected override void Draw(GameTime gameTime)
    {
        this.GraphicsDevice.Clear(Color.Black);
        base.Draw(gameTime);
    }
}
