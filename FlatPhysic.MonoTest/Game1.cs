namespace FlatPhysic.MonoTest;

using FlatPhysic.Bodies;
using FlatPhysic.Constraints;
using FlatPhysic.MonoDrawer;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using System;

public class Game1 : Game
{
    private readonly GraphicsDeviceManager graphics;

    private readonly PhysicScene physicScene;

    public Game1()
    {
        this.graphics = new GraphicsDeviceManager(this)
        { PreferredBackBufferWidth = 500, PreferredBackBufferHeight = 500 };
        this.IsMouseVisible = true;
        this.physicScene = new();
        //this.TargetElapsedTime = TimeSpan.FromSeconds(1d / 2);

        this.Window.KeyDown += (s, e) =>
        {
            switch (e.Key)
            {
                case Keys.W:
                    this.physicScene.AddBody(PolygonBody.CreateRegularPolygon(Mouse.GetState().Position.ToFlat(), new(40, 40), 6, 1));
                    //this.physicScene.AddBody(new CircleBody(Mouse.GetState().Position.ToFlat(), 20, 1));
                    break;
            }
        };

        this.physicScene.AddBody(PolygonBody.CreateBox(new(250, 450), new(510, 20)));

        var bs = new RigidBody[20];
        var size = 10f;

        for (int i = 0; i < bs.Length; i++)
        {
            var a = PolygonBody.CreateBox(new(100 + (size * i), 150), new(size, 5), 1f);
            this.physicScene.AddBody(a);
            bs[i] = a;
        }

        for (int i = 0; i < bs.Length; i++)
        {
            var a = bs[i];
            if (i == 0)
            {
                this.physicScene.AddConstraint(new WorldAttachment(a, new(-size / 2, 0)));
            }

            if (i == bs.Length - 1)
            {
                this.physicScene.AddConstraint(new WorldAttachment(a, new(size / 2, 0)));
            }
            else
            {
                var b = bs[i + 1];
                this.physicScene.AddConstraint(new BodyAttachment(a, new(size / 2, 0), b, new(-size / 2, 0)));
            }
        }

        this.Components.Add(new MonoDrawer(this, this.physicScene) { BoundingBox = true, CollisionPoint = true });
    }

    protected override void Update(GameTime gameTime)
    {
        this.physicScene.Update((float)gameTime.ElapsedGameTime.TotalSeconds);
        base.Update(gameTime);
    }

    protected override void Draw(GameTime gameTime)
    {
        this.GraphicsDevice.Clear(Color.Black);
        base.Draw(gameTime);
    }
}
