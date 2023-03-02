# FlatPhysic

2D physic rigid body simulation scene

# Features
- Static and rigid bodies
- Polygon bodies and more efficient circle bodies
- Collision detection and responce
- Angular momentum and realistic rotation response
- Friction between bodies and air
- Bounding box detection for efficiency
- Bodies freezing for efficiency
- Constraints:
    - Springs
    - Fixed point attachments
    - Body to body attachments
- OnCollision events
- Simple scene drawing libraries for SFML and MonoGame

# Quickstart
Here's sample code for Monogame framework

```c#
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

        // Create physic scene
        this.physicScene = new PhysicScene();

        // Create a static box
	    this.physicScene.AddBody(PolygonBody.CreateBox(new FlatVector(0, 400), new FlatVector(500, 20)));

	    // Create a circle rigid body
	    this.physicScene.AddBody(new CircleBody(new FlatVector(0, 0), 10, 1));

	    // Create a rigid box
	    this.physicScene.AddBody(PolygonBody.CreateBox(new FlatVector(0, 20), new FlatVector(10, 10), 5));

	    // Create a spring between the last two bodies
	    this.physicScene.AddConstraint(new Spring(physicScene.Bodies[^1], FlatVector.Zero, physicScene.Bodies[^2], FlatVector.Zero, 10, 20, 20));

	    // Create a collision event
	    this.physicScene.Bodies[0].OnCollision += (s, e) =>
        {
            if (e.Body is PolygonBody)
            {
                Console.WriteLine($"Polygon collision! Normal: {e.Normal}");
            }
        };

        // Create a Monogame drawer as a game component
        this.Components.Add(new MonoDrawer(this, this.physicScene));
    }

    protected override void Update(GameTime gameTime)
    {
        var keyboard = Keyboard.GetState();
        var mouse = Mouse.GetState();

        // Add a new circle body whem pressing the W key
        if (keyboard.IsKeyDown(Keys.W))
        {
            this.physicScene.AddBody(new CircleBody(mouse.Position.ToFlat(), 10, 1));
        }

        // Update physic scene
        this.physicScene.Update((float)gameTime.ElapsedGameTime.TotalSeconds);

        base.Update(gameTime);
    }

    protected override void Draw(GameTime gameTime)
    {
        this.GraphicsDevice.Clear(Color.Black);
        base.Draw(gameTime);
    }
}
```