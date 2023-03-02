namespace FlatPhysic.MonoDrawer;

using FlatPhysic;
using FlatPhysic.Bodies;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

public class MonoDrawer : DrawableGameComponent
{
    private Vector2 size;

    private BasicEffect? effect;

    private GraphicsDevice? graphicsDevice;

    private readonly PhysicScene physicScene;

    public MonoDrawer(Game game, PhysicScene scene) : base(game)
    {
        this.physicScene = scene;
        this.Game.Window.ClientSizeChanged += this.OnClientSizeChanged;
    }

    public bool BoundingBox { get; set; } = false;

    public bool CollisionPoint { get; set; } = false;

    public override void Initialize()
    {
        base.Initialize();
        this.UpdateSettings();
        this.Game.GraphicsDevice.DeviceReset += this.OnClientSizeChanged;
    }

    public override void Draw(GameTime gameTime)
    {
        // Springs
        foreach (var spring in this.physicScene.Springs)
        {
            this.Lines(new[] { spring.GetMountPointA(), spring.GetMountPointB() }, Color.DarkGoldenrod);
        }

        foreach (var body in this.physicScene.Bodies)
        {
            // Body
            switch (body)
            {
                case CircleBody circle:
                    this.Circle(circle.Position, circle.Radius, Color.Gray);
                    this.Lines(
                        new[]
                        {
                            circle.Position,
                            circle.Position + (new FlatVector(MathF.Cos(circle.Angle), MathF.Sin(circle.Angle)) * circle.Radius)
                        },
                        Color.Gray);
                    break;
                case InverseCircleBody inverseCircle:
                    this.Circle(inverseCircle.Position, inverseCircle.Radius, Color.Gray);
                    break;
                case PolygonBody polygon:
                    this.ClosedLine(polygon.AbsoluteVertices().ToArray(), Color.Gray);
                    break;
            }

            // Bounding box
            if (this.BoundingBox)
            {
                this.AABB(body.BoundingBox, body.IsFrozen ? Color.DarkGreen : Color.DarkRed);
            }
        }

        // Collision point
        if (this.CollisionPoint)
        {
            foreach (var manifold in this.physicScene.CollisionManifolds)
            {
                this.Circle(manifold.CollisionPoint1, 1, Color.Cyan, 5);

                if (manifold.CollisionPoint2.HasValue)
                {
                    this.Circle(manifold.CollisionPoint2.Value, 1, Color.Cyan, 5);
                }
            }
        }
    }

    public void Circle(FlatVector position, float radius, Color color, int pointCount = 60)
        => this.ClosedLine( 
            Enumerable
                .Range(0, pointCount + 1)
                .Select(i =>
                {
                    var angle = (float)i / pointCount * MathHelper.TwoPi;
                    return position + new FlatVector(MathF.Cos(angle) * radius, MathF.Sin(angle) * radius);
                })
                .ToArray(),
            color);

    public void AABB(FlatVector position, FlatVector size, Color color)
        => this.ClosedLine(
            new FlatVector[]
            {
                position - (size / 2),
                position + (new FlatVector(.5f, -.5f) * size),
                position + (size / 2),
                position + (new FlatVector(-.5f, .5f) * size)
            },
            color);

    public void AABB(BoundingRectangle boundingBox, Color color)
        => this.AABB(boundingBox.Center, boundingBox.Size, color);

    public void ClosedLine(FlatVector[] vertices, Color color)
    {
        this.Lines(vertices, color);
        this.Lines(new[] { vertices[0], vertices[^1] }, color);
    }

    public void Point(FlatVector position, Color color)
    {
        if (this.effect is null)
        {
            throw new Exception("Physic drawer must be initialized");
        }

        foreach (var pass in this.effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            this.graphicsDevice!.DrawUserPrimitives(
                PrimitiveType.PointList,
                new VertexPositionColor[] { this.ToVertex(position, color) },
                0,
                1);
        }
    }

    public void Lines(FlatVector[] vertices, Color color)
    {
        if (this.effect is null)
        {
            throw new Exception("Physic drawer must be initialized");
        }

        foreach (var pass in this.effect.CurrentTechnique.Passes)
        {
            pass.Apply();
            this.graphicsDevice!.DrawUserPrimitives(
                PrimitiveType.LineStrip,
                vertices.Select(v => this.ToVertex(v, color)).ToArray(),
                0,
                vertices.Length - 1);
        }
    }

    protected override void Dispose(bool disposing)
    {
        this.Game.Window.ClientSizeChanged -= this.OnClientSizeChanged;
        this.Game.GraphicsDevice.DeviceReset -= this.OnClientSizeChanged;

        base.Dispose(disposing);
    }

    private VertexPositionColor ToVertex(FlatVector a, Color color)
        => new(new(a.X, this.size.Y - a.Y, 0), color);

    private void OnClientSizeChanged(object? sender, EventArgs e)
        => this.UpdateSettings();

    private void UpdateSettings()
    {
        this.size = this.Game.Window.ClientBounds.Size.ToVector2();
        this.graphicsDevice = this.Game.GraphicsDevice;
        this.effect = new(this.graphicsDevice)
        {
            View = Matrix.Identity,
            Projection = Matrix.CreateOrthographicOffCenter(0, this.size.X, 0, this.size.Y, 0, 1),
            FogEnabled = false,
            LightingEnabled = false,
            TextureEnabled = false,
            VertexColorEnabled = true,
            PreferPerPixelLighting = false,
        };
    }
}
