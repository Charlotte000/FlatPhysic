namespace FlatPhysic.SFMLDrawer;

using FlatPhysic.Bodies;
using FlatPhysic.Constraints;
using SFML.Graphics;

public class SFMLDrawer
{
    private static readonly CircleShape circleShape = new() { FillColor = Color.Transparent, OutlineThickness = 1 };

    private static readonly RectangleShape aabbShape = new() { FillColor = Color.Transparent, OutlineThickness = 1 };

    private static readonly Color Gray = new(150, 150, 150);

    private readonly PhysicScene physicScene;

    public SFMLDrawer(PhysicScene scene)
    {
        this.physicScene = scene;
    }

    public bool BoundingBox { get; set; } = false;

    public bool CollisionPoint { get; set; } = false;

    public void Draw(RenderTarget target, RenderStates renderStates)
    {
        // Springs
        foreach (var constraint in this.physicScene.Constraints)
        {
            switch (constraint)
            {
                case Spring spring:
                    SFMLDrawer.Lines(new[] { spring.GetMountPointA(), spring.GetMountPointB() }, Color.Yellow, target, renderStates);
                    break;
                case WorldAttachment worldAttachment:
                    SFMLDrawer.Circle(worldAttachment.GetMountPoint(), 1, Color.Magenta, target, renderStates);
                    break;
                case BodyAttachment bodyAttachment:
                    SFMLDrawer.Circle(bodyAttachment.GetMountPointA(), 1, Color.Magenta, target, renderStates);
                    break;
            }
        }

        foreach (var body in this.physicScene.Bodies)
        {
            // Body
            switch (body)
            {
                case CircleBody circle:
                    SFMLDrawer.Circle(circle.Position, circle.Radius, SFMLDrawer.Gray, target, renderStates);
                    SFMLDrawer.Lines(
                        new[]
                        {
                            circle.Position,
                            circle.Position + (new FlatVector(MathF.Cos(circle.Angle), MathF.Sin(circle.Angle)) * circle.Radius)
                        },
                        SFMLDrawer.Gray,
                        target,
                        renderStates);
                    break;
                case InverseCircleBody inverseCircle:
                    SFMLDrawer.Circle(inverseCircle.Position, inverseCircle.Radius, SFMLDrawer.Gray, target, renderStates);
                    break;
                case PolygonBody polygon:
                    SFMLDrawer.ClosedLine(polygon.AbsoluteVertices().ToArray(), SFMLDrawer.Gray, target, renderStates);
                    break;
            }

            // Bounding box
            if (this.BoundingBox)
            {
                SFMLDrawer.AABB(body.BoundingBox, body.IsFrozen ? Color.Green : Color.Red, target, renderStates);
            }
        }

        // Collision point
        if (this.CollisionPoint)
        {
            foreach (var manifold in this.physicScene.CollisionManifolds)
            {
                SFMLDrawer.Circle(manifold.CollisionPoint1, 1, Color.Cyan, target, renderStates);

                if (manifold.CollisionPoint2.HasValue)
                {
                    SFMLDrawer.Circle(manifold.CollisionPoint2.Value, 1, Color.Cyan, target, renderStates);
                }
            }
        }
    }

    public static void Point(FlatVector position, Color color, RenderTarget target, RenderStates renderStates)
        => target.Draw(new Vertex[] { new(position.ToSFML(), color) }, PrimitiveType.Points, renderStates);

    public static void Circle(FlatVector position, float radius, Color color, RenderTarget target, RenderStates renderStates)
    {
        SFMLDrawer.circleShape.Radius = radius;
        SFMLDrawer.circleShape.Position = (position - new FlatVector(radius)).ToSFML();
        SFMLDrawer.circleShape.OutlineColor = color;
        target.Draw(SFMLDrawer.circleShape, renderStates);
    }

    public static void AABB(BoundingRectangle boundingBox, Color color, RenderTarget target, RenderStates renderStates)
    {
        SFMLDrawer.aabbShape.Position = (boundingBox.Center - (boundingBox.Size / 2)).ToSFML();
        SFMLDrawer.aabbShape.Size = boundingBox.Size.ToSFML();
        SFMLDrawer.aabbShape.OutlineColor = color;
        target.Draw(SFMLDrawer.aabbShape, renderStates);
    }

    public static void Lines(FlatVector[] vertices, Color color, RenderTarget target, RenderStates renderStates)
        => target.Draw(
            vertices.Select(v => new Vertex(v.ToSFML(), color)).ToArray(),
            PrimitiveType.LineStrip,
            renderStates);

    public static void ClosedLine(FlatVector[] vertices, Color color, RenderTarget target, RenderStates renderStates)
    {
        SFMLDrawer.Lines(vertices, color, target, renderStates);
        SFMLDrawer.Lines(new[] { vertices[0], vertices[^1] }, color, target, renderStates);
    }
}
