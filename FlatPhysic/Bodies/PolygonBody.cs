namespace FlatPhysic.Bodies;

using FlatPhysic;

/// <summary>
/// The polygon body with the specified vertices
/// </summary>
public class PolygonBody : RigidBody
{
    /// <summary>
    /// Creates a static polygon body
    /// </summary>
    /// <remarks>
    /// <inheritdoc cref="RigidBody.RigidBody(FlatVector)"/>
    /// </remarks>
    public PolygonBody(FlatVector position, FlatVector[] vertices) : base(position)
    {
        this.Vertices = vertices;
    }

    /// <summary>
    /// Creates a dynamic polygon body
    /// </summary>
    public PolygonBody(FlatVector position, FlatVector[] vertices, float mass) : base(position, mass)
    {
        this.Vertices = vertices;

        this.UpdateBoundingBox();
        var size = this.BoundingBox.Size;
        var sizeSq = size * size;
        this.Inertia = this.Mass * (sizeSq.X + sizeSq.Y) / 12;
    }

    /// <summary>
    /// The vertices of the polygon relative to the <see cref="RigidBody.Position"/>
    /// </summary>
    public FlatVector[] Vertices { get; set; }

    /// <summary>
    /// Returns the absolute values of the polygon <see cref="PolygonBody.Vertices"/>
    /// </summary>
    public IEnumerable<FlatVector> AbsoluteVertices()
    {
        foreach (var v in this.Vertices)
        {
            yield return v + this.Position;
        }
    }

    /// <summary>
    /// Returns absolute values of pairs of polygon <see cref="PolygonBody.Vertices"/>
    /// </summary>
    public IEnumerable<(FlatVector a, FlatVector b)> AbsoluteVertexPairs()
    {
        for (int i = 0; i < this.Vertices.Length; i++)
        {
            yield return (this.Vertices[i] + this.Position, this.Vertices[(i + 1) % this.Vertices.Length] + this.Position);
        }
    }

    /// <summary>
    /// Returns the values of pairs of polygon <see cref="PolygonBody.Vertices"/> relative to the <see cref="RigidBody.Position"/>
    /// </summary>
    /// <returns></returns>
    public IEnumerable<(FlatVector a, FlatVector b)> RelativeVertexPairs()
    {
        for (int i = 0; i < this.Vertices.Length; i++)
        {
            yield return (this.Vertices[i], this.Vertices[(i + 1) % this.Vertices.Length]);
        }
    }

    protected override void Rotate(float deltaAngle)
    {
        for (int i = 0; i < this.Vertices.Length; i++)
        {
            this.Vertices[i] = this.Vertices[i].Rotate(deltaAngle);
        }
    }

    protected override void UpdateBoundingBox()
    {
        float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
        foreach (var v in this.AbsoluteVertices())
        {
            minX = MathF.Min(minX, v.X);
            minY = MathF.Min(minY, v.Y);

            maxX = MathF.Max(maxX, v.X);
            maxY = MathF.Max(maxY, v.Y);
        }

        var leftUp = new FlatVector(minX, minY);
        var rightDown = new FlatVector(maxX, maxY);

        this.BoundingBox = new((rightDown + leftUp) / 2, rightDown - leftUp);
    }

    /// <summary>
    /// Creates a static box
    /// </summary>
    /// <remarks>
    /// <inheritdoc cref="RigidBody.RigidBody(FlatVector)"/>
    /// </remarks>
    public static PolygonBody CreateBox(FlatVector position, FlatVector size)
        => new(
            position,
            new[] { -size / 2, new FlatVector(size.X, -size.Y) / 2, size / 2, new FlatVector(-size.X, size.Y) / 2 });

    /// <summary>
    /// Creates a dynamic box
    /// </summary>
    public static PolygonBody CreateBox(FlatVector position, FlatVector size, float mass)
        => new(
            position,
            new[] { -size / 2, new FlatVector(size.X, -size.Y) / 2, size / 2, new FlatVector(-size.X, size.Y) / 2 },
            mass);

    /// <summary>
    /// Creates a static regular polygon with the specified number of points <paramref name="pointCount"/>
    /// </summary>
    /// <remarks>
    /// <inheritdoc cref="RigidBody.RigidBody(FlatVector)"/>
    /// </remarks>
    public static PolygonBody CreateRegularPolygon(FlatVector position, FlatVector size, uint pointCount)
        => new(
            position,
            Enumerable.Range(0, (int)pointCount).Select(i =>
            {
                var angle = (float)i / pointCount * MathF.PI * 2;
                return new FlatVector(MathF.Cos(angle), MathF.Sin(angle)) * size;
            }).ToArray());

    /// <summary>
    /// Creates a dynamic regular polygon with the specified number of points <paramref name="pointCount"/>
    /// </summary>
    public static PolygonBody CreateRegularPolygon(FlatVector position, FlatVector size, uint pointCount, float mass)
        => new(
            position,
            Enumerable.Range(0, (int)pointCount).Select(i =>
            {
                var angle = (float)i / pointCount * MathF.PI * 2;
                return new FlatVector(MathF.Cos(angle), MathF.Sin(angle)) * size;
            }).ToArray(),
            mass);
}
