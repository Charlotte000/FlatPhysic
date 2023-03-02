﻿namespace FlatPhysic.Bodies;

using FlatPhysic;

public class PolygonBody : RigidBody
{
    public PolygonBody(FlatVector position, FlatVector[] vertices) : base(position)
    {
        this.Vertices = vertices;
    }

    public PolygonBody(FlatVector position, FlatVector[] vertices, float mass) : base(position, mass)
    {
        this.Vertices = vertices;

        var size = this.BoundgingSize();
        var sizeSq = size * size;
        this.Inertia = this.Mass * (sizeSq.X + sizeSq.Y) / 12;
    }

    public FlatVector[] Vertices { get; set; }

    public IEnumerable<FlatVector> AbsoluteVertices()
    {
        foreach (var v in this.Vertices)
        {
            yield return v + this.Position;
        }
    }

    public IEnumerable<(FlatVector a, FlatVector b)> AbsoluteVertexPairs()
    {
        for (int i = 0; i < this.Vertices.Length; i++)
        {
            yield return (this.Vertices[i] + this.Position, this.Vertices[(i + 1) % this.Vertices.Length] + this.Position);
        }
    }

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
        this.BoundingBox = new(this.Position, this.BoundgingSize());
    }

    private FlatVector BoundgingSize()
    {
        float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
        foreach (var v in this.Vertices)
        {
            minX = MathF.Min(minX, v.X);
            minY = MathF.Min(minY, v.Y);

            maxX = MathF.Max(maxX, v.X);
            maxY = MathF.Max(maxY, v.Y);
        }

        return new(maxX - minX, maxY - minY);
    }

    public static PolygonBody CreateBox(FlatVector position, FlatVector size)
        => new(
            position,
            new[] { -size / 2, new FlatVector(size.X, -size.Y) / 2, size / 2, new FlatVector(-size.X, size.Y) / 2 });

    public static PolygonBody CreateBox(FlatVector position, FlatVector size, float mass)
        => new(
            position,
            new[] { -size / 2, new FlatVector(size.X, -size.Y) / 2, size / 2, new FlatVector(-size.X, size.Y) / 2 },
            mass);

    public static PolygonBody CreateRegularPolygon(FlatVector position, FlatVector size, uint pointCount)
        => new(
            position,
            Enumerable.Range(0, (int)pointCount).Select(i =>
            {
                var angle = (float)i / pointCount * MathF.PI * 2;
                return new FlatVector(MathF.Cos(angle), MathF.Sin(angle)) * size;
            }).ToArray());

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