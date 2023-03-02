namespace FlatPhysic;

/// <summary>
/// Collision box
/// </summary>
/// <remarks>
/// The engine does not immediately check for geometry collisions, first it checks for box collisions, which gives a significant performance boost
/// </remarks>
public readonly struct BoundingRectangle
{
    public BoundingRectangle(FlatVector center, FlatVector size)
    {
        this.Center = center;
        this.Size = size;
    }

    public FlatVector Center { get; }

    public FlatVector Size { get; }

    public bool Intersects(BoundingRectangle rect)
    {
        var delta = rect.Center - this.Center;
        var sizes = (this.Size + rect.Size) / 2;
        return MathF.Abs(delta.X) < sizes.X && MathF.Abs(delta.Y) < sizes.Y;
    }
}
