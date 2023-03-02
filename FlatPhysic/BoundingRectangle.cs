namespace FlatPhysic;

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
