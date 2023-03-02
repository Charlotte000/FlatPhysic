namespace FlatPhysic;

public readonly struct FlatVector
{
    public static readonly FlatVector Zero = new(0, 0);

    public FlatVector(in float x, in float y)
    {
        this.X = x;
        this.Y = y;
    }

    public FlatVector(in float value)
    {
        this.X = value;
        this.Y = value;
    }

    public readonly float X { get; }

    public readonly float Y { get; }

    public float LengthSquared()
    => (this.X * this.X) + (this.Y * this.Y);

    public float Length()
        => MathF.Sqrt((this.X * this.X) + (this.Y * this.Y));

    public float LengthInv()
        => MathF.ReciprocalSqrtEstimate((this.X * this.X) + (this.Y * this.Y));

    public FlatVector Perpendicular()
        => new(-this.Y, this.X);

    public FlatVector Rotate(float angle)
    {
        var cos = MathF.Cos(angle);
        var sin = MathF.Sin(angle);
        return new((this.X * cos) - (this.Y * sin), (this.X * sin) + (this.Y * cos));
    }

    public override bool Equals(object? obj)
        => obj is FlatVector v && v.X == this.X && v.Y == this.Y;

    public override int GetHashCode()
        => HashCode.Combine(this.X, this.Y);

    public override string ToString()
        => $"FlatVector({this.X}, {this.Y})";

    public static FlatVector operator +(in FlatVector v1, in FlatVector v2)
        => new(v1.X + v2.X, v1.Y + v2.Y);

    public static FlatVector operator -(in FlatVector v1, in FlatVector v2)
        => new(v1.X - v2.X, v1.Y - v2.Y);

    public static FlatVector operator -(in FlatVector v1)
        => new(-v1.X, -v1.Y);

    public static FlatVector operator *(in FlatVector vector, in float value)
        => new(vector.X * value, vector.Y * value);

    public static FlatVector operator *(in float value, in FlatVector vector)
        => new(vector.X * value, vector.Y * value);

    public static FlatVector operator *(in FlatVector v1, in FlatVector v2)
        => new(v1.X * v2.X, v1.Y * v2.Y);

    public static FlatVector operator /(in FlatVector vector, in float value)
        => new(vector.X / value, vector.Y / value);

    public static bool operator ==(in FlatVector v1, in FlatVector v2)
        => v1.X == v2.X && v1.Y == v2.Y;

    public static bool operator !=(in FlatVector v1, in FlatVector v2)
        => v1.X != v2.X || v1.Y != v2.Y;

    public static float Dot(in FlatVector v1, in FlatVector v2)
        => (v1.X * v2.X) + (v1.Y * v2.Y);

    public static FlatVector Normalize(in FlatVector v)
    {
        var lengthInv = MathF.ReciprocalSqrtEstimate((v.X * v.X) + (v.Y * v.Y));
        return new(v.X * lengthInv, v.Y * lengthInv);
    }

    public static float Cross(in FlatVector a, in FlatVector b)
        => (a.X * b.Y) - (a.Y * b.X);

    public static FlatVector FromAngle(in float angle)
        => new(MathF.Cos(angle), MathF.Sin(angle));
}
