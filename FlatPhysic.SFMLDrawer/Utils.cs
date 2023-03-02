namespace FlatPhysic.SFMLDrawer;

using SFML.System;

public static class Utils
{
    public static FlatVector ToFlat(in this Vector2f v)
        => new(v.X, v.Y);

    public static FlatVector ToFlat(in this Vector2i v)
        => new(v.X, v.Y);

    public static Vector2f ToSFML(in this FlatVector v)
        => new(v.X, v.Y);
}
