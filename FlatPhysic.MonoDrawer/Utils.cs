namespace FlatPhysic.MonoDrawer;

using Microsoft.Xna.Framework;

public static class Utils
{
    public static Vector2 ToVector2(in this FlatVector v)
        => new(v.X, v.Y);

    public static FlatVector ToFlat(in this Vector2 v)
        => new(v.X, v.Y);

    public static FlatVector ToFlat(in this Point v)
        => new(v.X, v.Y);
}
