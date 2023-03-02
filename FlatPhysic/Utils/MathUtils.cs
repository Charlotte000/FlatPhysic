namespace FlatPhysic.Utils;

internal static class MathUtils
{
    public static bool NearlyEqual(in float a, in float b, in float epsilon = PhysicScene.Resolution)
        => MathF.Abs(a - b) <= epsilon;

    public static bool NearlyEqual(in FlatVector a, in FlatVector b, in float epsilon = PhysicScene.Resolution)
        => MathF.Abs(a.X - b.X) <= epsilon && MathF.Abs(a.Y - b.Y) <= epsilon;
}
