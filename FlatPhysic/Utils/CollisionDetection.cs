namespace FlatPhysic.Utils;

using FlatPhysic;
using FlatPhysic.Bodies;

internal static class CollisionDetection
{
    public static bool RigidRigid(
        in RigidBody body1,
        in RigidBody body2,
        out CollisionManifold manifold)
    {
        manifold = default;
        return (body1, body2) switch
        {
            (CircleBody circle1, CircleBody circle2) =>
                CollisionDetection.CircleCircle(circle1, circle2, out manifold),
            (InverseCircleBody inverseCircle, CircleBody circle) =>
                CollisionDetection.CircleInverseCircle(circle, inverseCircle, out manifold),
            (CircleBody circle, InverseCircleBody inverseCircle) =>
                CollisionDetection.CircleInverseCircle(circle, inverseCircle, out manifold),
            (PolygonBody polygon1, PolygonBody polygon2) =>
                CollisionDetection.PolygonPolygon(polygon1, polygon2, out manifold),
            (PolygonBody polygon, InverseCircleBody inverseCircle) =>
                CollisionDetection.PolygonInverseCircle(polygon, inverseCircle, out manifold),
            (InverseCircleBody inverseCircle, PolygonBody polygon) =>
                CollisionDetection.PolygonInverseCircle(polygon, inverseCircle, out manifold),
            (PolygonBody polygon, CircleBody circle) =>
                CollisionDetection.PolygonCircle(polygon, circle, out manifold),
            (CircleBody circle, PolygonBody polygon) =>
                CollisionDetection.PolygonCircle(polygon, circle, out manifold),
            _ => false,
        };
    }

    public static bool PolygonPolygon(
        in PolygonBody polygon1,
        in PolygonBody polygon2,
        out CollisionManifold manifold)
    {
        var normal = FlatVector.Zero;
        var depth = float.MaxValue;
        if (CollisionDetection.SeparatingAxisTheorem(polygon1, polygon2, ref normal, ref depth) &&
            CollisionDetection.SeparatingAxisTheorem(polygon2, polygon1, ref normal, ref depth))
        {
            CollisionDetection.FindCollisionPoint(polygon1, polygon2, out var collisionPoint1, out var collisionPoint2);

            // Setting normal direction to bodyA
            if (FlatVector.Dot(polygon1.Position - polygon2.Position, normal) < 0)
            {
                normal *= -1;
            }

            manifold = new(polygon1, polygon2, normal, depth, collisionPoint1, collisionPoint2);
            return true;
        }

        manifold = default;
        return false;
    }

    public static bool PolygonCircle(
        in PolygonBody polygon,
        in CircleBody circle,
        out CollisionManifold manifold)
    {
        manifold = default;

        var normal = FlatVector.Zero;
        var depth = float.MaxValue;

        if (!CollisionDetection.SeparatingAxisTheorem(polygon, circle.Position, circle.Radius, ref normal, ref depth))
        {
            return false;
        }

        // Closest polygon point check
        var position = circle.Position;
        var closestPoint = polygon.AbsoluteVertices().MinBy(v => (v - position).LengthSquared());
        var axis = FlatVector.Normalize(closestPoint - circle.Position);
        if (!CollisionDetection.SATStep(
            polygon.AbsoluteVertices(),
            CollisionDetection.Project(circle.Position, circle.Radius, axis),
            axis,
            ref normal,
            ref depth))
        {
            return false;
        }

        // Find collision point
        CollisionDetection.FindCollisionPoint(polygon, circle.Position, out var collisionPoint);

        // Setting normal direction to bodyA
        if (FlatVector.Dot(polygon.Position - circle.Position, normal) < 0)
        {
            normal *= -1;
        }

        manifold = new(polygon, circle, normal, depth, collisionPoint, null);
        return true;
    }

    public static bool PolygonInverseCircle(
        in PolygonBody polygon,
        in InverseCircleBody inverseCircle,
        out CollisionManifold manifold)
    {
        manifold = default;

        var position = inverseCircle.Position;
        var farthestPoint = polygon.AbsoluteVertices().MaxBy(v => (v - position).LengthSquared());
        var delta = inverseCircle.Position - farthestPoint;
        if (delta.LengthSquared() <= inverseCircle.Radius * inverseCircle.Radius || delta == FlatVector.Zero)
        {
            return false;
        }

        var length = delta.Length();
        var normal = delta / length;
        manifold = new(polygon, inverseCircle, normal, length - inverseCircle.Radius, inverseCircle.Position - (normal * inverseCircle.Radius), null);
        return true;
    }

    public static bool CircleCircle(
        in CircleBody circle1,
        in CircleBody circle2,
        out CollisionManifold manifold)
    {
        manifold = default;

        var radiuses = circle1.Radius + circle2.Radius;
        var delta = circle1.Position - circle2.Position;
        if (delta.LengthSquared() >= radiuses * radiuses || delta == FlatVector.Zero)
        {
            return false;
        }

        var length = delta.Length();
        var normal = delta / length;

        manifold = new(circle1, circle2, normal, radiuses - length, circle2.Position + (normal * circle2.Radius), null);
        return true;
    }

    public static bool CircleInverseCircle(
        in CircleBody circle,
        in InverseCircleBody inverseCircle,
        out CollisionManifold manifold)
    {
        manifold = default;

        var radiuses = inverseCircle.Radius - circle.Radius;
        var delta = inverseCircle.Position - circle.Position;
        if (delta.LengthSquared() <= radiuses * radiuses || delta == FlatVector.Zero)
        {
            return false;
        }

        var length = delta.Length();
        var normal = delta / length;

        manifold = new(circle, inverseCircle, normal, length - radiuses, circle.Position - (normal * circle.Radius), null);
        return true;
    }

    private static void FindCollisionPoint(
        in PolygonBody polygon,
        in FlatVector position,
        out FlatVector collisionPoint)
    {
        var minDistanceSq = float.MaxValue;
        collisionPoint = FlatVector.Zero;

        foreach (var (vA, vB) in polygon.AbsoluteVertexPairs())
        {
            CollisionDetection.PointLineSegmentDistance(position, vA, vB, out var distanceSq, out var cP);
            if (distanceSq < minDistanceSq)
            {
                minDistanceSq = distanceSq;
                collisionPoint = cP;
            }
        }
    }

    private static void FindCollisionPoint(
        in PolygonBody polygon1,
        in PolygonBody polygon2,
        out FlatVector collisionPoint1,
        out FlatVector? collisionPoint2)
    {
        collisionPoint1 = FlatVector.Zero;
        collisionPoint2 = null;
        var minDistSq = float.MaxValue;
        CollisionDetection.FindCollisionPointStep(polygon1, polygon2, ref minDistSq, ref collisionPoint1, ref collisionPoint2);
        CollisionDetection.FindCollisionPointStep(polygon2, polygon1, ref minDistSq, ref collisionPoint1, ref collisionPoint2);
    }

    private static void PointLineSegmentDistance(
        in FlatVector point,
        in FlatVector lineA,
        in FlatVector lineB,
        out float distanceSq,
        out FlatVector collisionPoint)
    {
        var ab = lineB - lineA;
        var ap = point - lineA;

        var projection = FlatVector.Dot(ap, ab);
        var abLenSq = ab.LengthSquared();
        var d = projection / abLenSq;

        collisionPoint = d <= 0 ? lineA : d >= 1 ? lineB : lineA + (ab * d);
        distanceSq = (point - collisionPoint).LengthSquared();
    }

    private static bool SeparatingAxisTheorem(
        in PolygonBody polygon,
        in FlatVector position,
        in float radius,
        ref FlatVector normal,
        ref float depth)
    {
        foreach (var (vA, vB) in polygon.RelativeVertexPairs())
        {
            var axis = FlatVector.Normalize((vB - vA).Perpendicular());

            if (!CollisionDetection.SATStep(
                polygon.AbsoluteVertices(),
                CollisionDetection.Project(position, radius, axis),
                axis,
                ref normal,
                ref depth))
            {
                return false;
            }
        }

        return true;
    }

    private static bool SeparatingAxisTheorem(
        in PolygonBody polygon1,
        in PolygonBody polygon2,
        ref FlatVector normal,
        ref float depth)
    {
        foreach (var (vA, vB) in polygon1.RelativeVertexPairs())
        {
            var axis = FlatVector.Normalize((vB - vA).Perpendicular());

            if (!CollisionDetection.SATStep(
                polygon1.AbsoluteVertices(),
                polygon2.AbsoluteVertices(),
                axis,
                ref normal,
                ref depth))
            {
                return false;
            }
        }

        return true;
    }

    private static bool SATStep(
        in IEnumerable<FlatVector> vertices1,
        in IEnumerable<FlatVector> vertices2,
        in FlatVector axis,
        ref FlatVector normal,
        ref float depth)
    {
        CollisionDetection.MinMaxProject(vertices1, axis, out var min1, out var max1);
        CollisionDetection.MinMaxProject(vertices2, axis, out var min2, out var max2);

        if (min1 >= max2 || min2 >= max1)
        {
            return false;
        }

        var axisDepth = MathF.Min(max2 - min1, max1 - min2);
        if (depth > axisDepth)
        {
            depth = axisDepth;
            normal = axis;
        }

        return true;
    }

    private static void FindCollisionPointStep(
        in PolygonBody polygon1,
        in PolygonBody polygon2,
        ref float minDistSq,
        ref FlatVector collisionPoint1,
        ref FlatVector? collisionPoint2)
    {
        foreach (var v in polygon1.AbsoluteVertices())
        {
            foreach (var (vA, vB) in polygon2.AbsoluteVertexPairs())
            {
                CollisionDetection.PointLineSegmentDistance(v, vA, vB, out var distanceSq, out var cP);

                if (MathUtils.NearlyEqual(distanceSq, minDistSq))
                {
                    if (!MathUtils.NearlyEqual(cP, collisionPoint1))
                    {
                        collisionPoint2 = cP;
                    }
                }
                else if (distanceSq < minDistSq)
                {
                    minDistSq = distanceSq;
                    collisionPoint1 = cP;
                }
            }
        }
    }

    private static void MinMaxProject(in IEnumerable<FlatVector> vertices, in FlatVector axis, out float min, out float max)
    {
        min = float.MaxValue;
        max = float.MinValue;

        foreach (var v in vertices)
        {
            var projection = FlatVector.Dot(v, axis);

            if (projection < min)
            {
                min = projection;
            }

            if (projection > max)
            {
                max = projection;
            }
        }
    }

    private static FlatVector[] Project(in FlatVector position, in float radius, in FlatVector axis)
    {
        var radiusAxis = axis * radius;
        return new FlatVector[] { position - radiusAxis, position + radiusAxis };
    }
}
