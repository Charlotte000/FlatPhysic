namespace FlatPhysic.Utils;

using FlatPhysic;

internal static class CollisionSolver
{
    private static readonly FlatVector[] collisionPoints = new[] { FlatVector.Zero, FlatVector.Zero };
    
    private static readonly FlatVector[] normalImpulses = new[] { FlatVector.Zero, FlatVector.Zero };

    private static readonly float[] normalImpulseMags = new[] { 0f, 0f };

    private static readonly FlatVector[] tangentImpulses = new[] { FlatVector.Zero, FlatVector.Zero };

    private static readonly FlatVector[] aPerps = new[] { FlatVector.Zero, FlatVector.Zero };

    private static readonly FlatVector[] bPerps = new[] { FlatVector.Zero, FlatVector.Zero };

    public static void Move(in CollisionManifold manifold)
    {
        if (manifold.Depth == 0)
        {
            return;
        }

        switch (manifold.BodyA.IsStatic, manifold.BodyB.IsStatic)
        {
            case (false, false):
            {
                var delta = manifold.Normal * manifold.Depth / 2;
                manifold.BodyA.Position += delta;
                manifold.BodyB.Position -= delta;
                break;
            }
            case (false, true):
            {
                manifold.BodyA.Position += manifold.Normal * manifold.Depth;
                break;
            }
            case (true, false):
            {
                manifold.BodyB.Position -= manifold.Normal * manifold.Depth;
                break;
            }
            case (true, true):
                break;
        }
    }

    public static void Push(in CollisionManifold manifold)
    {
        if (manifold.Depth == 0)
        {
            return;
        }

        var collisionPointCount = manifold.CollisionPoint2.HasValue ? 2 : 1;
        
        // Reset parameters
        CollisionSolver.collisionPoints[0] = manifold.CollisionPoint1;
        CollisionSolver.collisionPoints[1] = manifold.CollisionPoint2 ?? FlatVector.Zero;
        for (int i = 0; i < collisionPointCount; i++)
        {
            CollisionSolver.normalImpulses[i] = FlatVector.Zero;
            CollisionSolver.tangentImpulses[i] = FlatVector.Zero;
            CollisionSolver.aPerps[i] = FlatVector.Zero;
            CollisionSolver.bPerps[i] = FlatVector.Zero;
            CollisionSolver.normalImpulseMags[i] = 0;
        }

        // Calculate normal impulses
        var restitution = MathF.Min(manifold.BodyA.Restitution, manifold.BodyB.Restitution);
        for (int i = 0; i < collisionPointCount; i++)
        {
            var collisionPoint = CollisionSolver.collisionPoints[i];

            var aPerp = (collisionPoint - manifold.BodyA.Position).Perpendicular();
            var bPerp = (collisionPoint - manifold.BodyB.Position).Perpendicular();
            CollisionSolver.aPerps[i] = aPerp;
            CollisionSolver.bPerps[i] = bPerp;

            var relativeVelocity = manifold.BodyB.LinearVelocity + (bPerp * manifold.BodyB.AngularVelocity) -
                manifold.BodyA.LinearVelocity - (aPerp * manifold.BodyA.AngularVelocity);

            var contactVelocityMag = FlatVector.Dot(relativeVelocity, manifold.Normal);
            if (contactVelocityMag < 0)
            {
                continue;
            }

            var rAPerpDotN = FlatVector.Dot(aPerp, manifold.Normal);
            var rBPerpDotN = FlatVector.Dot(bPerp, manifold.Normal);
            var normalImpulseMag = -(1 + restitution) * contactVelocityMag;
            normalImpulseMag /= manifold.BodyA.MassInv +
                manifold.BodyB.MassInv +
                (rAPerpDotN * rAPerpDotN * manifold.BodyA.InertiaInv) +
                (rBPerpDotN * rBPerpDotN * manifold.BodyB.InertiaInv);
            normalImpulseMag /= collisionPointCount;
            var normalImpulse = normalImpulseMag * manifold.Normal;

            CollisionSolver.normalImpulses[i] = normalImpulse;
            CollisionSolver.normalImpulseMags[i] = MathF.Abs(normalImpulseMag);
        }
       
        // Apply normal impulse
        for (int i = 0; i < collisionPointCount; i++)
        {
            var impulse = CollisionSolver.normalImpulses[i];
            var collisionPoint = CollisionSolver.collisionPoints[i];

            manifold.BodyA.ApplyImpulse(collisionPoint, -impulse);
            manifold.BodyB.ApplyImpulse(collisionPoint, impulse);
        }

        // Calculate tangent impulses
        var staticFriction = (manifold.BodyA.StaticFriction + manifold.BodyB.StaticFriction) * .5f;
        var dynamicFriction = (manifold.BodyA.DynamicFriction + manifold.BodyB.DynamicFriction) * .5f;
        for (int i = 0; i < collisionPointCount; i++)
        {
            var aPerp = CollisionSolver.aPerps[i];
            var bPerp = CollisionSolver.bPerps[i];
            var relativeVelocity = manifold.BodyB.LinearVelocity + (bPerp * manifold.BodyB.AngularVelocity) -
                manifold.BodyA.LinearVelocity - (aPerp * manifold.BodyA.AngularVelocity);

            var tangent = relativeVelocity - (FlatVector.Dot(relativeVelocity, manifold.Normal) * manifold.Normal);
            if (tangent == FlatVector.Zero)
            {
                continue;
            }

            tangent = FlatVector.Normalize(tangent);

            var rAPerpDotT = FlatVector.Dot(aPerp, tangent);
            var rBPerpDotT = FlatVector.Dot(bPerp, tangent);
            var tangentImpulseMag = -FlatVector.Dot(relativeVelocity, tangent);
            tangentImpulseMag /= manifold.BodyA.MassInv +
                manifold.BodyB.MassInv +
                (rAPerpDotT * rAPerpDotT * manifold.BodyA.InertiaInv) +
                (rBPerpDotT * rBPerpDotT * manifold.BodyB.InertiaInv);
            tangentImpulseMag /= collisionPointCount;

            var normalImpulseMag = CollisionSolver.normalImpulseMags[i];
            var tangentImpulse = MathF.Abs(tangentImpulseMag) <= normalImpulseMag * staticFriction // Coulomb's law
                ? tangentImpulseMag * tangent
                : -normalImpulseMag * dynamicFriction * tangent;

            CollisionSolver.tangentImpulses[i] = tangentImpulse;
        }

        // Apply tangent impulse
        for (int i = 0; i < collisionPointCount; i++)
        {
            var impulse = CollisionSolver.tangentImpulses[i];
            var collisionPoint = CollisionSolver.collisionPoints[i];

            manifold.BodyA.ApplyImpulse(collisionPoint, -impulse);
            manifold.BodyB.ApplyImpulse(collisionPoint, impulse);
        }
    }
}
