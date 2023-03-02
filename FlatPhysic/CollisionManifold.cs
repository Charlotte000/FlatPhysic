namespace FlatPhysic;

using FlatPhysic.Bodies;

public readonly record struct CollisionManifold(
    RigidBody BodyA,
    RigidBody BodyB,
    FlatVector Normal,
    float Depth,
    FlatVector CollisionPoint1,
    FlatVector? CollisionPoint2);
