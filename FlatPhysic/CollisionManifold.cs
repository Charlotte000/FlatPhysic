namespace FlatPhysic;

using FlatPhysic.Bodies;

/// <summary>
/// Information about the collision to be resolved
/// </summary>
/// <remarks>
/// The order of the bodies does not matter
/// </remarks>
/// <param name="BodyA">The first body in the collision</param>
/// <param name="BodyB">The second body in the collision</param>
/// <param name="Normal">Normal in the collision, looking towards the <see cref="BodyA"/></param>
/// <param name="Depth">Сollision depth</param>
/// <param name="CollisionPoint1">The first point of collision</param>
/// <param name="CollisionPoint2">The second possible point of collision</param>
public readonly record struct CollisionManifold(
    RigidBody BodyA,
    RigidBody BodyB,
    FlatVector Normal,
    float Depth,
    FlatVector CollisionPoint1,
    FlatVector? CollisionPoint2);
