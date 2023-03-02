namespace FlatPhysic;

using FlatPhysic.Bodies;

public readonly record struct CollisionEventArgs(RigidBody Body, FlatVector Normal);