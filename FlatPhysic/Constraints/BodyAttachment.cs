namespace FlatPhysic.Constraints;

using FlatPhysic.Bodies;
using FlatPhysic.Utils;

/// <summary>
/// Body to body attachment
/// </summary>
/// <param name="BodyA">The first body to attach</param>
/// <param name="MountA">Mounting point relative to the <see cref="RigidBody.Position"/> of the first <see cref="RigidBody"/> without <see cref="RigidBody.Angle"/></param>
/// <param name="BodyB">The second body to attach</param>
/// <param name="MountB">Mounting point relative to the <see cref="RigidBody.Position"/> of the second <see cref="RigidBody"/> without <see cref="RigidBody.Angle"/></param>
/// <param name="DisableCollision">Disable collision check between <paramref name="BodyA"/> and <paramref name="BodyB"/></param>
public readonly record struct BodyAttachment(
    RigidBody BodyA,
    FlatVector MountA,
    RigidBody BodyB,
    FlatVector MountB,
    bool DisableCollision = true) 
    : IConstraint
{
    public FlatVector GetMountPointA()
    => this.BodyA.Position + this.MountA.Rotate(this.BodyA.Angle);

    public FlatVector GetMountPointB()
        => this.BodyB.Position + this.MountB.Rotate(this.BodyB.Angle);

    public bool Contains(RigidBody body)
        => this.BodyA == body || this.BodyB == body;

    public void Apply(float dT)
    {
        var mountA = this.GetMountPointA();
        var mountB = this.GetMountPointB();

        var delta = mountB - mountA;
        var depth = delta.Length();
        var normal = delta / depth;
        var manifold = new CollisionManifold(this.BodyA, this.BodyB, normal, depth, (mountA + mountB) / 2, null);
        CollisionSolver.Move(manifold);
        CollisionSolver.Push(manifold);
    }
}
