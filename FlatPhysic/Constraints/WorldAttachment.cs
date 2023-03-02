namespace FlatPhysic.Constraints;

using FlatPhysic.Bodies;
using FlatPhysic.Utils;

/// <summary>
/// Body to fixed point attachment
/// </summary>
/// <param name="Body">Body to attach</param>
/// <param name="BodyMount">Mounting point relative to the <see cref="RigidBody.Position"/> of the <see cref="RigidBody"/> without <see cref="RigidBody.Angle"/></param>
/// <param name="Position">Fixed point</param>
public readonly record struct WorldAttachment(RigidBody Body, FlatVector BodyMount, FlatVector Position) : IConstraint
{
    private static readonly CircleBody mountPoint = new(FlatVector.Zero, 0);

    /// <summary>
    /// Creates fixed point attachment to the current <paramref name="BodyMount"/>
    /// </summary>
    public WorldAttachment(RigidBody Body, FlatVector BodyMount) : this(Body, BodyMount, FlatVector.Zero)
    {
        this.Position = this.GetMountPoint();
    }

    public FlatVector GetMountPoint()
        => this.Body.Position + this.BodyMount.Rotate(this.Body.Angle);

    public void Apply(float dT)
    {
        var mount = this.GetMountPoint();
        var delta = this.Position - mount;
        var depth = delta.Length();
        var normal = delta / depth;

        WorldAttachment.mountPoint.Position = this.Position;
        WorldAttachment.mountPoint.LinearVelocity = FlatVector.Zero;
        WorldAttachment.mountPoint.AngularVelocity = 0;

        var manifold = new CollisionManifold(this.Body, WorldAttachment.mountPoint, normal, depth, mount, null);
        CollisionSolver.Move(manifold);
        CollisionSolver.Push(manifold);
    }
}
