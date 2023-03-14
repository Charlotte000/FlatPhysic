namespace FlatPhysic.Constraints;

using FlatPhysic.Bodies;

/// <summary>
/// A spring constraint
/// </summary>
/// <param name="BodyA">The first body to attach</param>
/// <param name="MountA">Mounting point relative to the <see cref="RigidBody.Position"/> of the first <see cref="RigidBody"/> without <see cref="RigidBody.Angle"/></param>
/// <param name="BodyB">The second body to attach</param>
/// <param name="MountB">Mounting point relative to the <see cref="RigidBody.Position"/> of the second <see cref="RigidBody"/> without <see cref="RigidBody.Angle"/></param>
/// <param name="Hardness">Spring strength coefficient</param>
/// <param name="Damping">Oscillation reduction coefficient</param>
/// <param name="Length">Spring rest length</param>
public readonly record struct Spring(
    RigidBody BodyA,
    FlatVector MountA,
    RigidBody BodyB,
    FlatVector MountB,
    float Hardness,
    float Damping,
    float Length) : IConstraint
{
    /// <summary>
    /// Creates a spring with the current length
    /// </summary>
    public Spring(RigidBody BodyA, FlatVector MountA, RigidBody BodyB, FlatVector MountB, float Hardness, float Damping)
        : this(BodyA, MountA, BodyB, MountB, Hardness, Damping, 0)
    {
        this.Length = (this.GetMountPointB() - this.GetMountPointA()).Length();
    }

    public FlatVector GetMountPointA()
        => this.BodyA.Position + this.MountA.Rotate(this.BodyA.Angle);

    public FlatVector GetMountPointB()
        => this.BodyB.Position + this.MountB.Rotate(this.BodyB.Angle);

    public bool Contains(RigidBody body)
        => this.BodyA == body || this.BodyB == body;

    public void Apply(float dT)
    {
        var p1 = this.GetMountPointA();
        var p2 = this.GetMountPointB();
        var delta = p2 - p1;
        if (delta == FlatVector.Zero)
        {
            return;
        }

        var length = delta.Length();

        var norm = delta / length;

        var relativeVel = (this.BodyB.AbsoluteVelocity(p2) - this.BodyA.AbsoluteVelocity(p1));

        var dot = FlatVector.Dot(norm, relativeVel);

        var springMag = this.Hardness * (length - this.Length);
        var dampingMag = dot * this.Damping;

        var impulse = norm * (springMag + dampingMag) * dT;

        this.BodyA.ApplyImpulse(p1, impulse);
        this.BodyB.ApplyImpulse(p2, -impulse);
    }
}
