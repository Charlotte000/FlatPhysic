namespace FlatPhysic;

using FlatPhysic.Bodies;

public readonly record struct Spring(
    RigidBody BodyA,
    FlatVector OriginA,
    RigidBody BodyB,
    FlatVector OriginB,
    float Hardness,
    float Damping,
    float Length)
{
    public FlatVector GetMountPointA()
        => this.BodyA.Position + this.OriginA.Rotate(this.BodyA.Angle);

    public FlatVector GetMountPointB()
        => this.BodyB.Position + this.OriginB.Rotate(this.BodyB.Angle);

    internal void Apply(float dT)
    {
        var p1 = this.GetMountPointA();
        var p2 = this.GetMountPointB();
        var delta = p2 - p1;

        var length = delta.Length();

        var norm = delta / length;

        var relativeVel = this.BodyB.AbsoluteVelocity(p2) - this.BodyA.AbsoluteVelocity(p1);

        var dot = FlatVector.Dot(norm, relativeVel);

        var springMag = this.Hardness * (length - this.Length);
        var dampingMag = dot * this.Damping;

        var impulse = norm * (springMag + dampingMag) * dT;

        this.BodyA.ApplyImpulse(p1, impulse);
        this.BodyB.ApplyImpulse(p2, -impulse);
    }
}
