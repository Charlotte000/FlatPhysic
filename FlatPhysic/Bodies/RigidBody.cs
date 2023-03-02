namespace FlatPhysic.Bodies;

using FlatPhysic;
using FlatPhysic.Utils;

public abstract class RigidBody
{
    private float mass;

    private float massInv;

    private float inertia;

    private float inertiaInv;

    private uint frozenCount;

    private float angle;

    public RigidBody(FlatVector position)
    {
        this.Position = position;
        
        this.mass = float.MaxValue;
        this.massInv = 0;

        this.inertia = float.MaxValue;
        this.inertiaInv = 0;

        this.IsStatic = true;
    }

    public RigidBody(FlatVector position, float mass)
    {
        this.Position = position;
        this.Mass = mass;
        this.Inertia = 60f;
        this.IsStatic = false;
    }

    public event EventHandler<CollisionEventArgs>? OnCollision;

    public bool IsStatic { get; set; }

    public bool IsFrozen { get; internal set; }

    public BoundingRectangle BoundingBox { get; protected set; }

    public FlatVector Position { get; set; }

    public float Angle
    {
        get => this.angle;
        set
        {
            this.Rotate(value - this.angle);
            this.angle = value;
        }
    }

    public FlatVector LinearVelocity { get; set; }

    public float AngularVelocity { get; set; }

    public float Restitution { get; set; } = .2f;

    public float StaticFriction { get; set; } = .6f;

    public float DynamicFriction { get; set; } = .4f;

    public float AirFriction { get; set; } = .0005f;

    public float Mass
    {
        get => this.mass;
        set
        {
            this.mass = value;
            this.massInv = 1 / value;
        }
    }

    public float MassInv
    {
        get => this.massInv;
        set
        {
            this.massInv = value;
            this.mass = 1 / value;
        }
    }

    public float Inertia
    {
        get => this.inertia;
        set
        {
            this.inertia = value;
            this.inertiaInv = 1 / value;
        }
    }

    public float InertiaInv
    {
        get => this.inertiaInv;
        set
        {
            this.inertiaInv = value;
            this.inertia = 1 / value;
        }
    }

    public void Move(float dT)
    {
        if (this.IsFrozen)
        {
            this.frozenCount = 0;
            return;
        }

        this.Position += this.LinearVelocity * dT;
        this.Angle += this.AngularVelocity * dT;
        this.UpdateBoundingBox();
    }

    public void ApplyImpulse(FlatVector point, FlatVector vector)
    {
        this.LinearVelocity += vector * this.MassInv;
        this.AngularVelocity += FlatVector.Cross(point - this.Position, vector) * this.InertiaInv;
    }

    public FlatVector AbsoluteVelocity(FlatVector point)
    {
        var perpendicular = (point - this.Position).Perpendicular();
        return this.LinearVelocity + (perpendicular * this.AngularVelocity);
    }

    internal void TryFreeze(float minLinearVelocity, float minAngularVelocity, uint freezeDelay)
    {
        if (MathUtils.NearlyEqual(this.LinearVelocity, FlatVector.Zero, minLinearVelocity) &&
            MathUtils.NearlyEqual(this.AngularVelocity, 0, minAngularVelocity))
        {
            this.frozenCount++;

            if (this.frozenCount >= freezeDelay)
            {
                this.frozenCount = 0;
                this.IsFrozen = true;

                this.LinearVelocity *= 0;
                this.AngularVelocity *= 0;
            }
        }
        else
        {
            this.frozenCount = 0;
        }
    }

    internal void ApplyAirFriction(float dT)
    {
        if (this.LinearVelocity == FlatVector.Zero)
        {
            return;
        }

        var normal = -FlatVector.Normalize(this.LinearVelocity);
        var dragMag = this.LinearVelocity.LengthSquared() * this.AirFriction;
        var dragImpulse = normal * dragMag;

        this.LinearVelocity += dragImpulse * dT;
    }

    internal void InvokeOnCollision(RigidBody body, FlatVector normal)
        => this.OnCollision?.Invoke(this, new(body, normal));

    protected virtual void Rotate(float deltaAngle)
    {
    }

    protected abstract void UpdateBoundingBox();
}
