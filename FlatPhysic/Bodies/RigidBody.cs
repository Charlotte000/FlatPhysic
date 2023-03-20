namespace FlatPhysic.Bodies;

using FlatPhysic;
using FlatPhysic.Utils;

/// <summary>
/// Base class of rigid bodies
/// </summary>
public abstract class RigidBody
{
    private float mass;

    private float massInv;

    private float inertia;

    private float inertiaInv;

    private uint frozenCount;

    private float angle;

    internal PhysicScene? physicScene;

    /// <summary>
    /// Creates a static rigid body
    /// </summary>
    /// <param name="position">Position of the body</param>
    /// <remarks>
    /// Static bodies have infinite <see cref="RigidBody.Mass"/> and <see cref="RigidBody.Inertia"/>
    /// </remarks>
    public RigidBody(FlatVector position)
    {
        this.Position = position;
        this.Mass = float.PositiveInfinity;
        this.Inertia = float.PositiveInfinity;
        this.IsStatic = true;
    }

    /// <summary>
    /// Creates a dynamic rigid body
    /// </summary>
    /// <param name="position"><inheritdoc cref="RigidBody.RigidBody(FlatVector)"/></param>
    /// <param name="mass">Body mass</param>
    public RigidBody(FlatVector position, float mass)
    {
        this.Position = position;
        this.Mass = mass;
        this.Inertia = 60f;
        this.IsStatic = false;
    }

    public event EventHandler<CollisionEventArgs>? OnCollision;

    public bool IsStatic { get; }

    /// <summary>
    /// Is the body frozen
    /// </summary>
    /// <remarks>
    /// Frozen bodies are not checked for collisions, which gives an increase in performance
    /// <para/>
    /// Check <see cref="PhysicScene.AllowFreeze"/>
    /// </remarks>
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

    /// <summary>
    /// The ratio of the change in relative velocity after a collision
    /// </summary>
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
            this.massInv = float.IsPositiveInfinity(value) || value == float.MaxValue ? 0 : 1 / value;
        }
    }

    public float MassInv
    {
        get => this.massInv;
        set
        {
            this.massInv = value;
            this.mass = float.IsPositiveInfinity(value) || value == float.MaxValue ? 0 : 1 / value;
        }
    }

    public float Inertia
    {
        get => this.inertia;
        set
        {
            this.inertia = value;
            this.inertiaInv = float.IsPositiveInfinity(value) || value == float.MaxValue ? 0 : 1 / value;
        }
    }

    public float InertiaInv
    {
        get => this.inertiaInv;
        set
        {
            this.inertiaInv = value;
            this.inertia = float.IsPositiveInfinity(value) || value == float.MaxValue ? 0 : 1 / value;
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

    /// <summary>
    /// Applies an impulse to the body at a <paramref name="point"/>
    /// </summary>
    /// <remarks>
    /// Changes linear and angular velocity
    /// </remarks>
    public void ApplyImpulse(FlatVector point, FlatVector vector)
    {
        this.LinearVelocity += vector * this.MassInv;
        this.AngularVelocity += FlatVector.Cross(point - this.Position, vector) * this.InertiaInv;

        this.TryUnfreeze();
    }

    /// <summary>
    /// Combination of <see cref="RigidBody.LinearVelocity"/> and <see cref="RigidBody.AngularVelocity"/> at a given point of the body
    /// </summary>
    public FlatVector AbsoluteVelocity(FlatVector point)
    {
        var perpendicular = (point - this.Position).Perpendicular();
        return this.LinearVelocity + (perpendicular * this.AngularVelocity);
    }

    internal void TryFreeze()
    {
        if (this.physicScene is null)
        {
            return;
        }

        if (MathUtils.NearlyEqual(this.LinearVelocity, FlatVector.Zero, this.physicScene.MinLinearVelocity) &&
            MathUtils.NearlyEqual(this.AngularVelocity, 0, this.physicScene.MinAngularVelocity))
        {
            this.frozenCount++;

            if (this.frozenCount >= this.physicScene.FreezeDelay)
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
    
    internal void TryUnfreeze()
    {
        if (this.physicScene is null)
        {
            return;
        }

        if (!MathUtils.NearlyEqual(this.LinearVelocity, FlatVector.Zero, this.physicScene.MinLinearVelocity) ||
                !MathUtils.NearlyEqual(this.AngularVelocity, 0, this.physicScene.MinAngularVelocity))
        {
            this.IsFrozen &= this.IsStatic;
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
