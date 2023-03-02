namespace FlatPhysic.Bodies;

using FlatPhysic;

/// <summary>
/// Perfect circle body
/// </summary>
/// <remarks>
/// Using this class improves performance
/// </remarks>
public class CircleBody : RigidBody
{
    /// <summary>
    /// Creates a static circle body
    /// </summary>
    /// <remarks>
    /// <inheritdoc cref="RigidBody.RigidBody(FlatVector)"/>
    /// </remarks>
    public CircleBody(FlatVector position, float radius) : base(position)
    {
        this.Radius = radius;
    }

    /// <summary>
    /// Creates a dynamic circle body
    /// </summary>
    public CircleBody(FlatVector position, float radius, float mass) : base(position, mass)
    {
        this.Radius = radius;
        this.Inertia = .5f * this.Mass * this.Radius * this.Radius;
    }

    public float Radius { get; set; }

    protected override void UpdateBoundingBox()
        => this.BoundingBox = new(this.Position, new(this.Radius * 2));
}
