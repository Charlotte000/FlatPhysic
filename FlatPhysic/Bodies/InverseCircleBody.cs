namespace FlatPhysic.Bodies;

using FlatPhysic;

/// <summary>
/// Perfect inverse circle body
/// </summary>
public class InverseCircleBody : RigidBody
{
    /// <summary>
    /// Creates a static inverse circle body
    /// </summary>
    /// <remarks>
    /// <inheritdoc cref="RigidBody.RigidBody(FlatVector)"/>
    /// </remarks>
    public InverseCircleBody(FlatVector position, float radius) : base(position)
    {
        this.Radius = radius;
    }

    public float Radius { get; set; }

    protected override void UpdateBoundingBox()
        => this.BoundingBox = new(this.Position, new(this.Radius * 2));
}
