namespace FlatPhysic.Bodies;

using FlatPhysic;

public class InverseCircleBody : RigidBody
{
    public InverseCircleBody(FlatVector position, float radius) : base(position)
    {
        this.Radius = radius;
    }

    public float Radius { get; set; }

    protected override void UpdateBoundingBox()
        => this.BoundingBox = new(this.Position, new(this.Radius * 2));
}
