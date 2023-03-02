namespace FlatPhysic.Bodies;

using FlatPhysic;

public class CircleBody : RigidBody
{
    public CircleBody(FlatVector position, float radius) : base(position)
    {
        this.Radius = radius;
    }

    public CircleBody(FlatVector position, float radius, float mass) : base(position, mass)
    {
        this.Radius = radius;
        this.Inertia = .5f * this.Mass * this.Radius * this.Radius;
    }

    public float Radius { get; set; }

    protected override void UpdateBoundingBox()
        => this.BoundingBox = new(this.Position, new(this.Radius * 2));
}
