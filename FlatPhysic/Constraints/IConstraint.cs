namespace FlatPhysic.Constraints;

using FlatPhysic.Bodies;

public interface IConstraint
{
    public void Apply(float dT);

    public bool Contains(RigidBody body);
}
