namespace FlatPhysic;

using FlatPhysic.Bodies;
using FlatPhysic.Constraints;
using FlatPhysic.Utils;
using System.Collections.ObjectModel;

public class PhysicScene
{
    public const float Resolution = .5f;

    private readonly List<RigidBody> bodies = new();

    private readonly List<IConstraint> constraints = new();

    public PhysicScene()
    {
        this.Bodies = this.bodies.AsReadOnly();
        this.Constraints = this.constraints.AsReadOnly();
    }

    /// <summary>
    /// List of collision manifolds to be resolved
    /// </summary>
    public List<CollisionManifold> CollisionManifolds { get; } = new();

    /// <summary>
    /// The count of steps in each <see cref="Update"/> call
    /// </summary>
    /// <remarks>
    /// The higher the value, the more accurate the simulation is
    /// </remarks>
    public uint PhysSteps { get; set; } = 10;

    /// <summary>
    /// Allow body freezing for performance improvement
    /// </summary>
    /// <remarks>
    /// Freezes the body according to these parameters:
    /// <list type="bullet">
    /// <item><see cref="MinLinearVelocity"/></item>
    /// <item><see cref="MinAngularVelocity"/></item>
    /// <item><see cref="FreezeDelay"/></item>
    /// </list>
    /// </remarks>
    public bool AllowFreeze { get; set; } = true;

    /// <summary>
    /// Minimal <see cref="RigidBody.LinearVelocity"/> magnitude for freezing.
    /// </summary>
    /// <remarks>
    /// Enables using <see cref="AllowFreeze"/>
    /// </remarks>
    public float MinLinearVelocity { get; set; } = 2.5f;

    /// <summary>
    /// Minimal <see cref="RigidBody.AngularVelocity"/> magnitude for freezing.
    /// </summary>
    /// <remarks>
    /// Enables using <see cref="AllowFreeze"/>
    /// </remarks>
    public float MinAngularVelocity { get; set; } = .4f;

    /// <summary>
    /// Frames count for freezing.
    /// </summary>
    /// <remarks>
    /// Enables using <see cref="AllowFreeze"/>
    /// </remarks>
    public uint FreezeDelay { get; set; } = 100;

    /// <summary>
    /// List of <see cref="RigidBody"/> in the scene
    /// </summary>
    public ReadOnlyCollection<RigidBody> Bodies { get; }

    /// <summary>
    /// List of <see cref="IConstraint"/> in the scene
    /// </summary>
    public ReadOnlyCollection<IConstraint> Constraints { get; }

    /// <summary>
    /// Gravity force
    /// </summary>
    public FlatVector Gravity { get; set; } = new(0, 100);

    /// <summary>
    /// Update physic scene
    /// </summary>
    /// <param name="dT">Delta time</param>
    /// <remarks>
    /// <list type="bullet">
    /// <item>Constraints</item>
    /// <item>Gravity force</item>
    /// <item>Detect and solve collisions</item>
    /// <item>Friction between bodies and air</item>
    /// <item>Try to freeze bodies if <see cref="PhysicScene.AllowFreeze"/> is set to true</item>
    /// </list>
    /// </remarks>
    public void Update(float dT)
    {
        dT /= (float)this.PhysSteps;
        for (int step = 0; step < this.PhysSteps; step++)
        {
            this.CollisionManifolds.Clear();
            for (int i = 0; i < this.bodies.Count; i++)
            {
                // Apply gravity
                if (!this.bodies[i].IsStatic && !this.bodies[i].IsFrozen)
                {
                    this.bodies[i].LinearVelocity += this.Gravity * dT;
                }

                // Collision detection
                for (int j = i + 1; j < this.bodies.Count; j++)
                {
                    RigidBody bodyA = this.bodies[i], bodyB = this.bodies[j];

                    if ((bodyA.IsStatic && bodyB.IsStatic) ||
                        this.constraints.Any(c => c is BodyAttachment b && b.DisableCollision && b.Contains(bodyA) && b.Contains(bodyB)))
                    {
                        continue;
                    }

                    if (!(bodyA.IsFrozen && bodyB.IsFrozen) && bodyA.BoundingBox.Intersects(bodyB.BoundingBox))
                    {
                        if (CollisionDetection.RigidRigid(bodyA, bodyB, out var manifold))
                        {
                            CollisionSolver.Move(manifold);
                            this.CollisionManifolds.Add(manifold);
                        }
                    }
                }

                this.bodies[i].Move(dT);
            }

            // Collision solving
            foreach (var manifold in this.CollisionManifolds)
            {
                CollisionSolver.Push(manifold);

                // Invoke OnCollision event
                manifold.BodyA.InvokeOnCollision(manifold.BodyB, manifold.Normal);
                manifold.BodyB.InvokeOnCollision(manifold.BodyA, -manifold.Normal);
            }

            // Apply constraints
            foreach (var constraint in this.constraints)
            {
                constraint.Apply(dT);
            }

            // Air drag
            foreach (var body in this.bodies)
            {
                body.ApplyAirFriction(dT);
            }
        }

        // Try to freeze bodies
        if (this.AllowFreeze)
        {
            foreach (var body in this.bodies)
            {
                body.TryFreeze();
            }
        }
    }

    public void AddBody(RigidBody body)
    {
        body.physicScene = this;
        this.bodies.Add(body);
    }

    /// <summary>
    /// Ensures that all related constraints are also removed
    /// </summary>
    public void RemoveBody(RigidBody body)
    {
        this.constraints.RemoveAll(c => c.Contains(body));
        body.physicScene = null;
        this.bodies.Remove(body);
    }

    public void AddConstraint(IConstraint constraint)
        => this.constraints.Add(constraint);

    public void RemoveConstraint(IConstraint constraint)
        => this.constraints.Remove(constraint);
}
