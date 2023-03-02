namespace FlatPhysic;

using FlatPhysic.Bodies;
using FlatPhysic.Constraints;
using FlatPhysic.Utils;

public class PhysicScene
{
    public const float Resolution = .5f;

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
    public uint FreezeDelay { get; set; } = 10;

    /// <summary>
    /// List of <see cref="RigidBody"/> in the scene
    /// </summary>
    public List<RigidBody> Bodies { get; } = new();

    /// <summary>
    /// List of <see cref="IConstraint"/> in the scene
    /// </summary>
    public List<IConstraint> Constraints { get; } = new();

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

            // Apply constraints
            foreach (var constraint in this.Constraints)
            {
                constraint.Apply(dT);
            }

            for (int i = 0; i < this.Bodies.Count; i++)
            {
                // Apply gravity
                if (!this.Bodies[i].IsStatic && !this.Bodies[i].IsFrozen)
                {
                    this.Bodies[i].LinearVelocity += this.Gravity * dT;
                }

                // Collision detection
                for (int j = i + 1; j < this.Bodies.Count; j++)
                {
                    RigidBody bodyA = this.Bodies[i], bodyB = this.Bodies[j];

                    if ((bodyA.IsStatic && bodyB.IsStatic) ||
                        this.Constraints.Any(c => c is BodyAttachment b && b.DisableCollision && b.Contains(bodyA) && b.Contains(bodyB)))
                    {
                        continue;
                    }

                    if (!(bodyA.IsFrozen && bodyB.IsFrozen) && bodyA.BoundingBox.Intersects(bodyB.BoundingBox))
                    {
                        if (CollisionDetection.RigidRigid(bodyA, bodyB, out var manifold))
                        {
                            CollisionSolver.Move(manifold);
                            this.CollisionManifolds.Add(manifold);
                            if (!bodyA.IsStatic)
                            {
                                bodyA.IsFrozen = false;
                            }

                            if (!bodyB.IsStatic)
                            {
                                bodyB.IsFrozen = false;
                            }
                        }
                    }
                }

                this.Bodies[i].Move(dT);
            }

            // Collision solving
            foreach (var manifold in this.CollisionManifolds)
            {
                CollisionSolver.Push(manifold);
            }

            // Air drag
            foreach (var body in this.Bodies)
            {
                body.ApplyAirFriction(dT);
            }
        }

        // Try to freeze bodies
        if (this.AllowFreeze)
        {
            foreach (var body in this.Bodies)
            {
                body.TryFreeze(this.MinLinearVelocity, this.MinAngularVelocity, this.FreezeDelay);
            }
        }
    }
}
