namespace FlatPhysic;

using FlatPhysic.Bodies;
using FlatPhysic.Utils;

public class PhysicScene
{
    public const float Resolution = .5f;

    public List<CollisionManifold> CollisionManifolds { get; } = new();

    public bool AllowFreeze { get; set; } = true;

    public uint PhysSteps { get; set; } = 10;

    public float MinLinearVelocity { get; set; } = 2.5f;

    public float MinAngularVelocity { get; set; } = .4f;

    public uint FreezeDelay { get; set; } = 10;

    public List<RigidBody> Bodies { get; } = new();

    public List<Spring> Springs { get; } = new();

    public FlatVector Gravity { get; set; } = new(0, 100);

    public void Update(float dT)
    {
        dT /= (float)this.PhysSteps;

        for (int step = 0; step < this.PhysSteps; step++)
        {
            this.CollisionManifolds.Clear();

            // Apply spring forces
            foreach (var spring in this.Springs)
            {
                spring.Apply(dT);
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

                    if (bodyA.IsStatic && bodyB.IsStatic)
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
