using System;
using System.Collections.Generic;
using BulletSharp;
using BulletSharp.SoftBody;
using BulletSharp.Serialize;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace BulletTest
{
    class Physics
    {
        GraphicsDevice Device;

        Model sphere;

        SequentialImpulseConstraintSolver Solver;
        SoftBodyWorldInfo SoftBodyWorldInfo = new SoftBodyWorldInfo();
        public SoftRigidDynamicsWorld World;
        ConstraintSolver constraintSolver = new SequentialImpulseConstraintSolver();

        public static MotorControl Motor = new MotorControl();
        SteerControl Steer = new SteerControl(1);

        CollisionDispatcher Dispatcher;
        BroadphaseInterface Broadphase;
        List<CollisionShape> CollisionShapes;
        CollisionConfiguration CollisionConf;


        const int maxProxies = 32766;

        ///create 125 (5x5x5) dynamic objects
        int ArraySizeX = 5, ArraySizeY = 5, ArraySizeZ = 5;

        ///scaling of the objects (0.1 = 20 centimeter boxes )
        float StartPosX = -5;
        float StartPosY = 0;
        float StartPosZ = -3;

        public void Initialize(ContentManager Content, GraphicsDevice device)
        {
            Device = device;
            sphere = Content.Load<Model>("ball");

            // collision configuration contains default setup for memory, collision setup
            CollisionConf = new SoftBodyRigidBodyCollisionConfiguration();
            Dispatcher = new CollisionDispatcher(CollisionConf);

            Broadphase = new AxisSweep3(new Vector3(-1000, -1000, -1000),
                new Vector3(1000, 1000, 1000), maxProxies);

            // the default constraint solver.
            Solver = new SequentialImpulseConstraintSolver();

            SoftBodyWorldInfo = new SoftBodyWorldInfo
            {
                AirDensity = 1.2f,
                WaterDensity = 0,
                WaterOffset = 0,
                WaterNormal = Vector3.Zero,
                Gravity = new Vector3(0, -10, 0),
                Dispatcher = Dispatcher,
                Broadphase = Broadphase
            };
            SoftBodyWorldInfo.SparseSdf.Initialize();

            World = new SoftRigidDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConf);
            World.Gravity = new Vector3(0, -10, 0);
            World.DispatchInfo.EnableSpu = true;

            InitializeDemo();
        }

        protected void InitializeDemo()
        {

            CollisionShapes = new List<CollisionShape>();

            // create the ground
            CollisionShape groundShape = new BoxShape(50, 1, 50);
            CollisionShapes.Add(groundShape);
            CollisionObject ground = LocalCreateRigidBody(0, Matrix.Identity, groundShape);
            ground.UserObject = "Ground";

            // create a few dynamic rigidbodies
            float mass = 1.0f;

#region
            
             
            CollisionShape colShape = new BoxShape(1);
            CollisionShapes.Add(colShape);
            Vector3 localInertia = colShape.CalculateLocalInertia(mass);

            float start_x = StartPosX - ArraySizeX / 2;
            float start_y = StartPosY;
            float start_z = StartPosZ - ArraySizeZ / 2;

            int k, i, j;
            for (k = 0; k < ArraySizeY; k++)
            {
                for (i = 0; i < ArraySizeX; i++)
                {
                    for (j = 0; j < ArraySizeZ; j++)
                    {
                        Matrix startTransform = Matrix.CreateTranslation(
                            new Vector3(
                                2*i + start_x,
                                2*k + start_y,
                                2*j + start_z
                                )
                            );

                        // using motionstate is recommended, it provides interpolation capabilities
                        // and only synchronizes 'active' objects
                        DefaultMotionState myMotionState = new DefaultMotionState(startTransform);
                        RigidBodyConstructionInfo rbInfo =
                            new RigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia);
                        RigidBody body = new RigidBody(rbInfo);
                        body.UserObject = "Box";
                        // make it drop from a height
                        body.Translate(new Vector3(0, 0, 0));

                        World.AddRigidBody(body);
                        CollisionShapes.Add(body.CollisionShape);
                    }
                }
            }

#endregion
            
            //Create_SoftBox(new Vector3(0, 25f, 0), Vector3.One * 20f);
            //Create_SoftBox(new Vector3(0, 50f, 0), Vector3.One * 10f);

            for(int iteration = 0; iteration < 10; iteration++)
                Create_ClusterTorus(new Vector3(0, 250 + iteration * 20, 0), new Vector3(0, 3.131592f / 2f, 0), Vector3.One * 3);
            
            Init_ClusterCar(new Vector3(-10, 10, 10));
            //Create_ClusterModel(new Vector3(-10f, 5, 0), new Vector3(0, 0, 0), Vector3.One * 3);
        }

        public virtual void Update(float deltaTime)
        {
            KeyboardState keys = Keyboard.GetState();

            if (keys.IsKeyDown(Keys.Down))
            {
                Motor.MaxTorque = 1;
                Motor.Goal += deltaTime * 2;
            }
            else if (keys.IsKeyDown(Keys.Up))
            {
                Motor.MaxTorque = 1;
                Motor.Goal -= deltaTime * 2;
            }
            else if (keys.IsKeyDown(Keys.Left))
            {
                Steer.Angle += deltaTime;
                Steer.Angle += deltaTime;
            }
            else if (keys.IsKeyDown(Keys.Right))
            {
                Steer.Angle -= deltaTime;
                Steer.Angle -= deltaTime;
            }

            SoftBodyWorldInfo.SparseSdf.GarbageCollect();
            World.StepSimulation(deltaTime);
        }

        public void Draw(GraphicsDevice device, Effect effect)
        {
            EffectPass pass = effect.CurrentTechnique.Passes[0];

            for(int i = 0; i < World.NumCollisionObjects; i++)
            {
                CollisionObject collObj = World.CollisionObjectArray[i];
                CollisionShape collShape = CollisionShapes[i];

                switch(collShape.ShapeType)
                {
                    case BroadphaseNativeType.SoftBodyShape:
                        Vector3[] positions, normals;
                        int vertexCount = (collObj as SoftBody).GetVertexNormalData(out positions, out normals);

                        VertexPositionNormal[] vertices = new VertexPositionNormal[vertexCount];
                        for(int j = 0; j < vertices.Length; j++)
                        {
                            vertices[j].Position = positions[j];
                            vertices[j].Normal = normals[j];
                        }

                        

                        //effect.CurrentTechnique = effect.Techniques["Monochromatic"];
                        //effect.Parameters["xAmbient"].SetValue(0.5f);
                        effect.Parameters["xColor"].SetValue(Color.CornflowerBlue.ToVector3() * 2);
                        effect.Parameters["xWorld"].SetValue(collObj.WorldTransform);
                        pass.Apply();

                        device.DrawUserPrimitives(PrimitiveType.TriangleList, vertices, 0, vertexCount / 3, VertexPositionNormal.VertexDeclaration);
                        break;


                }
                

            }
        }

        public void ExitPhysics()
        {
            //remove/dispose constraints
            int i;
            for (i = World.NumConstraints - 1; i >= 0; i--)
            {
                TypedConstraint constraint = World.GetConstraint(i);
                World.RemoveConstraint(constraint);
                constraint.Dispose(); ;
            }

            //remove the rigidbodies from the dynamics world and delete them
            for (i = World.NumCollisionObjects - 1; i >= 0; i--)
            {
                CollisionObject obj = World.CollisionObjectArray[i];
                RigidBody body = obj as RigidBody;
                if (body != null && body.MotionState != null)
                {
                    body.MotionState.Dispose();
                }
                World.RemoveCollisionObject(obj);
                obj.Dispose();
            }

            //delete collision shapes
            foreach (CollisionShape shape in CollisionShapes)
                shape.Dispose();
            CollisionShapes.Clear();

            World.Dispose();
            Broadphase.Dispose();
            if (Dispatcher != null)
            {
                Dispatcher.Dispose();
            }
            CollisionConf.Dispose();
        }

        public RigidBody LocalCreateRigidBody(float mass, Matrix startTransform, CollisionShape shape)
        {
            bool isDynamic = (mass != 0.0f);

            Vector3 localInertia = Vector3.Zero;
            if (isDynamic)
                shape.CalculateLocalInertia(mass, out localInertia);

            DefaultMotionState myMotionState = new DefaultMotionState(startTransform);

            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia);
            RigidBody body = new RigidBody(rbInfo);

            World.AddRigidBody(body);

            return body;
        }

        public class PhysicsDebugDraw : DebugDraw
        {
            GraphicsDevice device;

            DebugDrawModes _debugMode;
            public override DebugDrawModes DebugMode
            {
                get { return _debugMode; }
                set { _debugMode = value; }
            }

            public PhysicsDebugDraw(GraphicsDevice device, BasicEffect effect)
            {
                this.device = device;
            }

            public override void Draw3dText(ref Vector3 location, string textString)
            {
                throw new NotImplementedException();
            }

            public override void DrawContactPoint(ref Vector3 pointOnB, ref Vector3 normalOnB, float distance, int lifeTime, Color color)
            {
                VertexPositionColor[] vertices = new VertexPositionColor[2];
                vertices[0] = new VertexPositionColor(pointOnB, color);
                vertices[1] = new VertexPositionColor(pointOnB + normalOnB, color);
                device.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, vertices, 0, 1);
            }

            public override void DrawLine(ref Vector3 from, ref Vector3 to, Color color)
            {
                VertexPositionColor[] vertices = new VertexPositionColor[2];
                vertices[0] = new VertexPositionColor(from, color);
                vertices[1] = new VertexPositionColor(to, color);
                device.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, vertices, 0, 1);
            }

            public void DrawDebugWorld(DynamicsWorld world)
            {
                world.DebugDrawWorld();
            }

            public override void ReportErrorWarning(string warningString)
            {
                throw new NotImplementedException();
            }
        }

        #region CreateObjects
        void Init_ClusterCar(Vector3 origin)
        {
            Quaternion orientation = Quaternion.CreateFromYawPitchRoll(-(float)Math.PI / 2, 0, 0);
            const float widthf = 8;
            const float widthr = 9;
            const float length = 8;
            const float height = 4;
            Vector3[] wheels = new Vector3[] {
		        new Vector3(+widthf,-height,+length), // Front left
		        new Vector3(-widthf,-height,+length), // Front right
		        new Vector3(+widthr,-height,-length), // Rear left
		        new Vector3(-widthr,-height,-length), // Rear right
	        };
            SoftBody body = Create_ClusterBunny(Vector3.Zero, Vector3.Zero);
            SoftBody wheelFL = Create_ClusterTorus(wheels[0], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(2, 4, 2));
            SoftBody wheelFR = Create_ClusterTorus(wheels[1], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(2, 4, 2));
            SoftBody wheelRL = Create_ClusterTorus(wheels[2], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(2, 5, 2));
            SoftBody wheelRR = Create_ClusterTorus(wheels[3], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(2, 5, 2));

            wheelFL.Cfg.DF =
                wheelFR.Cfg.DF =
                wheelRL.Cfg.DF =
                wheelRR.Cfg.DF = 1;

            LJoint.Specs lspecs = new LJoint.Specs();
            lspecs.Cfm = 1;
            lspecs.Erp = 1;
            lspecs.Position = Vector3.Zero;

            lspecs.Position = wheels[0]; body.AppendLinearJoint(lspecs, wheelFL);
            lspecs.Position = wheels[1]; body.AppendLinearJoint(lspecs, wheelFR);
            lspecs.Position = wheels[2]; body.AppendLinearJoint(lspecs, wheelRL);
            lspecs.Position = wheels[3]; body.AppendLinearJoint(lspecs, wheelRR);

            //"rotations-servo-liknande" som ska användas för att driva/svänga hjulen
            AJoint.Specs aspecs = new AJoint.Specs();
            aspecs.Cfm = 1;
            aspecs.Erp = 1;
            //Rotationens axel är liggande(likt hjulaxeln) axeln är riktad likt x-axeln enl nedan (KAN HA FEL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)
            aspecs.Axis = new Vector3(1, 0, 0);

            aspecs.Control = Steer; ;
            body.AppendAngularJoint(aspecs, wheelFL);
            body.AppendAngularJoint(aspecs, wheelFR);

            aspecs.Control = Motor; ;
            body.AppendAngularJoint(aspecs, wheelRL);
            body.AppendAngularJoint(aspecs, wheelRR);

            body.Rotate(orientation);
            wheelFL.Rotate(orientation);
            wheelFR.Rotate(orientation);
            wheelRL.Rotate(orientation);
            wheelRR.Rotate(orientation);
            body.Translate(origin);
            wheelFL.Translate(origin);
            wheelFR.Translate(origin);
            wheelRL.Translate(origin);
            wheelRR.Translate(origin);
            wheelFL.Cfg.PIterations =
                wheelFR.Cfg.PIterations =
                wheelRL.Cfg.PIterations =
                wheelRR.Cfg.PIterations = 1;
            wheelFL.Clusters[0].Matching =
                wheelFR.Clusters[0].Matching =
                wheelRL.Clusters[0].Matching =
                wheelRR.Clusters[0].Matching = 0.05f;
            wheelFL.Clusters[0].NodeDamping =
                wheelFR.Clusters[0].NodeDamping =
                wheelRL.Clusters[0].NodeDamping =
                wheelRR.Clusters[0].NodeDamping = 0.05f;

            //autocam=true;
        }

        SoftBody Create_ClusterBunny(Vector3 x, Vector3 a)
        {
            SoftBody psb = SoftBodyHelpers.CreateFromTriMesh(SoftBodyWorldInfo, BunnyMesh.Vertices, BunnyMesh.Indices);
            Material pm = psb.AppendMaterial();
            pm.Lst = 1;
            pm.Flags -= FMaterial.DebugDraw;
            psb.GenerateBendingConstraints(2, pm);
            psb.Cfg.PIterations = 2;
            psb.Cfg.DF = 1;
            psb.Cfg.Collisions = FCollisions.CLSS | FCollisions.CLRS;
            psb.RandomizeConstraints();
            Matrix m = Matrix.CreateFromYawPitchRoll(a.X, a.Y, a.Z) * Matrix.CreateTranslation(x);
            psb.Transform(m);
            psb.Scale(new Vector3(8, 8, 8));
            psb.SetTotalMass(150, true);
            psb.GenerateClusters(1);
            psb.UserObject = "Soft";
            World.AddSoftBody(psb);
            return (psb);
        }

        SoftBody Create_ClusterTorus(Vector3 x, Vector3 a, Vector3 s)
        {
            SoftBody psb = SoftBodyHelpers.CreateFromTriMesh(SoftBodyWorldInfo, TorusMesh.Vertices, TorusMesh.Indices);
            Material pm = psb.AppendMaterial();
            pm.Lst = 1f;
            pm.Flags -= FMaterial.DebugDraw;
            psb.GenerateBendingConstraints(2, pm);
            psb.Cfg.PIterations = 3;
            psb.Cfg.Collisions = FCollisions.CLSS | FCollisions.CLRS | FCollisions.CLSelf;
            psb.RandomizeConstraints();
            psb.Scale(s);
            Matrix m = Matrix.CreateFromYawPitchRoll(a.X, a.Y, a.Z) * Matrix.CreateTranslation(x);
            psb.Transform(m);
            psb.SetTotalMass(1000, true);
            psb.GenerateClusters(64);
            //psb.GenerateClusters(4);
            
            psb.UserObject = "Soft";
            World.AddSoftBody(psb);
            CollisionShapes.Add(psb.CollisionShape);

            return (psb);
        }


        void Create_ClusterModel(Vector3 x, Vector3 a, Vector3 s)
        {
            //foreach(ModelMesh mesh in sphere.Meshes)
                //foreach (ModelMeshPart part in mesh.MeshParts)
                //{
            Vector3[] vertices = new Vector3[sphere.Meshes[0].MeshParts[0].NumVertices];
            int[] indices = new int[sphere.Meshes[0].MeshParts[0].PrimitiveCount * 3];

                    sphere.Meshes[0].MeshParts[0].VertexBuffer.GetData<Vector3>(sphere.Meshes[0].MeshParts[0].VertexOffset * sphere.Meshes[0].MeshParts[0].VertexBuffer.VertexDeclaration.VertexStride, vertices, 0, sphere.Meshes[0].MeshParts[0].NumVertices, sphere.Meshes[0].MeshParts[0].VertexBuffer.VertexDeclaration.VertexStride);
                    sphere.Meshes[0].MeshParts[0].IndexBuffer.GetData<int>(sphere.Meshes[0].MeshParts[0].StartIndex * 2, indices, 0, sphere.Meshes[0].MeshParts[0].PrimitiveCount * 3);

                    SoftBody psb = SoftBodyHelpers.CreateFromTriMesh(SoftBodyWorldInfo, vertices, indices);

                    
                    Material pm = psb.AppendMaterial();
                    pm.Lst = 1f;
                    pm.Flags -= FMaterial.DebugDraw;
                    psb.GenerateBendingConstraints(2, pm);
                    psb.Cfg.PIterations = 2;
                    psb.Cfg.Collisions = FCollisions.CLSS | FCollisions.CLRS;
                    psb.RandomizeConstraints();
                    psb.Scale(s);
                    Matrix m = Matrix.CreateFromYawPitchRoll(a.X, a.Y, a.Z) * Matrix.CreateTranslation(x);
                    psb.Transform(m);
                    psb.SetTotalMass(1000, true);
                    psb.GenerateClusters(64);
                    //psb.GenerateClusters(4);

                    psb.UserObject = "Soft";
                    World.AddSoftBody(psb);
                    CollisionShapes.Add(psb.CollisionShape);

                //}
        }

        #endregion

        SoftBody Create_SoftBox(Vector3 position, Vector3 scale)
        {
            Vector3 h = scale * 0.5f;
            Vector3[] c = new Vector3[]{
                h * new Vector3(-1,-1,-1),
		        h * new Vector3(+1,-1,-1),
		        h * new Vector3(-1,+1,-1),
		        h * new Vector3(+1,+1,-1),
		        h * new Vector3(-1,-1,+1),
		        h * new Vector3(+1,-1,+1),
		        h * new Vector3(-1,+1,+1),
		        h * new Vector3(+1,+1,+1)};
            SoftBody psb = SoftBodyHelpers.CreateFromConvexHull(SoftBodyWorldInfo, c);
            
            psb.GenerateBendingConstraints(2);
            psb.Translate(position);
            psb.UserObject = "Soft";
            SoftBodyHelpers.ReoptimizeLinkOrder(psb);
            psb.GenerateClusters(64);
            World.AddSoftBody(psb);
            CollisionShapes.Add(psb.CollisionShape);

            return (psb);
        }
    }
}
