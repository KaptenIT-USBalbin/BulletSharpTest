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

        public static FreeRollControl FreeRoll = new FreeRollControl();
        public static MotorControl MotorPlayerOne = new MotorControl();
        public static MotorControl MotorPlayerTwo = new MotorControl();
        SteerAndMotorControl SteerAndMotorPlayerOne = new SteerAndMotorControl(1, MotorPlayerOne);
        SteerAndMotorControl SteerAndMotorPlayerTwo = new SteerAndMotorControl(1, MotorPlayerTwo);
        public static MotorControl MotorL = new MotorControl();
        public static MotorControl MotorR = new MotorControl();
        SteerControl Steer = new SteerControl(1);

        CollisionDispatcher Dispatcher;
        BroadphaseInterface Broadphase;
        CollisionConfiguration CollisionConf;

        JoystickCapabilities joyCapabl;
        GamePadCapabilities gamePadCapabl;

        const int maxProxies = 32766;

        ///create 125 (5x5x5) dynamic objects
        int ArraySizeX = 0, ArraySizeY = 0, ArraySizeZ = 0;

        ///scaling of the objects (0.1 = 20 centimeter boxes )
        float StartPosX = Terrain.ChunkArea.Width / 2f + 500;
        float StartPosY = 0;
        float StartPosZ = Terrain.ChunkArea.Height / 2f + 500;

        public void Initialize(ContentManager Content, Effect effect, GraphicsDevice device)
        {
            gamePadCapabl = GamePad.GetCapabilities(0);


            Device = device;
            sphere = LoadModel("xwing", Content, effect);
            

            // collision configuration contains default setup for memory, collision setup
            CollisionConf = new SoftBodyRigidBodyCollisionConfiguration();
            Dispatcher = new CollisionDispatcher(CollisionConf);

            Broadphase = new AxisSweep3(new Vector3(-1000, -1000, -1000),
                new Vector3(1000, 1000, 1000), maxProxies);

            // the default constraint solver.
            Solver = new SequentialImpulseConstraintSolver();

            SoftBodyWorldInfo = new SoftBodyWorldInfo
            {
                AirDensity = .2f,
                WaterDensity = 0,
                WaterOffset = 0,
                WaterNormal = Vector3.Zero,
                Gravity = new Vector3(0, -20, 0),
                Dispatcher = Dispatcher,
                Broadphase = Broadphase
            };

            SoftBodyWorldInfo.SparseSdf.Initialize();

            World = new SoftRigidDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConf);
            World.Gravity = new Vector3(0, -20, 0);
            World.DispatchInfo.EnableSpu = true;

            InitializeObjects();
        }

        protected void InitializeObjects()
        {

            Init_Volvo(new Vector3(1000, 100, -400));
            //Init_SAAB(new Vector3(300, 200, -300));
            //Create_ClusterModel(sphere, new Vector3(250, 400, -300), Quaternion.Identity, Vector3.One / 5f, 1000f);
            Create_RigidModel(new Vector3(1000, 150, 400), Vector3.One, sphere, 10, "Model_Sphere");

            // create the ground
            CollisionShape groundShape = new BoxShape(50000, 1, 50000);
            LocalCreateRigidBody(0, Matrix.Identity, groundShape, "Ground");

            Create_Terrain();

            // create a few dynamic rigidbodies
            float mass = 1.0f;

#region Init cubes
            
             
            CollisionShape colShape = new BoxShape(1);

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
                        LocalCreateRigidBody(mass, startTransform, colShape, "Box"); 
                    }
                }
            }

#endregion
            
            //Create_SoftBox(new Vector3(0, 25f, 0), Vector3.One * 20f);
            //Create_SoftBox(new Vector3(0, 50f, 0), Vector3.One * 10f);

            for(int iteration = 0; iteration < 0; iteration++)
                Create_ClusterTorus(new Vector3(1000, 250 + iteration * 20, -1000), new Vector3(0, 3.131592f / 2f, 0), Vector3.One * 3, 10);
            
            
            //Create_ClusterModel(new Vector3(-10f, 5, 0), new Vector3(0, 0, 0), Vector3.One * 3);

            //Create_ClusterModel(Vector3.Zero, Vector3.One, Vector3.One);
        }

        public virtual void Update(float deltaTime)
        {
            KeyboardState keys = Keyboard.GetState();
            GamePadCapabilities capbl = GamePad.GetCapabilities(PlayerIndex.Two);
            GamePadState wheel = GamePad.GetState(PlayerIndex.Two);

            

            if (keys.IsKeyDown(Keys.Down))
            {
                MotorPlayerOne.MaxTorque = .2f;
                MotorPlayerOne.Goal = 10000;


                MotorL.Goal = 5000;
                MotorR.Goal = 5000;
            }
            else if (keys.IsKeyDown(Keys.Up))
            {
                MotorPlayerOne.MaxTorque = .2f;
                MotorPlayerOne.Goal = -10000;


                MotorL.Goal = -5000;
                MotorR.Goal = -5000;
            }
            else
            {
                MotorPlayerOne.MaxTorque = 0;
                MotorL.Goal = 0;
                MotorR.Goal = 0;
            }
                

            if (keys.IsKeyDown(Keys.Left))
            {
                SteerAndMotorPlayerOne.Angle -= deltaTime;
                SteerAndMotorPlayerOne.Angle -= deltaTime;
                Steer.Angle -= deltaTime;
                Steer.Angle -= deltaTime;

                MotorL.Goal += 5000;
                MotorR.Goal -= 5000;
            }
            else if (keys.IsKeyDown(Keys.Right))
            {
                SteerAndMotorPlayerOne.Angle += deltaTime;
                SteerAndMotorPlayerOne.Angle += deltaTime;
                Steer.Angle += deltaTime;
                Steer.Angle += deltaTime;

                MotorL.Goal -= 5000;
                MotorR.Goal += 5000;
            }

            ///


            if (keys.IsKeyDown(Keys.NumPad5))
            {
                MotorPlayerTwo.MaxTorque = .2f;
                MotorPlayerTwo.Goal = 10000;
            }
            else if (keys.IsKeyDown(Keys.NumPad8))
            {
                MotorPlayerTwo.MaxTorque = .2f;
                MotorPlayerTwo.Goal = -10000;
            }
            else
            {
                MotorPlayerTwo.MaxTorque = 0;
                MotorL.Goal = 0;
                MotorR.Goal = 0;
            }


            if (keys.IsKeyDown(Keys.NumPad4))
            {
                SteerAndMotorPlayerTwo.Angle -= deltaTime;
                SteerAndMotorPlayerTwo.Angle -= deltaTime;
                Steer.Angle -= deltaTime;
                Steer.Angle -= deltaTime;

            }
            else if (keys.IsKeyDown(Keys.NumPad6))
            {
                SteerAndMotorPlayerTwo.Angle += deltaTime;
                SteerAndMotorPlayerTwo.Angle += deltaTime;
                Steer.Angle += deltaTime;
                Steer.Angle += deltaTime;

            }

            ///

            if (keys.IsKeyUp(Keys.Up) && keys.IsKeyUp(Keys.Down) && keys.IsKeyUp(Keys.Left) && keys.IsKeyUp(Keys.Right))
            {
                MotorL.MaxTorque = 0;
                MotorR.MaxTorque = 0;
            }
            else
            {
                MotorL.MaxTorque = 0.17f;
                MotorR.MaxTorque = 0.17f;
            }
            SoftBodyWorldInfo.SparseSdf.GarbageCollect();
            World.StepSimulation(deltaTime * 3f, 10);

            
        }

        public void Draw(GraphicsDevice device, Effect effect, Matrix projectionMatrix, Matrix viewMatrix)
        {
            effect.CurrentTechnique = effect.Techniques["Monochromatic"];
            effect.Parameters["xEnableLighting"].SetValue(true);
            effect.Parameters["xColor"].SetValue(Color.CornflowerBlue.ToVector4());
            effect.Parameters["xAmbient"].SetValue(0.4f);
            effect.Parameters["xDiffuse"].SetValue(1.5f);
            effect.Parameters["xLightDirection"].SetValue(-new Vector3(1, 1, 0));
            effect.Parameters["xView"].SetValue(viewMatrix);
            effect.Parameters["xProjection"].SetValue(projectionMatrix);
            


            for(int i = 0; i < World.NumCollisionObjects; i++)
            {
                CollisionObject collObj = World.CollisionObjectArray[i];
                effect.Parameters["xWorld"].SetValue(collObj.WorldTransform);
                switch (collObj.CollisionShape.ShapeType)
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
                        
                        EffectPass pass = effect.CurrentTechnique.Passes[0];

                        pass.Apply();
                        device.DrawUserPrimitives(PrimitiveType.TriangleList, vertices, 0, vertexCount / 3, VertexPositionNormal.VertexDeclaration);
                        
                        break;
                

                    default:
                        switch (collObj.UserObject as string)
                        {
                            case "Model_Sphere":
                                DrawModel(sphere, collObj.WorldTransform, projectionMatrix, viewMatrix, effect);
                                break;
                        }
                        break;
                }
                

            }
        }

        /*void ModelDraw(Model model, Matrix world, Matrix projectionMatrix, Matrix viewMatrix, Effect effect)
        {
            Matrix[] xwingTransforms = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(xwingTransforms);
            foreach (ModelMesh mesh in model.Meshes)
            {
                foreach (BasicEffect currentEffect in mesh.Effects)
                {
                    //currentEffect.CurrentTechnique = currentEffect.Techniques["BasicEffect_VertexColor"];
                    currentEffect.CurrentTechnique = currentEffect.Techniques["Colored"];
                    currentEffect.World = xwingTransforms[mesh.ParentBone.Index] * world;
                    currentEffect.View = viewMatrix;
                    currentEffect.Projection = projectionMatrix;
                }
                mesh.Draw();
            }
        }*/

        Model LoadModel(string assetName, ContentManager content, Effect effect)
        {

            Model newModel = content.Load<Model>(assetName); foreach (ModelMesh mesh in newModel.Meshes)
                foreach (ModelMeshPart meshPart in mesh.MeshParts)
                    meshPart.Effect = effect.Clone();
            return newModel;
        }

        void DrawModel(Model model, Matrix world, Matrix projectionMatrix, Matrix viewMatrix, Effect effect)
        {

            Matrix[] transforms = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(transforms);
            foreach (ModelMesh mesh in model.Meshes)
            {
                foreach (Effect currentEffect in mesh.Effects)
                {
                    currentEffect.CurrentTechnique = currentEffect.Techniques["Colored"];
                    currentEffect.Parameters["xWorld"].SetValue(transforms[mesh.ParentBone.Index] * world);
                    currentEffect.Parameters["xView"].SetValue(viewMatrix);
                    currentEffect.Parameters["xProjection"].SetValue(projectionMatrix);
                }
                mesh.Draw();
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

            World.Dispose();
            Broadphase.Dispose();
            if (Dispatcher != null)
            {
                Dispatcher.Dispose();
            }
            CollisionConf.Dispose();
        }

        public RigidBody LocalCreateRigidBody(float mass, Matrix startTransform, CollisionShape shape, string UserObject)
        {
            bool isDynamic = (mass != 0.0f);

            Vector3 localInertia = Vector3.Zero;
            if (isDynamic)
                shape.CalculateLocalInertia(mass, out localInertia);

            DefaultMotionState myMotionState = new DefaultMotionState(startTransform);

            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia);
            RigidBody body = new RigidBody(rbInfo);
            body.UserObject = UserObject;

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
            SoftBody wheelFL = Create_ClusterTorus(wheels[0], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(2, 4, 2), 10);
            SoftBody wheelFR = Create_ClusterTorus(wheels[1], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(2, 4, 2), 10);
            SoftBody wheelRL = Create_ClusterTorus(wheels[2], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(2, 5, 2), 10);
            SoftBody wheelRR = Create_ClusterTorus(wheels[3], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(2, 5, 2), 10);

            wheelFL.Cfg.DF =
                wheelFR.Cfg.DF =
                wheelRL.Cfg.DF =
                wheelRR.Cfg.DF = .1f;

            LJoint.Specs lspecs = new LJoint.Specs();
            lspecs.Cfm = 1;
            lspecs.Erp = 1;
            lspecs.Position = Vector3.Zero;

            lspecs.Position = wheels[0]; body.AppendLinearJoint(lspecs, wheelFL);
            lspecs.Position = wheels[1]; body.AppendLinearJoint(lspecs, wheelFR);
            lspecs.Position = wheels[2]; body.AppendLinearJoint(lspecs, wheelRL);
            lspecs.Position = wheels[3]; body.AppendLinearJoint(lspecs, wheelRR);

            //"rotations-servo-liknande" som ska användas för att driva och svänga hjulen
            AJoint.Specs aspecs = new AJoint.Specs();
            aspecs.Cfm = 1;
            aspecs.Erp = 1;
            //Huvud-rotationens axel är liggande(likt hjulaxeln) axeln är riktad likt x-axeln enl nedan
            aspecs.Axis = new Vector3(1, 0, 0);


            aspecs.Control = SteerAndMotorPlayerOne;//Binder en kontroll för "rotations-servo-liknande" - saken(denna fungerar både för styrning och drivning)
            body.AppendAngularJoint(aspecs, wheelFL);//Fäster framhjulen m.h.a. denne
            body.AppendAngularJoint(aspecs, wheelFR);

            aspecs.Control = MotorPlayerOne;////Binder en kontroll för "rotations-servo-liknande" - saken(denna fungerar bara för drivning)
            body.AppendAngularJoint(aspecs, wheelRL);//Fäster bakhjulen m.h.a. denne
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
                wheelRR.Clusters[0].Matching = 0.005f;//Däckens mjukhet
            wheelFL.Clusters[0].NodeDamping =
                wheelFR.Clusters[0].NodeDamping =
                wheelRL.Clusters[0].NodeDamping =
                wheelRR.Clusters[0].NodeDamping = 0.005f;//Däckens återdämpning
            
            //autocam=true;
        }



        void Init_Volvo(Vector3 origin)
        {
            Quaternion orientation = Quaternion.CreateFromYawPitchRoll(-(float)Math.PI / 2, 0, 0);
            const float bodyMass = 15;
            const float widthf = 40;
            const float widthr = 40;
            const float length = 75;
            const float height = 5;


            const float wheelOffsetX = 5f;
            const float wheelOffsetY = height;
            const float wheelOffsetZ = -5f;

            const float wheelRadius = 2.5f;
            const float wheelWidth = 7f;
            const float wheelMass = 1;

            const float suspensionStrength = .5f;

            Vector3[] wheels = new Vector3[] {
		        new Vector3(+widthf / 2f + wheelOffsetX,-wheelOffsetY / 2f,+length / 2f + wheelOffsetZ), // Front left
		        new Vector3(-widthf / 2f - wheelOffsetX,-wheelOffsetY / 2f,+length / 2f + wheelOffsetZ), // Front right
                new Vector3(+widthf / 2f + wheelOffsetX,-wheelOffsetY / 2f, -length / 2f - wheelOffsetZ + 8 * wheelRadius),
		        new Vector3(-widthf / 2f - wheelOffsetX,-wheelOffsetY / 2f, -length / 2f - wheelOffsetZ + 8 * wheelRadius),
		        new Vector3(+widthr / 2f + wheelOffsetX,-wheelOffsetY / 2f,-length / 2f - wheelOffsetZ), // Rear left
		        new Vector3(-widthr / 2f - wheelOffsetX,-wheelOffsetY / 2f,-length / 2f - wheelOffsetZ), // Rear right
	        };

            SoftBody body = Create_SoftBox(Vector3.Zero, new Vector3(widthr, height, length), bodyMass);
            SoftBody wheelFL = Create_ClusterTorus(wheels[0], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);
            SoftBody wheelFR = Create_ClusterTorus(wheels[1], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);
            //SoftBody wheelCL = Create_ClusterTorus(wheels[2], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);
            //SoftBody wheelCR = Create_ClusterTorus(wheels[3], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);

            SoftBody wheelRL = Create_ClusterTorus(wheels[4], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);
            SoftBody wheelRR = Create_ClusterTorus(wheels[5], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);

            wheelFL.Cfg.DF =
                wheelFR.Cfg.DF =
                //wheelCL.Cfg.DF =
                //wheelCR.Cfg.DF =
                wheelRL.Cfg.DF =
                wheelRR.Cfg.DF = .1f;

            LJoint.Specs lspecs = new LJoint.Specs();
            lspecs.Cfm = suspensionStrength;
            lspecs.Erp = .5f;
            lspecs.Position = Vector3.Zero;

            lspecs.Position = wheels[0]; body.AppendLinearJoint(lspecs, wheelFL);
            lspecs.Position = wheels[1]; body.AppendLinearJoint(lspecs, wheelFR);
            //lspecs.Position = wheels[2]; body.AppendLinearJoint(lspecs, wheelCL);
            //lspecs.Position = wheels[3]; body.AppendLinearJoint(lspecs, wheelCR);
            lspecs.Position = wheels[4]; body.AppendLinearJoint(lspecs, wheelRL);
            lspecs.Position = wheels[5]; body.AppendLinearJoint(lspecs, wheelRR);

            //"rotations-servo-liknande" som ska användas för att driva och svänga hjulen
            AJoint.Specs aspecs = new AJoint.Specs();
            aspecs.Cfm = 1;
            aspecs.Erp = 1;
            //Huvud-rotationens axel är liggande(likt hjulaxeln) axeln är riktad likt x-axeln enl nedan
            aspecs.Axis = new Vector3(1, 0, 0);


            aspecs.Control = SteerAndMotorPlayerOne;//Binder en kontroll för "rotations-servo-liknande" - saken(denna fungerar både för styrning och drivning)
            //Fäster framhjulen m.h.a. denne
            body.AppendAngularJoint(aspecs, wheelFL);//Fäster bakhjulen m.h.a. denne
            body.AppendAngularJoint(aspecs, wheelFR);

            aspecs.Control = Steer;//Binder en kontroll för "rotations-servo-liknande" - saken(denna bara fungerar för styrning)

            

            aspecs.Control = MotorPlayerOne;
            //body.AppendAngularJoint(aspecs, wheelCR);
            body.AppendAngularJoint(aspecs, wheelRR);
            //body.AppendAngularJoint(aspecs, wheelCL);
            body.AppendAngularJoint(aspecs, wheelRL);

            aspecs.Control = FreeRoll;


            aspecs.Control = MotorL;//Binder en kontroll för "rotations-servo-liknande" - saken(denna fungerar bara för drivning av ena sidan)


            aspecs.Control = MotorR;//Binder en kontroll för "rotations-servo-liknande" - saken(denna fungerar bara för drivning av andra sidan)

          


            body.Rotate(orientation);
            wheelFL.Rotate(orientation);
            wheelFR.Rotate(orientation);
            //wheelCL.Rotate(orientation);
            //wheelCR.Rotate(orientation);
            wheelRL.Rotate(orientation);
            wheelRR.Rotate(orientation);
            body.Translate(origin);
            wheelFL.Translate(origin);
            wheelFR.Translate(origin);
            //wheelCL.Translate(origin);
            //wheelCR.Translate(origin);
            wheelRL.Translate(origin);
            wheelRR.Translate(origin);
            wheelFL.Cfg.PIterations =
                wheelFR.Cfg.PIterations =
                //wheelCL.Cfg.PIterations =
                //wheelCR.Cfg.PIterations =
                wheelRL.Cfg.PIterations =
                wheelRR.Cfg.PIterations = 1;
            wheelFL.Clusters[0].Matching =
                wheelFR.Clusters[0].Matching =
                //wheelCL.Clusters[0].Matching =
                //wheelCR.Clusters[0].Matching =
                wheelRL.Clusters[0].Matching =
                wheelRR.Clusters[0].Matching = 0.5f;//Däckens hårdhet
            wheelFL.Clusters[0].NodeDamping =
                wheelFR.Clusters[0].NodeDamping =
                //wheelCL.Clusters[0].NodeDamping =
                //wheelCR.Clusters[0].NodeDamping =
                wheelRL.Clusters[0].NodeDamping =
                wheelRR.Clusters[0].NodeDamping = 0.05f;//Däckens återdämpning
            wheelFL.Friction = 
                wheelFR.Friction =
                //wheelCL.Friction =
                //wheelCR.Friction =
                wheelRL.Friction =
                wheelRR.Friction = 1f;//Däckens friktion

            //autocam=true;
        }


        void Init_SAAB(Vector3 origin)
        {
            Quaternion orientation = Quaternion.CreateFromYawPitchRoll(-(float)Math.PI / 2, 0, 0);
            const float bodyMass = 10000;
            const float widthf = 20;
            const float widthr = 20;
            const float length = 75;
            const float height = 5f;
            const float wheelOffsetX = 5f;
            const float wheelOffsetZ = -5f;

            const float wheelRadius = 2.5f;
            const float wheelWidth = 7f;
            const float wheelMass = 1;

            Vector3[] wheels = new Vector3[] {
		        new Vector3(+widthf / 2f + wheelOffsetX,-height / 2f,+length / 2f + wheelOffsetZ), // Front left
		        new Vector3(-widthf / 2f - wheelOffsetX,-height / 2f,+length / 2f + wheelOffsetZ), // Front right
                new Vector3(+widthf / 2f + wheelOffsetX,-height / 2f, -length / 2f - wheelOffsetZ + 8 * wheelRadius),
		        new Vector3(-widthf / 2f - wheelOffsetX,-height / 2f, -length / 2f - wheelOffsetZ + 8 * wheelRadius),
		        new Vector3(+widthr / 2f + wheelOffsetX,-height / 2f,-length / 2f - wheelOffsetZ), // Rear left
		        new Vector3(-widthr / 2f - wheelOffsetX,-height / 2f,-length / 2f - wheelOffsetZ), // Rear right
	        };

            SoftBody body = Create_SoftBox(Vector3.Zero, new Vector3(widthr, 5f, length), bodyMass);
            SoftBody wheelFL = Create_ClusterTorus(wheels[0], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);
            SoftBody wheelFR = Create_ClusterTorus(wheels[1], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);
            //SoftBody wheelCL = Create_ClusterTorus(wheels[2], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);
            //SoftBody wheelCR = Create_ClusterTorus(wheels[3], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);

            SoftBody wheelRL = Create_ClusterTorus(wheels[4], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);
            SoftBody wheelRR = Create_ClusterTorus(wheels[5], new Vector3(0, 0, (float)Math.PI / 2), new Vector3(wheelRadius, wheelWidth, wheelRadius), wheelMass);

            wheelFL.Cfg.DF =
                wheelFR.Cfg.DF =
                //wheelCL.Cfg.DF =
                //wheelCR.Cfg.DF =
                wheelRL.Cfg.DF =
                wheelRR.Cfg.DF = .1f;

            LJoint.Specs lspecs = new LJoint.Specs();
            lspecs.Cfm = 0.1f;
            lspecs.Erp = 0.1f;
            lspecs.Position = Vector3.Zero;

            lspecs.Position = wheels[0]; body.AppendLinearJoint(lspecs, wheelFL);
            lspecs.Position = wheels[1]; body.AppendLinearJoint(lspecs, wheelFR);
            //lspecs.Position = wheels[2]; body.AppendLinearJoint(lspecs, wheelCL);
            //lspecs.Position = wheels[3]; body.AppendLinearJoint(lspecs, wheelCR);
            lspecs.Position = wheels[4]; body.AppendLinearJoint(lspecs, wheelRL);
            lspecs.Position = wheels[5]; body.AppendLinearJoint(lspecs, wheelRR);

            //"rotations-servo-liknande" som ska användas för att driva och svänga hjulen
            AJoint.Specs aspecs = new AJoint.Specs();
            aspecs.Cfm = 1;
            aspecs.Erp = 1;
            //Huvud-rotationens axel är liggande(likt hjulaxeln) axeln är riktad likt x-axeln enl nedan
            aspecs.Axis = new Vector3(1, 0, 0);


            aspecs.Control = SteerAndMotorPlayerTwo;//Binder en kontroll för "rotations-servo-liknande" - saken(denna fungerar både för styrning och drivning)
            //Fäster framhjulen m.h.a. denne
            

            aspecs.Control = Steer;//Binder en kontroll för "rotations-servo-liknande" - saken(denna bara fungerar för styrning)

            body.AppendAngularJoint(aspecs, wheelFL);//Fäster bakhjulen m.h.a. denne
            body.AppendAngularJoint(aspecs, wheelFR);

            aspecs.Control = MotorPlayerTwo;

            //body.AppendAngularJoint(aspecs, wheelCR);
            body.AppendAngularJoint(aspecs, wheelRR);
            //body.AppendAngularJoint(aspecs, wheelCL);
            body.AppendAngularJoint(aspecs, wheelRL);

            //aspecs.Control = new FreeRollControl();

            

            aspecs.Control = MotorL;//Binder en kontroll för "rotations-servo-liknande" - saken(denna fungerar bara för drivning av ena sidan)


            aspecs.Control = MotorR;//Binder en kontroll för "rotations-servo-liknande" - saken(denna fungerar bara för drivning av andra sidan)




            body.Rotate(orientation);
            wheelFL.Rotate(orientation);
            wheelFR.Rotate(orientation);
            //wheelCL.Rotate(orientation);
            //wheelCR.Rotate(orientation);
            wheelRL.Rotate(orientation);
            wheelRR.Rotate(orientation);
            body.Translate(origin);
            wheelFL.Translate(origin);
            wheelFR.Translate(origin);
            //wheelCL.Translate(origin);
            //wheelCR.Translate(origin);
            wheelRL.Translate(origin);
            wheelRR.Translate(origin);
            wheelFL.Cfg.PIterations =
                wheelFR.Cfg.PIterations =
                //wheelCL.Cfg.PIterations =
                //wheelCR.Cfg.PIterations =
                wheelRL.Cfg.PIterations =
                wheelRR.Cfg.PIterations = 1;
            wheelFL.Clusters[0].Matching =
                wheelFR.Clusters[0].Matching =
                //wheelCL.Clusters[0].Matching =
                //wheelCR.Clusters[0].Matching =
                wheelRL.Clusters[0].Matching =
                wheelRR.Clusters[0].Matching = 0.10f;//Däckens mjukhet
            wheelFL.Clusters[0].NodeDamping =
                wheelFR.Clusters[0].NodeDamping =
                //wheelCL.Clusters[0].NodeDamping =
                //wheelCR.Clusters[0].NodeDamping =
                wheelRL.Clusters[0].NodeDamping =
                wheelRR.Clusters[0].NodeDamping = 0.05f;//Däckens återdämpning
            wheelFL.Friction =
                wheelFR.Friction =
                //wheelCL.Friction =
                //wheelCR.Friction =
                wheelRL.Friction =
                wheelRR.Friction = 0f;//Däckens friktion

            //autocam=true;
        }


        RigidBody Create_Terrain()
        {
            TriangleIndexVertexArray vertexArray = new TriangleIndexVertexArray(Terrain.ReducedIndices, Terrain.Vectors);

            //vertexArray.Scaling = Vector3.Transform(Vector3.One, Terrain.Translation);

            CollisionShape btTerrain = new BvhTriangleMeshShape(vertexArray, true, true);

            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(0, new DefaultMotionState(), btTerrain ,Vector3.Zero);

            RigidBody body = new RigidBody(rbInfo);

            //body.CollisionFlags = CollisionFlags.None;
            World.AddRigidBody(body);

            return body;
        }

        RigidBody Create_RigidModel(Vector3 pos, Vector3 scale, Model model, float mass, string objectName)
        {
            List<float> vertices = new List<float>();
            List<int> indices = new List<int>();

            VertexExtractor.ExtractTrianglesFrom(model, vertices, indices, Matrix.CreateScale(scale));

            TriangleIndexVertexArray vertexArray = new TriangleIndexVertexArray(indices.ToArray(), vertices.ToArray());

            CollisionShape shape = new BvhTriangleMeshShape(vertexArray, true, true);
            Vector3 localInertia = Vector3.Zero;
            shape.CalculateLocalInertia(mass, out localInertia);
            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, new DefaultMotionState(), shape, localInertia);

            RigidBody body = new RigidBody(rbInfo);
            body.Translate(pos);

            //body.WorldTransform = Matrix.CreateScale(1f / 1f);
            body.UserObject = objectName;

            World.AddRigidBody(body);

            return body;
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

        SoftBody Create_ClusterTorus(Vector3 x, Vector3 a, Vector3 s, float mass)
        {
            SoftBody psb = SoftBodyHelpers.CreateFromTriMesh(SoftBodyWorldInfo, TorusMesh.Vertices, TorusMesh.Indices);
            Material pm = psb.AppendMaterial();
            pm.Lst = 0.01f;
            pm.Flags -= FMaterial.DebugDraw;
            psb.GenerateBendingConstraints(2, pm);
            psb.Cfg.PIterations = 2;
            psb.Cfg.Collisions = FCollisions.CLSS | FCollisions.CLRS | FCollisions.CLSelf;
            psb.RandomizeConstraints();
            psb.Scale(s);
            Matrix m = Matrix.CreateFromYawPitchRoll(a.X, a.Y, a.Z) * Matrix.CreateTranslation(x);
            psb.Transform(m);
            psb.SetTotalMass(mass, true);
            psb.GenerateClusters(32);
            //psb.GenerateClusters(4);
            
            psb.UserObject = "Soft";
            World.AddSoftBody(psb);

            return (psb);
        }


        void Create_ClusterModel(Model model, Vector3 pos, Quaternion rot, Vector3 scale, float mass)
        {
            //foreach(ModelMesh mesh in sphere.Meshes)
                //foreach (ModelMeshPart part in mesh.MeshParts)
                //{

            List<float> vertices = new List<float>();
            //List<Vector3> vertices = new List<Vector3>();
            List<int> indices = new List<int>();

            VertexExtractor.ExtractTrianglesFrom(model, vertices, indices, Matrix.Identity);
                
                //Model modelToUse, List<Vector3> vertices, List<int> indices, Matrix worldPosition)
            float[] verticesArray = vertices.ToArray();
            int[] indicesArray = indices.ToArray();


            SoftBody psb = SoftBodyHelpers.CreateFromTriMesh(SoftBodyWorldInfo, verticesArray, indicesArray);
            Material pm = psb.AppendMaterial();
            pm.Lst = 1;
            pm.Flags -= FMaterial.DebugDraw;
            psb.GenerateBendingConstraints(2, pm);
            psb.Cfg.PIterations = 2;
            psb.Cfg.DF = 1;
            psb.Cfg.Collisions = FCollisions.CLSS | FCollisions.CLRS;
            psb.RandomizeConstraints();
            Matrix m = Matrix.CreateFromQuaternion(rot) * Matrix.CreateTranslation(pos);
            psb.Transform(m);
            psb.Scale(scale);
            psb.SetTotalMass(150, true);
            psb.GenerateClusters(1);
            psb.UserObject = "Soft";
            World.AddSoftBody(psb);

                //}
        }

        #endregion

        SoftBody Create_SoftBox(Vector3 position, Vector3 scale, float mass)
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

            Material pm = psb.AppendMaterial();
            pm.Lst = 1;
            pm.Flags -= FMaterial.DebugDraw;
            psb.GenerateBendingConstraints(2, pm);
            psb.Cfg.PIterations = 2;
            psb.Cfg.DF = 1;
            psb.Cfg.Collisions = FCollisions.CLSS | FCollisions.CLRS;
            psb.RandomizeConstraints();
            Matrix m = Matrix.CreateTranslation(position);
            psb.Transform(m);
            psb.SetTotalMass(mass, true);
            psb.GenerateClusters(1);
            psb.UserObject = "Soft";
            World.AddSoftBody(psb);

            return (psb);
        }
    }
}
