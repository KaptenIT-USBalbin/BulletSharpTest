using System;
using BulletSharp;
using BulletSharp.SoftBody;
using BulletSharp.Serialize;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace BulletTest
{
    /// <summary>
    /// This is the main type for your game.
    /// </summary>
    public class BulletTest : Game
    {
        MeshFactory _meshFactory;

        Terrain Terrain = new Terrain();

        Color activeColor = Color.Orange;
        Color passiveColor = Color.DarkOrange;
        Color groundColor = Color.Green;

        Vector3 eye = new Vector3(30, 30, 30);
        Vector3 target = new Vector3(0, 5, 0);

        GraphicsDeviceManager graphics;
        GraphicsDevice device;
        BasicEffect DebugEffect;
        Effect Effect;

        Physics physics;
        Physics.PhysicsDebugDraw DebugDrawer;
        bool IsDebugDrawEnabled;
        bool f3KeyPressed;

        Matrix projectionMatrix;
        Matrix viewMatrix;
        VertexBuffer groundBox, box;
        Model Ball;

        Quaternion rotation;
        float yaw = 0;
        float pitch = 0;

        Texture2D Texture;

        Vector3 lastPos;
        Vector3 cameraPos = new Vector3();


        public BulletTest()
        {
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";

            Window.Title = "BulletSharp - MonoGame Basic Demo";
            Window.AllowUserResizing = true;
            Window.ClientSizeChanged += Window_ClientSizeChanged;
        }

        void Window_ClientSizeChanged(object sender, EventArgs e)
        {
            DebugEffect.Projection = 
                projectionMatrix = 
                Matrix.CreatePerspectiveFieldOfView(MathHelper.PiOver4, device.Viewport.AspectRatio, 1.0f, 10000);
        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            physics = new Physics();

            Terrain.Initialize(Content, GraphicsDevice);

            DebugDrawer = new Physics.PhysicsDebugDraw(graphics.GraphicsDevice, DebugEffect);
            

            base.Initialize();
        }

        protected override void EndRun()
        {
            physics.ExitPhysics();
            base.EndRun();
        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            device = graphics.GraphicsDevice;
            

            #region DebugEffect
            DebugEffect = new BasicEffect(device);

            DebugEffect.Projection = projectionMatrix;

            // Set light
            DebugEffect.AmbientLightColor = Color.Gray.ToVector3();
            DebugEffect.DirectionalLight0.Enabled = true;
            DebugEffect.DirectionalLight0.DiffuseColor = Color.LemonChiffon.ToVector3();
            #endregion

            Effect = Content.Load<Effect>("neweffects");
            Texture = Content.Load<Texture2D>("texture");

            box = VertexHelper.CreateBox(device, new Vector3(1, 1, 1));
            groundBox = VertexHelper.CreateTexturedBox(device, new Vector3(50000, 1, 50000), 100);

            physics.Initialize(Content, Effect, device);
            physics.World.DebugDrawer = DebugDrawer;
        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// game-specific content.
        /// </summary>
        protected override void UnloadContent()
        {
            // TODO: Unload any non ContentManager content here
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            KeyboardState ns = Keyboard.GetState();
            if (ns.IsKeyDown(Keys.Escape) || ns.IsKeyDown(Keys.Q))
            {
                Exit();
            }

            // Toggle debug
            if (ns.IsKeyDown(Keys.F3))
            {
                if (f3KeyPressed == false)
                {
                    f3KeyPressed = true;
                    if (IsDebugDrawEnabled == false)
                    {
                        DebugDrawer.DebugMode = DebugDrawModes.DrawAabb;
                        IsDebugDrawEnabled = true;
                    }
                    else
                    {
                        DebugDrawer.DebugMode = DebugDrawModes.None;
                        IsDebugDrawEnabled = false;
                    }
                }
            }
            if (f3KeyPressed == true)
            {
                if (ns.IsKeyUp(Keys.F3))
                    f3KeyPressed = false;
            }

            if (ns.IsKeyDown(Keys.A))
                yaw -= 0.05f;

            if (ns.IsKeyDown(Keys.D))
                yaw += 0.05f;

            if (ns.IsKeyDown(Keys.W))
                pitch -= 0.05f;

            if (ns.IsKeyDown(Keys.S))
                pitch += 0.05f;

            physics.Update((float)gameTime.ElapsedGameTime.TotalSeconds);


            Vector3 pos = new Vector3();

            Vector3[] vectors;
            if (physics.World.CollisionObjectArray[0].CollisionShape.IsSoftBody)
            {
                (physics.World.CollisionObjectArray[0] as SoftBody).GetVertexNormalData(out vectors);
                pos = vectors[0];
            }
            else
            {
                pos = physics.World.CollisionObjectArray[0].WorldTransform.Translation;
            }//pos = (physics.World.CollisionObjectArray[0] as SoftBody).Joints[0].Bodies[0].CollisionObject.WorldTransform.Translation;

            /*foreach(Vector3 vector in vectors)
                pos += vector;

            pos *= 1f/(float)vectors.Length;
            */
            

            rotation = Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), yaw) * Quaternion.CreateFromAxisAngle(new Vector3(1,0,0), pitch);

            cameraPos = new Vector3(0, 20, 200);            
            cameraPos = Vector3.Transform(cameraPos, Matrix.CreateFromQuaternion(rotation));
            cameraPos += pos;




            /*float lerp = 0.025f;

            cameraPos += (pos - cameraPos) * lerp;
            cameraPos += new Vector3(0, 1, 0);
            cameraPos = Vector3.Transform(cameraPos, Matrix.CreateFromQuaternion(rotation));
            */

            viewMatrix = Matrix.CreateLookAt(cameraPos, pos, Vector3.UnitY);

            //lastPos = vectors[0];


            base.Update(gameTime);
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.RasterizerState = RasterizerState.CullNone;
            GraphicsDevice.Clear(ClearOptions.Target | ClearOptions.DepthBuffer | ClearOptions.Stencil, Color.Black, 1.0f, 0);

            DebugEffect.View = viewMatrix;
            
            // Debug draw
            DebugEffect.VertexColorEnabled = true;
            DebugEffect.LightingEnabled = true;

            DebugEffect.World = Matrix.Identity;
            DebugEffect.CurrentTechnique.Passes[0].Apply();
            DebugDrawer.DrawDebugWorld(physics.World);


            Terrain.Draw(Effect, projectionMatrix, viewMatrix, Texture);
            

            physics.Draw(device, Effect, projectionMatrix, viewMatrix);

            // Draw shapes
            Effect.CurrentTechnique = Effect.Techniques["Monochromatic"];
            Effect.Parameters["xEnableLighting"].SetValue(true);
            Effect.Parameters["xAmbient"].SetValue(0.4f);
            Effect.Parameters["xDiffuse"].SetValue(1.5f);
            Effect.Parameters["xLightDirection"].SetValue(-new Vector3(1, 1, 0));
            Effect.Parameters["xView"].SetValue(viewMatrix);
            Effect.Parameters["xProjection"].SetValue(projectionMatrix);

            foreach (CollisionObject colObj in physics.World.CollisionObjectArray)
            {
                if ("Soft".Equals(colObj.UserObject) || colObj.CollisionShape.ShapeType == BroadphaseNativeType.SoftBodyShape)
                    continue;
                if(colObj.UserObject != null && (colObj.UserObject as string).StartsWith("Model_"))
                    continue;

                RigidBody body = RigidBody.Upcast(colObj);
                Effect.Parameters["xWorld"].SetValue(body.MotionState.WorldTransform);


                if ("Ground".Equals(colObj.UserObject))
                {
                    Effect.CurrentTechnique = Effect.Techniques["Textured"];
                    Effect.Parameters["xEnableLighting"].SetValue(true);
                    Effect.Parameters["xAmbient"].SetValue(0.4f);
                    Effect.Parameters["xDiffuse"].SetValue(1.5f);
                    Effect.Parameters["xLightDirection"].SetValue(-new Vector3(1, 1, 0));
                    Effect.Parameters["xView"].SetValue(viewMatrix);
                    Effect.Parameters["xProjection"].SetValue(projectionMatrix);

                    Effect.Parameters["xTexture"].SetValue(Texture);

                    Effect.CurrentTechnique.Passes[0].Apply();

                    
                    VertexHelper.DrawBox(device, groundBox);
                    continue;
                }

                if (colObj.ActivationState == ActivationState.ActiveTag)
                    Effect.Parameters["xColor"].SetValue(activeColor.ToVector4());
                else
                    Effect.Parameters["xColor"].SetValue(passiveColor.ToVector4());


                VertexHelper.DrawBox(device, box);
                Effect.CurrentTechnique.Passes[0].Apply();
                
                
            }

            base.Draw(gameTime);
        }
    }
}
