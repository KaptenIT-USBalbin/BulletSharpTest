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
                Matrix.CreatePerspectiveFieldOfView(MathHelper.PiOver4, device.Viewport.AspectRatio, 1.0f, 200.0f);
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

            DebugDrawer = new Physics.PhysicsDebugDraw(graphics.GraphicsDevice, DebugEffect);
            physics.Initialize(Content, device);
            physics.World.DebugDrawer = DebugDrawer;

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

            box = VertexHelper.CreateBox(device, new Vector3(1, 1, 1));
            groundBox = VertexHelper.CreateBox(device, new Vector3(50, 1, 50));
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

            viewMatrix = Matrix.CreateLookAt(eye, target, Vector3.UnitY);

            physics.Update((float)gameTime.ElapsedGameTime.TotalSeconds);

            base.Update(gameTime);
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            DebugEffect.View = viewMatrix;
            
            // Debug draw
            DebugEffect.VertexColorEnabled = true;
            DebugEffect.LightingEnabled = true;

            DebugEffect.World = Matrix.Identity;
            DebugEffect.CurrentTechnique.Passes[0].Apply();
            DebugDrawer.DrawDebugWorld(physics.World);


            // Draw shapes
            Effect.CurrentTechnique = Effect.Techniques["Monochromatic"];
            Effect.Parameters["xEnableLighting"].SetValue(true);
            Effect.Parameters["xAmbient"].SetValue(0.2f);
            Effect.Parameters["xLightDirection"].SetValue(-Vector3.One);
            Effect.Parameters["xView"].SetValue(viewMatrix);
            Effect.Parameters["xProjection"].SetValue(projectionMatrix);
            

            physics.Draw(device, Effect);

            foreach (CollisionObject colObj in physics.World.CollisionObjectArray)
            {
                if ("Soft".Equals(colObj.UserObject))
                    continue;

                RigidBody body = RigidBody.Upcast(colObj);
                Effect.Parameters["xWorld"].SetValue(body.MotionState.WorldTransform);

                if ("Ground".Equals(colObj.UserObject))
                {
                    Effect.Parameters["xColor"].SetValue(groundColor.ToVector3());
                    Effect.CurrentTechnique.Passes[0].Apply();
                    VertexHelper.DrawBox(device, groundBox);
                    continue;
                }

                if (colObj.ActivationState == ActivationState.ActiveTag)
                    Effect.Parameters["xColor"].SetValue(activeColor.ToVector3());
                else
                    Effect.Parameters["xColor"].SetValue(passiveColor.ToVector3());

                if ("Model".Equals(colObj.UserObject))
                {
                    Effect.Parameters["xWorld"].SetValue(Matrix.CreateScale(10) * body.MotionState.WorldTransform);
                    Effect.Parameters["xColor"].SetValue(groundColor.ToVector3());
                    Effect.CurrentTechnique.Passes[0].Apply();
                    //VertexHelper.DrawBall(device, Effect, Ball);
                    continue;
                }

                Effect.CurrentTechnique.Passes[0].Apply();
                VertexHelper.DrawBox(device, box);
            }

            base.Draw(gameTime);
        }
    }
}
