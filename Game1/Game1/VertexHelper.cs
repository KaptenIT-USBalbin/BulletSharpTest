// Adapted from http://www.switchonthecode.com/tutorials/creating-a-textured-box-in-xna

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace BulletTest
{
    public struct VertexPositionNormal
    {
        public Vector3 Position;
        public Vector3 Normal;

        public VertexPositionNormal(Vector3 position, Vector3 normal)
        {
            Position = position;
            Normal = normal;
        }

        public readonly static VertexDeclaration VertexDeclaration = new VertexDeclaration
        (
            new VertexElement(0, VertexElementFormat.Vector3, VertexElementUsage.Position, 0),
            new VertexElement(12, VertexElementFormat.Vector3, VertexElementUsage.Normal, 0)
        );

        /// <summary>
        /// Size of this vertex type.
        /// </summary>
        public const int SizeInBytes = 24;
    }

    class VertexHelper
    {
        static public VertexBuffer CreateBox(GraphicsDevice device, Vector3 size)
        {
            VertexPositionNormal[] vertices = new VertexPositionNormal[36];

            Vector3 topLeftFront = new Vector3(-1.0f, 1.0f, -1.0f) * size;
            Vector3 bottomLeftFront = new Vector3(-1.0f, -1.0f, -1.0f) * size;
            Vector3 topRightFront = new Vector3(1.0f, 1.0f, -1.0f) * size;
            Vector3 bottomRightFront = new Vector3(1.0f, -1.0f, -1.0f) * size;
            Vector3 topLeftBack = new Vector3(-1.0f, 1.0f, 1.0f) * size;
            Vector3 topRightBack = new Vector3(1.0f, 1.0f, 1.0f) * size;
            Vector3 bottomLeftBack = new Vector3(-1.0f, -1.0f, 1.0f) * size;
            Vector3 bottomRightBack = new Vector3(1.0f, -1.0f, 1.0f) * size;

            Vector3 frontNormal = new Vector3(0.0f, 0.0f, 1.0f) * size;
            Vector3 backNormal = new Vector3(0.0f, 0.0f, -1.0f) * size;
            Vector3 topNormal = new Vector3(0.0f, 1.0f, 0.0f) * size;
            Vector3 bottomNormal = new Vector3(0.0f, -1.0f, 0.0f) * size;
            Vector3 leftNormal = new Vector3(-1.0f, 0.0f, 0.0f) * size;
            Vector3 rightNormal = new Vector3(1.0f, 0.0f, 0.0f) * size;

            // Front face.
            vertices[2] = new VertexPositionNormal(topLeftFront, frontNormal);
            vertices[1] = new VertexPositionNormal(bottomLeftFront, frontNormal);
            vertices[0] = new VertexPositionNormal(topRightFront, frontNormal);
            vertices[5] = new VertexPositionNormal(bottomLeftFront, frontNormal);
            vertices[4] = new VertexPositionNormal(bottomRightFront, frontNormal);
            vertices[3] = new VertexPositionNormal(topRightFront, frontNormal);

            // Back face.
            vertices[8] = new VertexPositionNormal(topLeftBack, backNormal);
            vertices[7] = new VertexPositionNormal(topRightBack, backNormal);
            vertices[6] = new VertexPositionNormal(bottomLeftBack, backNormal);
            vertices[11] = new VertexPositionNormal(bottomLeftBack, backNormal);
            vertices[10] = new VertexPositionNormal(topRightBack, backNormal);
            vertices[9] = new VertexPositionNormal(bottomRightBack, backNormal);

            // Top face.
            vertices[14] = new VertexPositionNormal(topLeftFront, topNormal);
            vertices[13] = new VertexPositionNormal(topRightBack, topNormal);
            vertices[12] = new VertexPositionNormal(topLeftBack, topNormal);
            vertices[17] = new VertexPositionNormal(topLeftFront, topNormal);
            vertices[16] = new VertexPositionNormal(topRightFront, topNormal);
            vertices[15] = new VertexPositionNormal(topRightBack, topNormal);

            // Bottom face. 
            vertices[20] = new VertexPositionNormal(bottomLeftFront, bottomNormal);
            vertices[19] = new VertexPositionNormal(bottomLeftBack, bottomNormal);
            vertices[18] = new VertexPositionNormal(bottomRightBack, bottomNormal);
            vertices[23] = new VertexPositionNormal(bottomLeftFront, bottomNormal);
            vertices[22] = new VertexPositionNormal(bottomRightBack, bottomNormal);
            vertices[21] = new VertexPositionNormal(bottomRightFront, bottomNormal);

            // Left face.
            vertices[26] = new VertexPositionNormal(topLeftFront, leftNormal);
            vertices[25] = new VertexPositionNormal(bottomLeftBack, leftNormal);
            vertices[24] = new VertexPositionNormal(bottomLeftFront, leftNormal);
            vertices[29] = new VertexPositionNormal(topLeftBack, leftNormal);
            vertices[28] = new VertexPositionNormal(bottomLeftBack, leftNormal);
            vertices[27] = new VertexPositionNormal(topLeftFront, leftNormal);

            // Right face. 
            vertices[32] = new VertexPositionNormal(topRightFront, rightNormal);
            vertices[31] = new VertexPositionNormal(bottomRightFront, rightNormal);
            vertices[30] = new VertexPositionNormal(bottomRightBack, rightNormal);
            vertices[35] = new VertexPositionNormal(topRightBack, rightNormal);
            vertices[34] = new VertexPositionNormal(topRightFront, rightNormal);
            vertices[33] = new VertexPositionNormal(bottomRightBack, rightNormal);

            VertexBuffer buffer = new VertexBuffer(device, VertexPositionNormal.VertexDeclaration,
                vertices.Length, BufferUsage.WriteOnly);
            buffer.SetData(vertices);

            return buffer;
        }


        static public VertexBuffer CreateTexturedBox(GraphicsDevice device, Vector3 size, float texSize)
        {
            VertexPositionNormalTexture[] vertices = new VertexPositionNormalTexture[36];

            Vector3 topLeftFront = new Vector3(-1.0f, 1.0f, -1.0f) * size;
            Vector3 bottomLeftFront = new Vector3(-1.0f, -1.0f, -1.0f) * size;
            Vector3 topRightFront = new Vector3(1.0f, 1.0f, -1.0f) * size;
            Vector3 bottomRightFront = new Vector3(1.0f, -1.0f, -1.0f) * size;
            Vector3 topLeftBack = new Vector3(-1.0f, 1.0f, 1.0f) * size;
            Vector3 topRightBack = new Vector3(1.0f, 1.0f, 1.0f) * size;
            Vector3 bottomLeftBack = new Vector3(-1.0f, -1.0f, 1.0f) * size;
            Vector3 bottomRightBack = new Vector3(1.0f, -1.0f, 1.0f) * size;

            Vector3 frontNormal = new Vector3(0.0f, 0.0f, 1.0f) * size;
            Vector3 backNormal = new Vector3(0.0f, 0.0f, -1.0f) * size;
            Vector3 topNormal = new Vector3(0.0f, 1.0f, 0.0f) * size;
            Vector3 bottomNormal = new Vector3(0.0f, -1.0f, 0.0f) * size;
            Vector3 leftNormal = new Vector3(-1.0f, 0.0f, 0.0f) * size;
            Vector3 rightNormal = new Vector3(1.0f, 0.0f, 0.0f) * size;

            // Front face.
            vertices[2] = new VertexPositionNormalTexture(topLeftFront, frontNormal, new Vector2(topLeftFront.X, topLeftFront.Z) / texSize);
            vertices[1] = new VertexPositionNormalTexture(bottomLeftFront, frontNormal, new Vector2(bottomLeftFront.X, bottomLeftFront.Z) / texSize);
            vertices[0] = new VertexPositionNormalTexture(topRightFront, frontNormal, new Vector2(topRightFront.X, topRightFront.Z) / texSize);
            vertices[5] = new VertexPositionNormalTexture(bottomLeftFront, frontNormal, new Vector2(bottomLeftFront.X, bottomLeftFront.Z) / texSize);
            vertices[4] = new VertexPositionNormalTexture(bottomRightFront, frontNormal, new Vector2(bottomRightFront.X, bottomRightFront.Z) / texSize);
            vertices[3] = new VertexPositionNormalTexture(topRightFront, frontNormal, new Vector2(topRightFront.X, topRightFront.Z) / texSize);

            // Back face.
            vertices[8] = new VertexPositionNormalTexture(topLeftBack, backNormal, new Vector2(topLeftBack.X, topLeftBack.Z) / texSize);
            vertices[7] = new VertexPositionNormalTexture(topRightBack, backNormal, new Vector2(topRightBack.X, topRightBack.Z) / texSize);
            vertices[6] = new VertexPositionNormalTexture(bottomLeftBack, backNormal, new Vector2(bottomLeftBack.X, bottomLeftBack.Z) / texSize);
            vertices[11] = new VertexPositionNormalTexture(bottomLeftBack, backNormal, new Vector2(bottomLeftBack.X, bottomLeftBack.Z) / texSize);
            vertices[10] = new VertexPositionNormalTexture(topRightBack, backNormal, new Vector2(topRightBack.X, topRightBack.Z) / texSize);
            vertices[9] = new VertexPositionNormalTexture(bottomRightBack, backNormal, new Vector2(bottomRightBack.X, bottomRightBack.Z) / texSize);

            // Top face.
            vertices[14] = new VertexPositionNormalTexture(topLeftFront, topNormal, new Vector2(topLeftFront.X, topLeftFront.Z) / texSize);
            vertices[13] = new VertexPositionNormalTexture(topRightBack, topNormal, new Vector2(topRightBack.X, topRightBack.Z) / texSize);
            vertices[12] = new VertexPositionNormalTexture(topLeftBack, topNormal, new Vector2(topLeftBack.X, topLeftBack.Z) / texSize);
            vertices[17] = new VertexPositionNormalTexture(topLeftFront, topNormal, new Vector2(topLeftFront.X, topLeftFront.Z) / texSize);
            vertices[16] = new VertexPositionNormalTexture(topRightFront, topNormal, new Vector2(topRightFront.X, topRightFront.Z) / texSize);
            vertices[15] = new VertexPositionNormalTexture(topRightBack, topNormal, new Vector2(topRightBack.X, topRightBack.Z) / texSize);

            // Bottom face. 
            vertices[20] = new VertexPositionNormalTexture(bottomLeftFront, bottomNormal, new Vector2(bottomLeftFront.X, bottomLeftFront.Z) / texSize);
            vertices[19] = new VertexPositionNormalTexture(bottomLeftBack, bottomNormal, new Vector2(bottomLeftBack.X, bottomLeftBack.Z) / texSize);
            vertices[18] = new VertexPositionNormalTexture(bottomRightBack, bottomNormal, new Vector2(bottomRightBack.X, bottomRightBack.Z) / texSize);
            vertices[23] = new VertexPositionNormalTexture(bottomLeftFront, bottomNormal, new Vector2(bottomLeftFront.X, bottomLeftFront.Z) / texSize);
            vertices[22] = new VertexPositionNormalTexture(bottomRightBack, bottomNormal, new Vector2(bottomRightBack.X, bottomRightBack.Z) / texSize);
            vertices[21] = new VertexPositionNormalTexture(bottomRightFront, bottomNormal, new Vector2(bottomRightFront.X, bottomRightFront.Z) / texSize);

            // Left face.
            vertices[26] = new VertexPositionNormalTexture(topLeftFront, leftNormal, new Vector2(topLeftFront.X, topLeftFront.Z) / texSize);
            vertices[25] = new VertexPositionNormalTexture(bottomLeftBack, leftNormal, new Vector2(bottomLeftBack.X, bottomLeftBack.Z) / texSize);
            vertices[24] = new VertexPositionNormalTexture(bottomLeftFront, leftNormal, new Vector2(bottomLeftFront.X, bottomLeftFront.Z) / texSize);
            vertices[29] = new VertexPositionNormalTexture(topLeftBack, leftNormal, new Vector2(topLeftBack.X, topLeftBack.Z) / texSize);
            vertices[28] = new VertexPositionNormalTexture(bottomLeftBack, leftNormal, new Vector2(bottomLeftBack.X, bottomLeftBack.Z) / texSize);
            vertices[27] = new VertexPositionNormalTexture(topLeftFront, leftNormal, new Vector2(topLeftFront.X, topLeftFront.Z) / texSize);

            // Right face. 
            vertices[32] = new VertexPositionNormalTexture(topRightFront, rightNormal, new Vector2(topRightFront.X, topRightFront.Z) / texSize);
            vertices[31] = new VertexPositionNormalTexture(bottomRightFront, rightNormal, new Vector2(bottomRightFront.X, bottomRightFront.Z) / texSize);
            vertices[30] = new VertexPositionNormalTexture(bottomRightBack, rightNormal, new Vector2(bottomRightBack.X, bottomRightBack.Z) / texSize);
            vertices[35] = new VertexPositionNormalTexture(topRightBack, rightNormal, new Vector2(topRightBack.X, topRightBack.Z) / texSize);
            vertices[34] = new VertexPositionNormalTexture(topRightFront, rightNormal, new Vector2(topRightFront.X, topRightFront.Z) / texSize);
            vertices[33] = new VertexPositionNormalTexture(bottomRightBack, rightNormal, new Vector2(bottomRightBack.X, bottomRightBack.Z) / texSize);

            VertexBuffer buffer = new VertexBuffer(device, VertexPositionNormalTexture.VertexDeclaration,
                vertices.Length, BufferUsage.WriteOnly);
            buffer.SetData(vertices);

            return buffer;
        }

        static public Model CreateBall(ContentManager content)
        {
            return content.Load<Model>("ball");
        }

        static public void DrawBox(GraphicsDevice device, VertexBuffer buffer)
        {
            device.SetVertexBuffer(buffer);
            device.DrawPrimitives(PrimitiveType.TriangleList, 0, 12);
        }

        static public void DrawBall(GraphicsDevice device, BasicEffect effect, Model model)
        {
            foreach (ModelMesh mesh in model.Meshes)
            {
                foreach (BasicEffect modelEffect in mesh.Effects)
                {
                    modelEffect.World = effect.World;
                    modelEffect.View = effect.View;
                    modelEffect.Projection = effect.Projection;
                }

                mesh.Draw();
            }

        }
    }
}
