/*
	Classes taken from BulletSharp SoftDemo but modified and some merged
*/

using System;
using Microsoft.Xna.Framework;
using BulletSharp.SoftBody;

namespace BulletTest
{
    class FreeRollControl : AJoint.IControl
    {

        /// <summary>
        /// Skapar en fritt roterande axel( styrningen är låst)
        /// </summary>
        public FreeRollControl() { }

        public override float Speed(AJoint joint, float current)
        {
            return current;
        }
    }

    class MotorControl : AJoint.IControl
    {
        float goal = 0;
        float maxTorque = 0;

        public float Goal
        {
            get { return goal; }
            set { goal = value; }
        }

        /// <summary>
        /// Skapar en kontroll som bara kan driva( styrningen är låst)
        /// </summary>
        public MotorControl() { }

        public float MaxTorque
        {
            get { return maxTorque; }
            set { maxTorque = value; }
        }

        public override float Speed(AJoint joint, float current)
        {
            return current + Math.Min(maxTorque, Math.Max(-maxTorque, goal - current));
        }
    }

    class SteerAndMotorControl : SteerControl
    {
        protected MotorControl Motor;

        /// <summary>
        /// Skapar en kontroll som både kan driva samt svänga
        /// </summary>
        /// <param name="sign">Riktning</param>
        public SteerAndMotorControl(float sign, MotorControl motor) : base(sign) 
        {
            Motor = motor;
        }

        public override float Speed(AJoint joint, float current)
        {
            return Motor.Speed(joint, current);
        }
    }

    class SteerControl : AJoint.IControl
    {
        float minAngle = -MathHelper.PiOver2 / 2f;
        float maxAngle = MathHelper.PiOver2 / 2f;

        float angle = 0;
        protected float sign;

        public float Angle
        {
            get { return angle; }

            //Limiting the steering angle to minAngle to maxAngle
            set { angle = MathHelper.Clamp(value, minAngle, maxAngle); }
        }

        /// <summary>
        /// Skapar en kontroll som bara kan svänga( om hjul används kommer dessa att rotera fritt som i "friläge")
        /// </summary>
        /// <param name="sign"></param>
        public SteerControl(float sign)
        {
            this.sign = sign;
        }

        public override void Prepare(AJoint joint)
        {
            joint.Refs[0] = new Vector3((float)Math.Cos(angle * sign), 0, (float)Math.Sin(angle * sign));
        }

        public override float Speed(AJoint joint, float current)
        {
            return current;
        }
    }
}
