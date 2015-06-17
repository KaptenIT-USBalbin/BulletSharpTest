/*
	Classes taken from BulletSharp SoftDemo
*/

using System;
using Microsoft.Xna.Framework;
using BulletSharp.SoftBody;

namespace BulletTest
{
    class MotorControl : AJoint.IControl
    {
        float goal = 0;
        float maxTorque = 0;

        public float Goal
        {
            get { return goal; }
            set { goal = value; }
        }

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

    class SteerControl : AJoint.IControl
    {
        float angle = 0;
        float sign;

        public float Angle
        {
            get { return angle; }
            set { angle = value; }
        }

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
            return Physics.Motor.Speed(joint, current);
        }
    }
}
