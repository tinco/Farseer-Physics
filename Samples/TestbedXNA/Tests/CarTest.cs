/*
* Farseer Physics Engine based on Box2D.XNA port:
* Copyright (c) 2011 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org 
* 
* This software is provided 'as-is', without any express or implied 
* warranty.  In no event will the authors be held liable for any damages 
* arising from the use of this software. 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it 
* freely, subject to the following restrictions: 
* 1. The origin of this software must not be misrepresented; you must not 
* claim that you wrote the original software. If you use this software 
* in a product, an acknowledgment in the product documentation would be 
* appreciated but is not required. 
* 2. Altered source versions must be plainly marked as such, and must not be 
* misrepresented as being the original software. 
* 3. This notice may not be removed or altered from any source distribution. 
*/

using System;
using FarseerPhysics.Collision.Shapes;
using FarseerPhysics.Common;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Dynamics.Joints;
using FarseerPhysics.Factories;
using FarseerPhysics.TestBed.Framework;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace FarseerPhysics.TestBed.Tests
{
    public class CarTest : Test
    {
        private Body _car;
        public float steeringAngle;
        private float engineSpeed;

        public Body LeftWheel, RightWheel, LeftRearWheel, RightRearWheel;
        public RevoluteJoint leftJoint, rightJoint;
        public WeldJoint LeftRearJoint, RightRearJoint;


        /** Defaults **/
        private const float DEFAULT_MAX_STEER_ANGLE = (float)(Math.PI / 5);
        private const int DEFAULT_HORSEPOWER = 120;
        private const float DEFAULT_STEER_SPEED = 10f;
        private const float DEFAULT_SIDEWAYS_FRICTION_FORCE = 10f;

        private const float DEFAULT_CAR_WIDTH = 3f;
        private const float DEFAULT_CAR_HEIGHT = 5f;
        private const float DEFAULT_WHEEL_WIDTH = 0.4f;
        private const float DEFAULT_WHEEL_HEIGHT = 1f;

        private static readonly Vector2 DEFAULT_LR_WHEELPOS = new Vector2(-1.5f, 1.90f);
        private static readonly Vector2 DEFAULT_RR_WHEELPOS = new Vector2(1.5f, 1.9f);
        private static readonly Vector2 DEFAULT_LF_WHEELPOS = new Vector2(-1.5f, -1.9f);
        private static readonly Vector2 DEFAULT_RF_WHEELPOS = new Vector2(1.5f, -1.9f);

        private CarTest()
        {
            steeringAngle = 0;
            engineSpeed = 0;


            World.Gravity = Vector2.Zero;

            var body = BodyFactory.CreateBody(World);
            _car = body;

            body.LinearDamping = 1f;
            body.AngularDamping = 1f;
            body.Friction = 0;
            body.BodyType = BodyType.Dynamic;

            // Define wheel bodies
            LeftWheel = BodyFactory.CreateBody(World);
            LeftWheel.BodyType = BodyType.Dynamic;
            LeftWheel.Friction = 1;


            RightWheel = BodyFactory.CreateBody(World);
            RightWheel.BodyType = BodyType.Dynamic;
            RightWheel.Friction = 1;

            LeftRearWheel = BodyFactory.CreateBody(World);
            LeftRearWheel.BodyType = BodyType.Dynamic;
            LeftRearWheel.Friction = 1;

            RightRearWheel = BodyFactory.CreateBody(World);
            RightRearWheel.BodyType = BodyType.Dynamic;
            RightRearWheel.Friction = 1;


            LeftWheel.Position = DEFAULT_LF_WHEELPOS;
            RightWheel.Position = DEFAULT_RF_WHEELPOS;
            LeftRearWheel.Position = DEFAULT_LR_WHEELPOS;
            RightRearWheel.Position = DEFAULT_RR_WHEELPOS;

            // Define shapes
            var carShape = new PolygonShape(1);
            carShape.SetAsBox(DEFAULT_CAR_WIDTH / 2, DEFAULT_CAR_HEIGHT / 2);
            var fixture = body.CreateFixture(carShape);

            var wheelShape = new PolygonShape(1);
            wheelShape.SetAsBox(DEFAULT_WHEEL_WIDTH / 2, DEFAULT_WHEEL_HEIGHT / 2);

            LeftWheel.CreateFixture(wheelShape);
            RightWheel.CreateFixture(wheelShape);
            LeftRearWheel.CreateFixture(wheelShape);
            RightRearWheel.CreateFixture(wheelShape);

            // Define joints
            leftJoint = JointFactory.CreateRevoluteJoint(World, LeftWheel, body, DEFAULT_LF_WHEELPOS); //Position relative to the body
            //leftJoint = JointFactory.CreateWeldJoint(World, LeftWheel, body, DEFAULT_LF_WHEELPOS); //Position relative to the body
            leftJoint.MotorEnabled = true;
            leftJoint.LimitEnabled = true;
            leftJoint.LowerLimit = -DEFAULT_MAX_STEER_ANGLE;
            leftJoint.UpperLimit = DEFAULT_MAX_STEER_ANGLE;
            leftJoint.MaxMotorTorque = 10; //Doesnt seem to do anything*/

            //rightJoint = JointFactory.CreateWeldJoint(World, RightWheel, body, DEFAULT_RF_WHEELPOS); //Position relative to the body
            rightJoint = JointFactory.CreateRevoluteJoint(World, RightWheel, body, DEFAULT_RF_WHEELPOS); //Position relative to the body
            rightJoint.MotorEnabled = true;
            rightJoint.LimitEnabled = true;
            rightJoint.LowerLimit = -DEFAULT_MAX_STEER_ANGLE;
            rightJoint.UpperLimit = DEFAULT_MAX_STEER_ANGLE;
            rightJoint.MaxMotorTorque = 10; //Doesnt seem to do anything*/

            LeftRearJoint = JointFactory.CreateWeldJoint(World, body, LeftRearWheel, DEFAULT_LR_WHEELPOS);
            //LeftRearJoint.LimitEnabled = true;
            //LeftRearJoint.LowerLimit = LeftRearJoint.UpperLimit = 0;

            RightRearJoint = JointFactory.CreateWeldJoint(World, body, RightRearWheel, DEFAULT_RR_WHEELPOS);
            //RightRearJoint.LimitEnabled = true;
            //RightRearJoint.LowerLimit = RightRearJoint.UpperLimit = 0;


        }

        public void SetState(Vector2 position)
        {
            _car.SetTransform(ref position, 0);
            //Debug.WriteLine("Client Position: " + state.Position + " Rotation: " + state.WheelAngle);
            //Debug.WriteLine("   Own Position: " + PhysicsBody.Position + " Rotation: " + LeftWheel.Rotation);
            /*var dP = _car.Position - position;
            var dA = _car.Rotation - 0;
            _car.SetTransform(ref position, 0);
            //*/
            /*LeftWheel.Position = LeftWheel.Position + dP;
            RightWheel.Position += dP;
            RightRearWheel.Position += dP;
            LeftRearWheel.Position += dP;*/
            /*LeftWheel.Rotation = LeftWheel.Rotation + dA;
            leftJoint.LocalAnchorA = LeftWheel.GetLocalPoint(LeftWheel.Position);
            leftJoint.LocalAnchorB = _car.GetLocalPoint(LeftWheel.Position);
            leftJoint.ReferenceAngle = _car.Rotation - LeftWheel.Rotation;
            //RightWheel.Position = rotate(RightWheel.Position, position, _car.Rotation);
            //LeftRearWheel.Position = rotate(LeftRearWheel.Position, state.Position, PhysicsBody.Rotation);
            //RightRearWheel.Position = rotate(RightRearWheel.Position, state.Position, PhysicsBody.Rotation);
            /*
            _car.AngularVelocity = carState.AngularVelocity;
            _car.LinearVelocity = carState.LinearVelocity;
            LeftWheel.Rotation = carState.WheelAngle;
            RightWheel.Rotation = carState.WheelAngle;
            */
            // EngineSpeed
        }

        private Vector2 rotate(Vector2 s, Vector2 o, float angle)
        {
            /** lol wut? */
            var x = 0f;
            var y = 0f;
            return new Vector2(x, y);
        }

        private void killOrthogonalVelocity()
        {
            Body[] wheels = { LeftWheel, RightWheel, LeftRearWheel, RightRearWheel };
            Transform t;
            foreach (var wheel in wheels)
            {
                var localPoint = Vector2.Zero;
                var velocity = wheel.GetLinearVelocityFromLocalPoint(localPoint);
                wheel.GetTransform(out t);
                var sidewaysAxis = t.q.GetYAxis();
                wheel.LinearVelocity = (sidewaysAxis * (Vector2.Dot(velocity, sidewaysAxis)));

            }
        }

        private bool doUpdate = false;
        public override void Keyboard(KeyboardManager keyboardManager)
        {
            if (keyboardManager.IsKeyDown(Keys.W))
            {
                engineSpeed = -DEFAULT_HORSEPOWER;
            }
            if (keyboardManager.IsKeyDown(Keys.S))
            {
                engineSpeed = DEFAULT_HORSEPOWER;
            }
            if (keyboardManager.IsKeyDown(Keys.D))
            {
                steeringAngle = DEFAULT_MAX_STEER_ANGLE;
            }
            if (keyboardManager.IsKeyDown(Keys.A))
            {

                steeringAngle = -DEFAULT_MAX_STEER_ANGLE;
            }
            if (keyboardManager.IsNewKeyPress(Keys.Q))
            {
            }
            if (keyboardManager.IsNewKeyPress(Keys.E))
            {
            }
            if (keyboardManager.IsNewKeyPress(Keys.N))
            {
                doUpdate = true;
            }

            base.Keyboard(keyboardManager);
        }

        Vector2 newPosition;

        public override void Mouse(MouseState state, MouseState oldState)
        {
            Vector2 position = GameInstance.ConvertScreenToWorld(state.X, state.Y);

            if (state.LeftButton == ButtonState.Pressed && oldState.LeftButton == ButtonState.Released)
            {
                newPosition = position;
            }

            if (state.LeftButton == ButtonState.Released && oldState.LeftButton == ButtonState.Pressed)
            {
            }
        }

        public override void Update(GameSettings settings, GameTime gameTime)
        {
            if (!doUpdate)
            {
                return;
            }

            if (newPosition != Vector2.Zero)
            {
                SetState(newPosition);
                newPosition = Vector2.Zero;
            }
            //GameInstance.ViewCenter = _car.Position;
            Transform t;
            killOrthogonalVelocity();
            /* Driving */
            LeftRearWheel.GetTransform(out t);
            var ldirection = t.q.GetYAxis();
            ldirection *= engineSpeed / 2;
            RightRearWheel.GetTransform(out t);
            var rdirection = t.q.GetYAxis();
            rdirection *= engineSpeed / 2;
            LeftRearWheel.ApplyForce(ldirection);
            RightRearWheel.ApplyForce(rdirection);
            LeftWheel.GetTransform(out t);
            ldirection = t.q.GetYAxis();
            ldirection *= engineSpeed / 2;
            RightWheel.GetTransform(out t);
            rdirection = t.q.GetYAxis();
            rdirection *= engineSpeed / 2;
            LeftWheel.ApplyForce(ldirection); // front wheel drive?
            RightWheel.ApplyForce(rdirection);

            //Steering
            var mspeed = steeringAngle - leftJoint.JointAngle;
            leftJoint.MotorSpeed = (mspeed * DEFAULT_STEER_SPEED);
            mspeed = steeringAngle - rightJoint.JointAngle;
            rightJoint.MotorSpeed = (mspeed * DEFAULT_STEER_SPEED);

            //_car.Rotation += steeringAngle;
            resetInput();
            doUpdate = false;
            base.Update(settings, gameTime);
        }

        private void resetInput()
        {
            engineSpeed = 0;
            steeringAngle = 0;
        }

        internal static Test Create()
        {
            return new CarTest();
        }
    }
}