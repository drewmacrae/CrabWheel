﻿using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

/* Code: Wheel plugin for KSP.
   Author: drewmacrae@gmail.com
   License: BY-SA, Attribution & Share-alike Creative Common Licence. Feel
       free to modify as long as a) original author is mentioned, and b) 
       you distribute your code under the same license.
   Inspired by:
       Cart plugin for KSP by tosh@bk.ru
   Features: 
     - Surface attachable omnidirectional drive unit.
     - Working suspension. 
     - Forward and backward drive (<I>/<K> keys), brake (<N> key).
     - Left to right strafe (<J>/<L>), Rotation (<Q>,<E>)
     - Procedural steering and drive of the wheels
   Issues:
     - Totally helpless in water.
     - The model and texture are a bit ugly :). 
     - Certian steering maneuvers will break things. Go slow. Steer gently and 
       use the <N> key when executing sharp turns
     - when in contact with next stage animates badly at speed
     - suspension rotated by 90 degrees
   Note on models. As you can see .DAE file contains a helluwa lot of objects.
       Wheel is the one that rolls (animated); Suspension is the suspension
       lever and is animated separately. And Center is an invisible GameObject to 
       which a WheelCollider is connected.
*/
   
class Wheel : Part {
    private Transform model;//wheel
    private Transform center;//stationary part
    private Transform suspension;//pivot and spring
    private WheelCollider wheel;
    private float wheelRotation;
    private float pivot;
    private float drive;

    private bool moveRight    = false,//control inputs 
		         moveLeft     = false, 
		         moveForward  = false, 
		         moveBackward = false,
                 rotateRight  = false,
                 rotateLeft   = false,
		         brake        = false;

    private bool created = false;

    private float pivotGain;

    //----------------------//
    //configurable variables//
    //----------------------//
    public float brakeForce = 1.5F;
    public float suspensionSpring = 2.0F;
    public float suspensionDamper = 0.3F;
    public float suspensionDistance = 0.5F;
    public float wheelRadius = 0.3F;
    public float wheelMass = 0.1F;
    public float driveStrength = 0.5F;
    public float turnStrength = 0.02F;
    public float steeringRateLimit = 45F;
    public float driveMotorTimeConstant = 1F;
    public float viscousFrictionCoefficient = 0.2F;


    
    // Find all the parts of interest and hide wheel centers on startup.
    protected override void onPartStart() {
        //print("Wheel: STARTING WHEEL");
        model = transform.Find( "model/wheel" );
        suspension = transform.Find( "model/suspension" );
    	center = transform.Find( "model/center" );
        center.renderer.enabled = false;
        center.localEulerAngles = new Vector3(90, 0, 0);//rotate center to face 
        //downward the center will cast a ray downward to find how high it is 
        //off the ground. If it's rotated wrong it will allow the wheel 
        //colliders to fall into the terrain.
        pivot = 0;
        drive = 0;
        pivotGain = steeringRateLimit / 0.9F;
        
        base.onPartStart();
    }
    
    // Build physics. 
    private void createObjects() {
        if( created ) 
            return;

        JointSpring spring; // a spring used by all WheelColliders
        spring.spring = suspensionSpring;
        spring.damper = suspensionDamper;
        spring.targetPosition = 0F;

        // move wheel and suspension to proper poistion 
        model.localPosition = center.localPosition;
        suspension.localPosition = center.localPosition;

        // connect a WheelCollider to wheel center object
        wheel = center.gameObject.AddComponent< WheelCollider >();
        wheel.radius = wheelRadius;
        wheel.suspensionDistance = suspensionDistance;
        wheel.mass = wheelMass;
	    wheel.suspensionSpring = spring;

        wheelRotation = 0;

        created = true;
    }

    // Destroy physics.
    private void destroyObjects() {
        if( created ) {          
           //print( "CART: DESTROYING WHEEL" );

           created = false;

           Destroy( wheel );
           wheel = null;
        }
    }

    // Destroy all the physics objects when the part is packed to orbit.
    protected override void onPack() {
        destroyObjects();
        base.onPack();
    }

    // Process controls and wheels animation.
    // Currently all the physics objects are created in this call, as soon 
    // as the part has created its rigidbody. Physics creation in OnPartStart 
    // or OnFlightStart does not work because the game recreates 
    // node_collider after that point (and fails if something is attached to 
    // it).
    protected override void onPartFixedUpdate()
    {
        if ((rigidbody != null) && (!rigidbody.isKinematic) && !created)
            createObjects();

        base.onPartFixedUpdate();

        if (!created)
            return;
        
        float velocity = rigidbody.velocity.magnitude;

        Vector3 offCenter = vessel.findWorldCenterOfMass() - model.position;
        offCenter = offCenter-Vector3.Project(offCenter, vessel.transform.up);

        //where does the player want to go?
        Vector3 driveComp = vessel.transform.right * (moveRight ? driveStrength : moveLeft ? -driveStrength : 0) +
                            vessel.transform.forward * (moveForward ? driveStrength : moveBackward ? -driveStrength : 0);
        float turnMag = (rotateRight ? turnStrength : rotateLeft ? -turnStrength : 0);
        
        //which way should the wheel drive to turn
        Vector3 turnDir = Vector3.Cross(vessel.transform.up,offCenter);
        Vector3 turnComp = turnMag*offCenter.magnitude*turnDir.normalized;
        
        Vector3 total = driveComp+turnComp;

        if (total.magnitude > 0.001F)//if we want to be driving anywhere
        {
            //solve for the nearest steering angle that is appropriate
            Vector3 totalDir = total.normalized;
            float tripProd=Vector3.Dot(vessel.transform.up,Vector3.Cross(suspension.transform.right,totalDir));
            if (Mathf.Abs(tripProd)>1F)
            {
                print("assertion Failed, triple Product overran range");
                tripProd = Mathf.Sign(tripProd);
            }

            float angularError = Mathf.Asin(tripProd)*Mathf.Sign(Vector3.Dot(suspension.transform.right,totalDir));
            
            float degreeError = angularError * 180F / Mathf.PI;
            pivot = pivot + angularError * pivotGain * TimeWarp.fixedDeltaTime;
            
        }
        float decay = driveMotorTimeConstant*TimeWarp.fixedDeltaTime;//FIXME add justification
        drive = drive * (1 - decay) - decay * Vector3.Dot(total.normalized, suspension.transform.right);//

        // Wheels friction. Proportional to speed to prevent car jerking on 
        // stop, and constant at high speed.
        float friction = velocity * viscousFrictionCoefficient;
        if (friction < 0.001F)
            friction = 0; // it jerks otherwise.


        float braking = brake ? brakeForce : friction;

        // Steering.
        wheel.steerAngle = pivot;

        bool grounded = false;

        // Determine whether the WheelCollider touches the ground,
        // and at what distance from rigidbody's origin.
        float offset = wheel.suspensionDistance;
        WheelHit wh;
        if (wheel.GetGroundHit(out wh))
        {
            wh.point = center.InverseTransformPoint(wh.point);
            offset = -(wheel.radius + wh.point.y);
            grounded = true; 
            if (offset> wheel.suspensionDistance)
                //if it's in contact with the rest of the ship it will gradually drift 
                offset = wheel.suspensionDistance;
        }

        if (grounded)
            wheel.motorTorque = drive / (velocity + 5F) * 5F;//over 5 m/s limit power not torque
        else
            wheel.motorTorque = drive;
        wheel.brakeTorque = braking;

        // now offset visible model and suspension lever. 
        Vector3 v = model.localPosition;
        v.z = center.localPosition.z - offset;
        model.localPosition = v;
        suspension.localPosition = v;
        // animate steering and wheel rotation.

        model.localEulerAngles = new Vector3(0,0,wheel.steerAngle);
        suspension.localEulerAngles = new Vector3(0, 0, 90F+wheel.steerAngle);
        model.Rotate(wheelRotation, 0, 0);
        wheelRotation += 6 * wheel.rpm *
                               TimeWarp.fixedDeltaTime;
        wheelRotation %= 360;
    }
    // Process keyboard input. //FIXME so I can be linked to axes instead
    protected override void onCtrlUpd( FlightCtrlState s ) {
        rotateRight = Input.GetKey( KeyCode.E );
        rotateLeft = Input.GetKey( KeyCode.Q );
        moveRight= Input.GetKey( KeyCode.L );
        moveLeft = Input.GetKey( KeyCode.J );
        moveForward = Input.GetKey( KeyCode.I );
        moveBackward = Input.GetKey( KeyCode.K );
        brake = Input.GetKey( KeyCode.N );

        base.onCtrlUpd( s );
    }

}