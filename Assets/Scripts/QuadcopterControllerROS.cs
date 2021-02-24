﻿using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class QuadcopterControllerROS : Agent
{
	[SerializeField] private Transform drone;
	[SerializeField] private Transform level;
	[SerializeField] private Transform target;
	[SerializeField] private Transform centerOfMass;
	[SerializeField] private Transform ForwardRightEngine;
	[SerializeField] private Transform ForwardLeftEngine;
	[SerializeField] private Transform BackRightEngine;
	[SerializeField] private Transform BackLeftEngine;
	[SerializeField] private float throttleSpeed = 25;
	[SerializeField] private float turnSpeed = 100;
	[SerializeField] private float sideSpeed = 200;
	[SerializeField] private float forwardSpeed = 150;
	[SerializeField] private bool realisticSideAdjustment;

	[SerializeField] private PID yPid = new PID(0, 0, 0);
	[SerializeField] private PID xPid = new PID(0, 0, 0);
	[SerializeField] private PID sidePid = new PID(0, 0, 0);

	[SerializeField] private PID yawPid = new PID(0, 0, 0);
	[SerializeField] private PID pitchPid = new PID(0, 0, 0);
	[SerializeField] private PID rollPid = new PID(0, 0, 0);

	[SerializeField] private float maxEngineSpeed = 100;

	private SortedList<int, Action<VelocityContext>> CalculateWorldDesiredVelocity = new SortedList<int, Action<VelocityContext>>();
	private float forwardAmount;
	private float throttle;
	private VelocityContext velocityContext = new VelocityContext();

	private Rigidbody rigidbody;

	private void OnCollisionEnter(Collision collision)
	{
		//AddReward(-0.1f);
		//Debug.Log("Coolision. Total rewards: "+ GetCumulativeReward());
		if (collision.gameObject.tag != "Target")
			EndEpisode();
	}

	private void land()
	{
		landing = true;
	}

	private void OnTriggerEnter(Collider other)
	{
		moveTarget = false;
		AddReward(3.0f);
		//Debug.Log("TARGET. Total rewards: " + GetCumulativeReward());
		StartCoroutine(Wait(3));
		land();
	}

	IEnumerator Wait(float duration)
	{
		yield return new WaitForSeconds(duration);
		EndEpisode();
	}

	private void OnTriggerStay(Collider other)
	{
		AddReward(5f);
		//Debug.Log("TARGET. Total rewards: " + GetCumulativeReward());

	}

	//ML BEGIN
	public override void Initialize()
	{
		rigidbody = GetComponent<Rigidbody>();
		rigidbody.centerOfMass = transform.InverseTransformPoint(centerOfMass.transform.position);
		velocityContext.DesiredDirection = Vector3.forward;

	}

	float previousRotationDifference = 1000f, previousHorizontalDistance = 1000f, previousVerticalDistance = 1000f;
	float currentRotation, desiredRotation, horizontalDistance, verticalDistance;

	public override void CollectObservations(VectorSensor sensor)
	{
		//Direction
		sensor.AddObservation(currentRotation - desiredRotation);
		//Speed
		sensor.AddObservation(horizontalDistance);
		//Altitude
		sensor.AddObservation(drone.position.y - target.position.y);
		//Velocity
		sensor.AddObservation(rigidbody.velocity.magnitude);
	}

	public override void OnActionReceived(float[] vectorAction)
	{
		initializeControlls();
		int direction = Mathf.FloorToInt(vectorAction[0]);
		int altitude = Mathf.FloorToInt(vectorAction[1]);
		int speed = Mathf.FloorToInt(vectorAction[2]);
		//Debug.Log("Altitude = " + altitude + " Direction = " + direction + " Speed = " + speed);
		if (landing)
		{
			goDown();
		}
		/*switch (direction)
		{
			case 1:
				goLeft();
				break;
			case 2:
				goRight();
				break;
		}
		switch (altitude)
		{
			case 1:
				goUp();
				break;
			case 2:
				goDown();
				break;
		}
		switch (speed)
		{
			case 1:
				goForward();
				break;
			case 2:
				goBackward();
				break;
		}*/

		//Turning
		currentRotation = drone.transform.eulerAngles.y;
		Quaternion desiredRot = Quaternion.LookRotation(target.position - drone.position, Vector3.up);
		desiredRotation = desiredRot.eulerAngles.y;
		//normalize to [-1, 1]
		currentRotation = ((2f * currentRotation) / 360f) - 1f;
		desiredRotation = ((2f * desiredRotation) / 360f) - 1f;
		float rotationDifference = Mathf.Abs(currentRotation - desiredRotation);
		if (rotationDifference < (previousRotationDifference - 0.05f))
		{
			AddReward(0.1f);
			//Debug.Log("ROTATION + Total rewards: " + GetCumulativeReward() + " difference: " + rotationDifference + " current: " + currentRotation + " desired: " + desiredRotation);
			previousRotationDifference = rotationDifference;
		}
		else if (rotationDifference > (previousRotationDifference + 0.05f))
		{
			AddReward(-0.2f);
			//Debug.Log("ROTATION - Total rewards: " + GetCumulativeReward() + " difference: " + rotationDifference + " current: " + currentRotation + " desired: " + desiredRotation);
			previousRotationDifference = rotationDifference;
		}

		//Horizontal
		horizontalDistance = Vector2.Distance(new Vector2(drone.position.x, drone.position.z), new Vector2(target.position.x, target.position.z));
		if (horizontalDistance < (previousHorizontalDistance - 0.2f))
		{
			AddReward(0.1f);
			//Debug.Log("HORIZONTAL +. Total rewards: " + GetCumulativeReward() + " horizontal distance: " + horizontalDistance);
			previousHorizontalDistance = horizontalDistance;
		}
		else if (horizontalDistance > (previousHorizontalDistance + 0.2f))
		{
			AddReward(-0.2f);
			//Debug.Log("HORIZONTAL -. Total rewards: " + GetCumulativeReward() + " horizontal distance: " + horizontalDistance);
			previousHorizontalDistance = horizontalDistance;
		}

		//Vertical
		verticalDistance = Mathf.Abs(drone.position.y - target.position.y);
		if (verticalDistance < (previousVerticalDistance - 0.2f))
		{
			AddReward(0.1f);
			//Debug.Log("VERTICAL +. Total rewards: " + GetCumulativeReward() + " vertical distance: " + verticalDistance);
			previousVerticalDistance = verticalDistance;
		}
		else if (verticalDistance > (previousVerticalDistance + 0.2f))
		{
			AddReward(-0.2f);
			//Debug.Log("VERTICAL -. Total rewards: " + GetCumulativeReward() + " vertical distance: " + verticalDistance);
			previousVerticalDistance = verticalDistance;
		}

		//Debug.Log("Total Rewards: " + GetCumulativeReward());

		//MANUAL CONTROLS
		/*
		if (rotationDifference < 5)
		{
			correctDirection = 0;
		}
		else if (desiredRotation > currentRotation && (desiredRotation - currentRotation) < 180)
		{
			actionsOut[1] = 2;
			correctDirection = 2;
		}
		else
		{
			actionsOut[1] = 1;
			correctDirection = 1;
		}*//*
		if (horizontalDistance > 0.9 && rotationDifference < 5)
		{
			goForward();
			correctSpeed = 1;
		}
		else
		{
			correctSpeed = 0;
		}
		if (verticalDistance > 0 && horizontalDistance < 1)
		{
			goDown();
			correctAltitude = 1;
		}
		else
		{
			correctAltitude = 0;
		}*/

	}

	private float spawnRange = 10f;

	public override void OnEpisodeBegin()
	{
		rigidbody.centerOfMass = transform.InverseTransformPoint(centerOfMass.transform.position);
		velocityContext.DesiredDirection = Vector3.forward;
		drone.SetPositionAndRotation(new Vector3(level.position.x + UnityEngine.Random.Range(-spawnRange, spawnRange), UnityEngine.Random.Range(1f, 3f), level.position.z + UnityEngine.Random.Range(-spawnRange, spawnRange)), Quaternion.Euler(0, UnityEngine.Random.Range(0f, 360f), 0));
		target.SetPositionAndRotation(new Vector3(level.position.x + UnityEngine.Random.Range(-spawnRange, spawnRange), 1, level.position.z + UnityEngine.Random.Range(-spawnRange, spawnRange)), Quaternion.Euler(0, 0, 0));
		forwardAmount = 0f;
		throttle = 0f;
		CalculateWorldDesiredVelocity = new SortedList<int, Action<VelocityContext>>();
		velocityContext = new VelocityContext();
		velocityContext.DesiredDirection = Vector3.forward;
		moveTarget = true;
		landing = false;
	}

	public override void Heuristic(float[] actionsOut)
	{
		actionsOut[0] = 0f;
		actionsOut[1] = 0f;
		actionsOut[2] = 0f;
		AddControlls(actionsOut);
	}
	//ML END

	private bool targetMovementDirectionX = true, targetMovementDirectionZ = true, moveTarget = true, landing = false;
	public float targetSpeed;
	private void FixedUpdate()
	{
		AddInternalForces();
		//Target Movement
		/*if (moveTarget)
		{
			if (targetMovementDirectionX)
			{
				target.Translate(UnityEngine.Random.Range(0f, targetSpeed) * Time.deltaTime, 0f, 0f);
				if (target.position.x > level.position.x + 10)
					targetMovementDirectionX = false;
			}
			else
			{
				target.Translate(UnityEngine.Random.Range(-targetSpeed, 0f) * Time.deltaTime, 0f, 0f);
				if (target.position.x < level.position.x - 10)
					targetMovementDirectionX = true;
			}
			if (targetMovementDirectionZ)
			{
				target.Translate(0f, 0f, UnityEngine.Random.Range(0f, targetSpeed) * Time.deltaTime);
				if (target.position.z > level.position.z + 10)
					targetMovementDirectionZ = false;
			}
			else
			{
				target.Translate(0f, 0f, UnityEngine.Random.Range(-targetSpeed, 0f) * Time.deltaTime);
				if (target.position.z < level.position.z - 10)
					targetMovementDirectionZ = true;
			}
		}*/
	}

	private void initializeControlls()
	{
		//yawDestination = 0;
		forwardAmount = 0;
		throttle = 0;
	}

	private void AddControlls(float[] actionsOut)
	{
		//initializeControlls();

		if (Input.GetKey(KeyCode.Space))
		{
			//goUp();
			actionsOut[1] = 1f;
		}
		if (Input.GetKey(KeyCode.LeftShift))
		{
			//goDown();
			actionsOut[1] = 2f;
		}
		if (Input.GetKey(KeyCode.W))
		{
			//goForward();
			actionsOut[2] = 1f;
		}
		if (Input.GetKey(KeyCode.S))
		{
			//goBackward();
			actionsOut[2] = 2f;
		}
		if (Input.GetKey(KeyCode.A))
		{
			//goLeft();
			actionsOut[0] = 1f;
		}
		if (Input.GetKey(KeyCode.D))
		{
			//goRight();
			actionsOut[0] = 2f;
		}
	}

	private void goDown()
	{
		throttle -= Time.deltaTime * throttleSpeed;
	}

	private void goUp()
	{
		throttle += Time.deltaTime * throttleSpeed;
	}

	private void goForward()
	{
		forwardAmount += Time.deltaTime * forwardSpeed;
	}

	private void goBackward()
	{
		forwardAmount -= Time.deltaTime * forwardSpeed;
	}

	private void goLeft()
	{
		velocityContext.DesiredDirection = Quaternion.AngleAxis(Time.deltaTime * turnSpeed, Vector3.down) * velocityContext.DesiredDirection;
	}

	private void goRight()
	{
		velocityContext.DesiredDirection = Quaternion.AngleAxis(Time.deltaTime * turnSpeed, Vector3.up) * velocityContext.DesiredDirection;
	}

	public Vector3 CalculateWorldDesiredVelocityInternal()
	{
		return (velocityContext.DesiredDirection * forwardAmount) + Vector3.up * throttle;
	}

	private void AddInternalForces()
	{
		Quaternion inverseDesiredLookRotation = Quaternion.Inverse(Quaternion.LookRotation(velocityContext.DesiredDirection, -Physics.gravity.normalized));

		Vector3 localVelocity = inverseDesiredLookRotation * rigidbody.velocity;
		velocityContext.DesiredWorldVelocity = CalculateWorldDesiredVelocityInternal();
		foreach (var listener in CalculateWorldDesiredVelocity)
		{
			listener.Value.Invoke(velocityContext);
		}

		Vector3 localDesiredVelocity = inverseDesiredLookRotation * velocityContext.DesiredWorldVelocity;

		float newX = Mathf.Clamp((float)xPid.Update(localDesiredVelocity.x, localVelocity.x, Time.deltaTime), -maxEngineSpeed, maxEngineSpeed);
		float newZ = Mathf.Clamp((float)sidePid.Update(localDesiredVelocity.z, localVelocity.z, Time.deltaTime), -maxEngineSpeed, maxEngineSpeed);

		float pitchError = GetPitchError();
		float rollError = GetRollError();
		float yawError = GetYawError();

		float newXRot = (float)pitchPid.Update(0, pitchError, Time.deltaTime);
		float newYRot = (float)yawPid.Update(0, yawError, Time.deltaTime);
		float newZRot = (float)rollPid.Update(0, rollError, Time.deltaTime);

		if (!realisticSideAdjustment)
			rigidbody.AddRelativeForce(new Vector3(newX, 0, 0), ForceMode.VelocityChange);

		rigidbody.AddTorque(new Vector3(newXRot, newYRot, newZRot), ForceMode.VelocityChange);

		float newZAbs = Mathf.Abs(newZ);
		if (newZ > 0)
		{
			AddEngineForce(BackLeftEngine, newZAbs);
			AddEngineForce(BackRightEngine, newZAbs);
		}
		else if (newZ < 0)
		{
			AddEngineForce(ForwardLeftEngine, newZAbs);
			AddEngineForce(ForwardRightEngine, newZAbs);
		}
		if (realisticSideAdjustment)
		{
			if (newX > 0)
			{
				AddEngineForce(BackRightEngine, -newX);
				AddEngineForce(ForwardRightEngine, -newX);
			}
			else if (newX < 0)
			{
				AddEngineForce(BackLeftEngine, newX);
				AddEngineForce(ForwardLeftEngine, newX);
			}
		}
		float newY = Mathf.Clamp((float)yPid.Update(localDesiredVelocity.y, localVelocity.y, Time.deltaTime), -maxEngineSpeed, maxEngineSpeed);
		AddEngineForce(BackLeftEngine, newY);
		AddEngineForce(ForwardLeftEngine, newY);
		AddEngineForce(BackRightEngine, newY);
		AddEngineForce(ForwardRightEngine, newY);
	}

	private void AddEngineForce(Transform engine, float force)
	{
		rigidbody.AddForceAtPosition(engine.TransformDirection(Vector3.up * force), engine.position, ForceMode.VelocityChange);
		rigidbody.AddForceAtPosition(engine.TransformDirection(Vector3.up * force), engine.position, ForceMode.VelocityChange);
	}

	//Pitch is rotation around x-axis
	//Returns positive if pitching forward
	private float GetPitchError()
	{
		float getXAngle = AngleSigned(transform.up, Vector3.up, Vector3.left);
		return getXAngle;
	}

	//Roll is rotation around z-axis
	//Returns positive if rolling left
	private float GetRollError()
	{
		float getZAngle = AngleSigned(transform.up, Vector3.up, Vector3.back);
		return getZAngle;
	}

	//Roll is rotation around y-axis
	//Returns positive if rolling left
	private float GetYawError()
	{
		float getYAngle = AngleSigned(transform.forward, velocityContext.DesiredDirection, Vector3.down);
		return getYAngle;
	}

	public static float AngleSigned(Vector3 v1, Vector3 v2, Vector3 n)
	{
		return Mathf.Atan2(Vector3.Dot(n, Vector3.Cross(v1, v2)), Vector3.Dot(v1, v2));
	}

	public void AddCalculateDesiredVelocityListener(Action<VelocityContext> action, int priority)
	{
		if (!CalculateWorldDesiredVelocity.ContainsValue(action))
		{
			CalculateWorldDesiredVelocity.Add(priority, action);
		}
		else
		{
			Debug.Log("Listener present");
		}
	}

	public void RemoveCalculateDesiredVelocityListener(Action<VelocityContext> action)
	{
		CalculateWorldDesiredVelocity.RemoveAt(CalculateWorldDesiredVelocity.IndexOfValue(action));
	}

	[Serializable]
	public class VelocityContext
	{
		private Vector3 desiredDirection;
		public Vector3 DesiredWorldVelocity { get; set; }
		public Vector3 DesiredDirection
		{
			get
			{
				if (desiredDirection == Vector3.zero)
				{
					return Vector3.forward;
				}
				return desiredDirection;
			}
			set
			{
				desiredDirection = value;
			}
		}
	}

	public Rigidbody Rigidbody
	{
		get { return rigidbody; }
	}

	public float TurnSpeed
	{
		get { return turnSpeed; }
	}

	public Vector3 DesiredDirection { get { return velocityContext.DesiredDirection; } }
}
