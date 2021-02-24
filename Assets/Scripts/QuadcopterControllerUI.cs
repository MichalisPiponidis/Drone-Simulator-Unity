using RosSharp.RosBridgeClient;
using System.Collections;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.UI;

public class QuadcopterControllerUI : Agent
{
	public DroneInfoPublisher publisher;

	[Header("Toggles")]
	public Toggle ROSToggle;
	public Toggle manualToggle;

	[Header("Sprites")]
	public Sprite blackArrow;
	public Sprite grayArrow;
	public Sprite blackTurnArrow;
	public Sprite grayTurnArrow;

	[Header("Texts")]
	public Text heightText;
	public Text distanceText;
	public Text angleText;
	public Text landingText;

	[Header("Images")]
	public Image forwardImage;
	public Image backwardImage;
	public Image leftImage;
	public Image rightImage;
	public Image upImage;
	public Image downImage;
	public Image turnLeftImage;
	public Image turnRightImage;

	public override void CollectObservations(VectorSensor sensor)
	{
		sensor.AddObservation(angleDifferenceReceived/180f);
		//Speed
		sensor.AddObservation((horizontalDistanceReceived + 2.5f)/30f);
		//Altitude
		sensor.AddObservation(verticalDistanceReceived/7f);
	}

	IEnumerator Wait(float duration)
	{
		yield return new WaitForSeconds(duration);
		waiting = false;
	}

	private int direction, altitude, speed, side;
	private bool waiting = false;
	public override void OnActionReceived(float[] vectorAction)
	{
		if (manualToggle.isOn) //Manual Controls
		{
			vectorAction = AddControllsManual();
		}
		else if (landingProcedure) // Landing procedure
		{
			if (verticalDistanceReceived <= land)
            {
				landing = 1;
            }
			vectorAction = AddControllsFromObservations();
		}

		direction = Mathf.FloorToInt(vectorAction[0]);
		altitude = Mathf.FloorToInt(vectorAction[1]);
		speed = Mathf.FloorToInt(vectorAction[2]);
		side = Mathf.FloorToInt(vectorAction[3]);

		//SEND ROS COMMANDS
		if (ROSToggle.isOn)
		{
			//Landing
			if (verticalDistanceReceived <= startLandingProcedure && !landingProcedure)
            {
				landingProcedure = true;
				waiting = true;
				StartCoroutine(Wait(3));
				landingText.text = "LANDING!";
			}
			if (!waiting) // Dont send commands while waiting
			{
				publisher.sendDroneCommands(speed, side, altitude, direction, landing);
			}
		}

		switch (direction)
		{
			case 0:
				turnLeftImage.sprite = grayTurnArrow;
				turnRightImage.sprite = grayTurnArrow;
				break;
			case 1:
				turnLeftImage.sprite = blackTurnArrow;
				turnRightImage.sprite = grayTurnArrow;
				break;
			case 2:
				turnLeftImage.sprite = grayTurnArrow;
				turnRightImage.sprite = blackTurnArrow;
				break;
		}
		switch (altitude)
		{
			case 0:
				upImage.sprite = grayArrow;
				downImage.sprite = grayArrow;
				break;
			case 1:
				upImage.sprite = blackArrow;
				downImage.sprite = grayArrow;
				break;
			case 2:
				upImage.sprite = grayArrow;
				downImage.sprite = blackArrow;
				break;
		}
		switch (speed)
		{
			case 0:
				forwardImage.sprite = grayArrow;
				backwardImage.sprite = grayArrow;
				break;
			case 1:
				forwardImage.sprite = blackArrow;
				backwardImage.sprite = grayArrow;
				break;
			case 2:
				forwardImage.sprite = grayArrow;
				backwardImage.sprite = blackArrow;
				break;
		}
		switch (side)
		{
			case 0:
				leftImage.sprite = grayArrow;
				rightImage.sprite = grayArrow;
				break;
			case 1:
				leftImage.sprite = blackArrow;
				rightImage.sprite = grayArrow;
				break;
			case 2:
				leftImage.sprite = grayArrow;
				rightImage.sprite = blackArrow;
				break;
		}
	}

	[Header("Observations Received")]
	public float horizontalDistanceReceived;
	public float verticalDistanceReceived;
	public float angleDifferenceReceived;
	public float droneVelocityReceived;
	private bool receivedInfo = false;

	[Header("Info Sent")]
	public int landing = 0;
	private bool landingProcedure = false;

	public void receiveInfo(float horizontalDistanceR, float verticalDistanceR, float angleDifferenceR, float droneVelocityR)
	{
		horizontalDistanceReceived = horizontalDistanceR;
		verticalDistanceReceived = verticalDistanceR;
		angleDifferenceReceived = angleDifferenceR;
		droneVelocityReceived = droneVelocityR;
		heightText.text = "Height: " + verticalDistanceReceived + "m";
		distanceText.text = "Distance: " + horizontalDistanceReceived + "m";
		angleText.text = "Angle: " + angleDifferenceReceived + "°";
		receivedInfo = true;
	}

	public override void OnEpisodeBegin()
	{
		landingProcedure = false;
		landing = 0;
	}

	public override void Heuristic(float[] actionsOut)
	{
		actionsOut[0] = 0f;
		actionsOut[1] = 0f;
		actionsOut[2] = 0f;
		actionsOut[3] = 0f;
		AddControlls(actionsOut);
	}
	//ML END

	private void FixedUpdate()
	{
		if (receivedInfo)
        {
			receivedInfo = false;
			RequestDecision();
        }
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
		if (Input.GetKey(KeyCode.Q))
		{
			//turnLeft();
			actionsOut[0] = 1f;
		}
		if (Input.GetKey(KeyCode.E))
		{
			//turnRight();
			actionsOut[0] = 2f;
		}
		if (Input.GetKey(KeyCode.A))
		{
			//goLeft();
			actionsOut[3] = 1f;
		}
		if (Input.GetKey(KeyCode.D))
		{
			//goRight();
			actionsOut[3] = 2f;
		}
	}

	private float[] AddControllsManual()
	{
		float[] actionsOut = new float[] {0, 0, 0, 0};

		if (Input.GetKey(KeyCode.Space))
		{
            actionsOut[1] = 1f;
		}
		else if (Input.GetKey(KeyCode.LeftShift))
		{
			actionsOut[1] = 2f;
		}

		if (Input.GetKey(KeyCode.W))
		{
			actionsOut[2] = 1f;
		}
		else if (Input.GetKey(KeyCode.S))
		{
			actionsOut[2] = 2f;
		}

		if (Input.GetKey(KeyCode.Q))
		{
			actionsOut[0] = 1f;
		}
		else if (Input.GetKey(KeyCode.E))
		{
			actionsOut[0] = 2f;
		}

		if (Input.GetKey(KeyCode.A))
		{
			actionsOut[3] = 1f;
		}
		else if (Input.GetKey(KeyCode.D))
		{
			actionsOut[3] = 2f;
		}

		return actionsOut;
	}

	[Header("Limits")]
	public float horizontalDistanceLimit;
	public float startLandingProcedure;
	public float land;

	private float[] AddControllsFromObservations()
	{
		float[] actionsOut = new float[] { 0, 0, 0, 0 };

		actionsOut[1] = 2f; //down

		if (horizontalDistanceReceived >= horizontalDistanceLimit)
		{
			actionsOut[2] = 1f; //forward
		}

		if (angleDifferenceReceived > 10) //turn left
		{
			actionsOut[0] = 1f;
		}
		else if (angleDifferenceReceived < -10)
		{
			actionsOut[0] = 2f; //turn right
		}

		return actionsOut;
	}
}