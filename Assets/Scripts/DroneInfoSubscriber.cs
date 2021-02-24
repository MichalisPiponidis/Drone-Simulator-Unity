/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class DroneInfoSubscriber : UnitySubscriber<MessageTypes.Std.Float32MultiArray>
    {
        [SerializeField] private QuadcopterControllerUI controller;

        /*[SerializeField] private Transform drone;
        [SerializeField] private Transform target;
        [SerializeField] private Transform level;*/

        [SerializeField] private float horizontalDistanceROS;
        [SerializeField] private float verticalDistanceROS;
        [SerializeField] private float angleDifferenceROS;
        [SerializeField] private float droneVelocityROS;
        [SerializeField] private float platformSpeedROS;

        private void Awake()
        {
        }

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Std.Float32MultiArray message)
        {
            horizontalDistanceROS = message.data[0];
            verticalDistanceROS = message.data[1];
            angleDifferenceROS = message.data[2];
            platformSpeedROS = message.data[3];
            droneVelocityROS = message.data[4];
            controller.receiveInfo(horizontalDistanceROS, verticalDistanceROS, angleDifferenceROS, droneVelocityROS);
            //Debug.Log("DONE");
            /*drone.SetPositionAndRotation(new Vector3(0, level.position.y + verticalDistance,0),Quaternion.Euler(new Vector3(0, ((360f * (angleDifference + 1)) / 2), 0)));
            Debug.Log("2");
            target.SetPositionAndRotation(new Vector3(horizontalDistance, 1, 0),Quaternion.Euler(new Vector3(0,0,0)));
            Debug.Log("SET");*/
        }

    }
}