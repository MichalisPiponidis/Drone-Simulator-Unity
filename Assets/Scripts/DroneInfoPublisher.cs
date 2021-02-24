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

using RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class DroneInfoPublisher : UnityPublisher<MessageTypes.Std.Int32MultiArray>
    {

        private MessageTypes.Std.Int32MultiArray message;
        private MultiArrayLayout layout;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void InitializeMessage()
        {
            
            layout = new MultiArrayLayout();
            //message.data.Initialize();
        }

        public void sendDroneCommands(int pitch, int roll, int throttle, int yaw, int landing)
        {
            //update message
            int[] data = new int[5];
            data[0] = pitch;
            data[1] = roll;
            data[2] = throttle;
            data[3] = yaw;
            data[4] = landing;
            message = new MessageTypes.Std.Int32MultiArray(layout, data);

            //send message
            Publish(message);
        }

    }
}