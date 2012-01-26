/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.tutorial.image_transport;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.views.RosImageView;
import org.ros.message.sensor_msgs.CompressedImage;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import android.os.Bundle;

/**
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MainActivity extends RosActivity {

  private RosImageView<CompressedImage> image;
  private Talker talker; 
  public MainActivity() {
    super("ImageTransportTutorial", "ImageTransportTutorial");
  }

  @SuppressWarnings("unchecked")
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(org.ros.android.R.layout.main);
    image = (RosImageView<CompressedImage>) findViewById(org.ros.android.R.id.image);
    image.setTopicName("/usb_cam/image_raw/compressed");
    image.setMessageType("sensor_msgs/CompressedImage");
    image.setMessageToBitmapCallable(new BitmapFromCompressedImage());
  }

  /*@Override
  protected void init(NodeMainExecutor nodeMainExecutor) {
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress().toString(), getMasterUri());
    nodeMainExecutor.executeNodeMain(image, nodeConfiguration.setNodeName("android/video_view"));
  }*/

  @Override
  protected void init(NodeRunner nodeRunner) {
    // TODO Auto-generated method stub
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
    java.net.URI master_uri = getMasterUri();
    nodeConfiguration.setMasterUri(master_uri);
    talker = new Talker();
    nodeRunner.run(talker, nodeConfiguration.setDefaultNodeName("pubsub_android"));
    nodeRunner.run(rosTextView, nodeConfiguration);
  }

  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {
    // TODO Auto-generated method stub
    
  }
}
