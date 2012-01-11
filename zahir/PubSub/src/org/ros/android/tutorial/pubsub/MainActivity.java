package org.ros.android.tutorial.pubsub;

import org.ros.address.InetAddressFactory;
import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.views.RosTextView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeRunner;
import org.ros.tutorials.pubsub.R;
import org.ros.tutorials.pubsub.Talker;
import android.os.Bundle;
/**
 * @author damonkohler@google.com (Damon Kohler)
 */

public class MainActivity extends RosActivity {

  private RosTextView<org.ros.message.std_msgs.String> rosTextView;
  private Talker talker;

  public MainActivity() {
    super("PubSub", "PubSub");
    
  }
  @SuppressWarnings("unchecked")
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);

    rosTextView = (RosTextView<org.ros.message.std_msgs.String>) findViewById(R.id.text);
    rosTextView.setTopicName("/chatter");
    rosTextView.setMessageType("std_msgs/String");
    rosTextView.setMessageToStringCallable(new MessageCallable<String, org.ros.message.std_msgs.String>() {
      public String call(org.ros.message.std_msgs.String message) {
        return message.data;
      }
    });
  }

  @Override
  protected void init(NodeRunner nodeRunner) {
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
    java.net.URI master_uri = getMasterUri();
    nodeConfiguration.setMasterUri(master_uri);
    talker = new Talker();
    nodeRunner.run(talker, nodeConfiguration.setDefaultNodeName("pubsub_android"));
    nodeRunner.run(rosTextView, nodeConfiguration);
  }
}

