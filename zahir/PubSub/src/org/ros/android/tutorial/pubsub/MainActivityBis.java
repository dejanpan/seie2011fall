package org.ros.android.tutorial.pubsub;

import org.ros.RosCore;
import org.ros.address.InetAddressFactory;
import org.ros.android.MessageCallable;
import org.ros.android.views.RosTextView;
import org.ros.node.DefaultNodeRunner;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeRunner;
import org.ros.tutorials.pubsub.R;
import org.ros.tutorials.pubsub.Talker;
import android.app.Activity;
import android.os.Bundle;
	/**
	 * @author damonkohler@google.com (Damon Kohler)
	 */

	public class MainActivityBis extends Activity {

	  private final NodeRunner nodeRunner;
	  private RosCore rosCore;
	  private RosTextView<org.ros.message.std_msgs.String> rosTextView;
	  private Talker talker;

	  public MainActivityBis() {
	    super();
	    nodeRunner = DefaultNodeRunner.newDefault();
	  }

	  @SuppressWarnings("unchecked")
	  @Override

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
	 protected void onPause() {
	    super.onPause();
	    rosCore.shutdown();
	 }

	  @Override
	  protected void onResume() {
	    super.onResume();
	    try {
	      rosCore = RosCore.newPublic("192.168.1.14",11311);
	      NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
	      java.net.URI master_uri = new java.net.URI("http://rim-Satellite-A200:11311");
	      nodeConfiguration.setMasterUri(master_uri);
	      rosCore.awaitStart();
	      talker = new Talker();
	      nodeRunner.run(talker, nodeConfiguration);
	      nodeRunner.run(rosTextView, nodeConfiguration);
	    } catch (Exception e) {
	      throw new RuntimeException(e);
	    }
	  }

	}

