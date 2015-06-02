package com.github.kazuto1011.activity_recognition.activity_recognition_client;

import android.app.Activity;
import android.util.Log;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import activity_recognition.user_voice;
import activity_recognition.user_voiceRequest;
import activity_recognition.user_voiceResponse;
import de.keyboardsurfer.android.widget.crouton.Crouton;
import de.keyboardsurfer.android.widget.crouton.Style;

/**
 * Created by kazuto on 14/12/12.
 */
public class SpeechPublisher extends AbstractNodeMain {
  private ServiceClient<user_voiceRequest, user_voiceResponse> serviceClient;
  private Crouton crouton;
  private Activity activity;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("activity_recognition_client/speech_publisher");
  }

  // Create a new client & connect servers
  //------------------------------------------------------------------------------------------------
  @Override
  public void onStart(final ConnectedNode connectedNode) {
    try {
      serviceClient = connectedNode.newServiceClient("user_voice_command", user_voice._TYPE);
    } catch (ServiceNotFoundException e) {
      Log.i("SpeechPublisher", "unable to connect the server");
      throw new RosRuntimeException(e);
    }
  }

  // Set the activity using on Crouton
  //------------------------------------------------------------------------------------------------
  public void setActivity(Activity parent) {
    activity = parent;
  }

  // Call the service
  //------------------------------------------------------------------------------------------------
  public void sendRequest(String message) {
    final user_voiceRequest request = serviceClient.newMessage();
    request.setText(message);

    serviceClient.call(request, new ServiceResponseListener<user_voiceResponse>() {
      @Override
      public void onSuccess(final user_voiceResponse srvResponse) {
        Log.i("SpeechPublisher", "succeeded to call service");
        //showCrouton("Succeed to call the service", Style.INFO);
      }
      @Override
      public void onFailure(RemoteException e) {
        Log.i("SpeechPublisher", "failed to call service");
        //showCrouton("Failed to call the service", Style.ALERT);
      }
    });
  }


  // Crouton
  //------------------------------------------------------------------------------------------------
  public void showCrouton(CharSequence message, Style style) {
    /*
    if (crouton != null) {
      crouton.hide();
      crouton = null;
    }
    crouton = Crouton.makeText(activity, message, style);
    crouton.show();
    */
    //Toast.makeText(activity, message, Toast.LENGTH_SHORT).show();
  }
}
