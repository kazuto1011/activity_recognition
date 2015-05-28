package com.github.kazuto1011.activity_recognition.activity_recognition_client;

import android.content.Intent;
import android.hardware.Camera;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.view.RosTextView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.util.ArrayList;

import de.keyboardsurfer.android.widget.crouton.Crouton;
import de.keyboardsurfer.android.widget.crouton.Style;

public class ActivityRecognitionClient extends RosActivity implements View.OnClickListener,RecognitionListener
{
  // Voice command publishing
  private SpeechPublisher speechPublisher;
  private java.lang.Object mSpeechRecognizer;
  public static TextView voice;
  private Intent intent;

  // 1st-person view publishing
  public ImagePublisher imagePublisher;
  public Preview mPreview;
  public Camera mCamera;
  int numberOfCameras;
  int cameraCurrentlyLocked;
  int defaultCameraId;

  // RosTextView subscriber
  private RosTextView<std_msgs.String> userActivity;
  private RosTextView<std_msgs.String> serverStatus;

  public ScreenCapturer screenCapturer;
  public RelativeLayout relativeLayout;

  // Crouton
  private Crouton crouton;

  public ActivityRecognitionClient() {
    super("Activity recognition client", "Activity recognition client");
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    Log.d("rosjava", "onCreate");
    super.onCreate(savedInstanceState);

    setContentView(R.layout.main);
    relativeLayout = (RelativeLayout) findViewById(R.id.relative);

    voice = (TextView)findViewById(R.id.voice);
    Button button = (Button)findViewById(R.id.button);
    button.setOnClickListener(this);

    mSpeechRecognizer = SpeechRecognizer.createSpeechRecognizer(this);
    ((SpeechRecognizer) mSpeechRecognizer).setRecognitionListener(this);

    // RosTextView
    userActivity = (RosTextView<std_msgs.String>)findViewById(R.id.user_activity);
    userActivity.setTopicName("user_activity");
    userActivity.setMessageType(std_msgs.String._TYPE);
    userActivity.setMessageToStringCallable(new MessageCallable<String, std_msgs.String>() {
      @Override
      public String call(std_msgs.String message) {
        return message.getData();
      }
    });

    // RosTextView
    serverStatus = (RosTextView<std_msgs.String>)findViewById(R.id.server_status);
    serverStatus.setTopicName("server_status");
    serverStatus.setMessageType(std_msgs.String._TYPE);
    serverStatus.setMessageToStringCallable(new MessageCallable<String, std_msgs.String>() {
      @Override
      public String call(std_msgs.String message) {
        return message.getData();
      }
    });

    // Camera preview
    mPreview = new Preview(this);
    relativeLayout.addView(mPreview);

    // Camera info
    numberOfCameras = Camera.getNumberOfCameras();
    Camera.CameraInfo cameraInfo = new Camera.CameraInfo();
    for (int i = 0; i < numberOfCameras; i++) {
      Camera.getCameraInfo(i, cameraInfo);
      if (cameraInfo.facing == Camera.CameraInfo.CAMERA_FACING_BACK) {
        defaultCameraId = i;
      }
    }


  }

  @Override
  protected void onResume() {
    Log.d("rosjava", "onResume");
    super.onResume();
    mCamera = Camera.open();
    cameraCurrentlyLocked = defaultCameraId;
    mPreview.setCamera(mCamera);
  }

  @Override
  protected void onPause() {
    Log.d("rosjava", "onPause");
    super.onPause();
    if (mCamera != null) {
      mPreview.setCamera(null);
      mCamera.setPreviewCallback(null);
      mCamera.stopPreview();
      mCamera.release();
      mCamera = null;
    }
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();
    Crouton.cancelAllCroutons();
  }

  // ROS init
  //------------------------------------------------------------------------------------------------
  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {
    Log.d("rosjava", "init");

    // ROS nodes
    imagePublisher = new ImagePublisher();
    speechPublisher = new SpeechPublisher();
    speechPublisher.setActivity(this);

    screenCapturer = new ScreenCapturer();
    screenCapturer.setLayout(relativeLayout.getRootView());

    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
    nodeConfiguration.setMasterUri(getMasterUri());

    nodeMainExecutor.execute(imagePublisher, nodeConfiguration);
    nodeMainExecutor.execute(speechPublisher, nodeConfiguration);
    nodeMainExecutor.execute(screenCapturer, nodeConfiguration);
    nodeMainExecutor.execute(userActivity, nodeConfiguration.setNodeName("activity_recognition_client/user_activity"));
    nodeMainExecutor.execute(serverStatus, nodeConfiguration.setNodeName("activity_recognition_client/server_status"));
  }

  // onClickListener for speech recognition
  //------------------------------------------------------------------------------------------------
  @Override
  public void onClick(View v) {
    switch (v.getId()) {
      case R.id.button:
        intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, getPackageName());
        ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
        break;
      default:
        break;
    }
  }

  // Speech recognition
  //------------------------------------------------------------------------------------------------
  @Override
  public void onReadyForSpeech(Bundle params) {
    showCrouton("Ready for speech", Style.INFO);
  }

  @Override
  public void onBeginningOfSpeech() {
    showCrouton("Listening...", Style.INFO);
  }

  @Override
  public void onBufferReceived(byte[] buffer) {
    Log.v("DEBUG", "onBufferReceived");
  }

  @Override
  public void onRmsChanged(float rmsdB) {
    Log.v("DEBUG","receive : " + rmsdB + "dB");
  }

  @Override
  public void onEndOfSpeech() {
    showCrouton("End of speech", Style.INFO);
  }

  @Override
  public void onError(int error) {
    switch (error) {
      case SpeechRecognizer.ERROR_AUDIO:
        showCrouton("ERROR_AUDIO", Style.ALERT);
        break;
      case SpeechRecognizer.ERROR_CLIENT:
        showCrouton("ERROR_CLIENT", Style.ALERT);
        break;
      case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
        showCrouton("ERROR_INSUFFICIENT_PERMISSIONS", Style.ALERT);
        break;
      case SpeechRecognizer.ERROR_NETWORK:
        showCrouton("ERROR_NETWORK", Style.ALERT);
        break;
      case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
        showCrouton("ERROR_NETWORK_TIMEOUT", Style.ALERT);
        break;
      case SpeechRecognizer.ERROR_NO_MATCH:
        showCrouton("ERROR_NO_MATCH", Style.ALERT);
        ((SpeechRecognizer) mSpeechRecognizer).cancel();
        ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
        break;
      case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
        showCrouton("ERROR_RECOGNIZER_BUSY", Style.ALERT);
        ((SpeechRecognizer) mSpeechRecognizer).cancel();
        ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
        break;
      case SpeechRecognizer.ERROR_SERVER:
        showCrouton("ERROR_SERVER", Style.ALERT);
        break;
      case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
        showCrouton("ERROR_SPEECH_TIMEOUT(No input)", Style.ALERT);
        ((SpeechRecognizer) mSpeechRecognizer).cancel();
        ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
        break;
      default:
    }
  }

  @Override
  public void onEvent(int eventType, Bundle params) {
    Log.v("DEBUG", "onEvent");
  }

  @Override
  public void onPartialResults(Bundle partialResults) {
    Log.v("DEBUG", "onPartialResults");
  }

  @Override
  public void onResults(Bundle results) {
    ArrayList<String> recData = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
    final String[] str_items = recData.toArray(new String[recData.size()]);

    Log.i("ROS:TmsUrDemo","Send recognized result to server");
    voice.setText(str_items[0]);
    speechPublisher.sendRequest(str_items[0]);
    ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
  }

  // Crouton
  //------------------------------------------------------------------------------------------------
  public void showCrouton(CharSequence message, Style style) {
    /*
    if (crouton != null) {
      crouton.hide();
      crouton = null;
    }
    crouton = Crouton.makeText(this, message, style);
    crouton.show();
    */
    Toast.makeText(this, message, Toast.LENGTH_SHORT).show();
  }
}