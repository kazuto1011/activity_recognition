package com.github.kazuto1011.activity_recognition.activity_recognition_client;

import android.graphics.Bitmap;
import android.util.Log;
import android.view.View;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import sensor_msgs.CompressedImage;

/**
 * Created by kazuto on 14/12/24.
 */
public class ScreenCapturer extends AbstractNodeMain {

  // CompressedImage publisher
  private Publisher<CompressedImage> publisher;
  private sensor_msgs.CompressedImage img;

  private Time time;
  private int sequenceNumber = 0;

  private View mView;
  private ChannelBufferOutputStream stream  = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("activity_recognition_client/screen_capturer");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    Log.d("rosjava", "onStart");

    NameResolver resolver = connectedNode.getResolver().newChild("moverio");
    publisher = connectedNode.newPublisher(resolver.resolve("screen/compressed"), sensor_msgs.CompressedImage._TYPE);

    connectedNode.executeCancellableLoop(new CancellableLoop() {
      @Override
      protected void setup()
      {
        Log.d("rosjava", "onStart");
      }

      @Override
      protected void loop() throws InterruptedException {
        Log.d("rosjava", "onStart");
        Thread.sleep(500);

        img = publisher.newMessage();

        // message setting
        time = connectedNode.getCurrentTime();
        img.getHeader().setStamp(time);
        img.getHeader().setFrameId("moverio");
        img.getHeader().setSeq(sequenceNumber);
        img.setFormat("jpg");

        mView.setDrawingCacheEnabled(false);
        mView.setDrawingCacheEnabled(true);

        Bitmap bitmap0 = Bitmap.createBitmap(mView.getDrawingCache());
        bitmap0.compress(Bitmap.CompressFormat.JPEG, 100, stream);
        img.setData(stream.buffer().copy());
        stream.buffer().clear();
        publisher.publish(img);

        sequenceNumber++;
      }
    });
  }

  public void setLayout(View view) {
    mView = view;
  }

  @Override
  public void onShutdown(Node node) {
    Log.d("rosjava", "onShutdown");
  }
}
