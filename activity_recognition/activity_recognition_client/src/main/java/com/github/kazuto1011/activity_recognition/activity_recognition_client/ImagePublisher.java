package com.github.kazuto1011.activity_recognition.activity_recognition_client;

import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.util.Log;

import com.google.common.base.Preconditions;

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
 * Created by kazuto on 14/12/10.
 */
public class ImagePublisher extends AbstractNodeMain{
  // to convert yuv byte data to RGB data
  private byte[] rawImageBuffer;
  private Camera.Size rawImageSize;
  private YuvImage yuvImage;
  private Rect rect;
  private ChannelBufferOutputStream stream  = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());

  // CompressedImage publisher
  private Publisher<CompressedImage> publisher;
  private sensor_msgs.CompressedImage img;

  private Time time;
  private long last_time, current_time;
  private int sequenceNumber = 0;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("activity_recognition_client/image_publisher");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    Log.d("rosjava", "onStart");

    NameResolver resolver = connectedNode.getResolver().newChild("moverio");
    publisher = connectedNode.newPublisher(resolver.resolve("camera/compressed"), sensor_msgs.CompressedImage._TYPE);

    // like a while(ros::ok())
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      @Override
      protected void setup()
      {
        Log.d("rosjava", "setup");
        try {
          Thread.sleep(2000);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        last_time = System.nanoTime()/1000000;
      }

      @Override
      protected void loop() throws InterruptedException {
        if (Preview.yuv_data != null) {
          if (Preview.yuv_data != rawImageBuffer || !Preview.mPreviewSize.equals(rawImageSize)) {
            rawImageBuffer = Preview.yuv_data;
            rawImageSize = Preview.mPreviewSize;
          }
          // new rosjava message
          img = publisher.newMessage();

          // message setting
          time = connectedNode.getCurrentTime();
          img.getHeader().setStamp(time);
          img.getHeader().setFrameId("moverio");
          img.getHeader().setSeq(sequenceNumber);
          img.setFormat("jpg");

          // make yuv compressed to jpeg
          yuvImage = new YuvImage(rawImageBuffer, ImageFormat.NV21, rawImageSize.width, rawImageSize.height, null);
          rect = new Rect(0, 0, rawImageSize.width, rawImageSize.height);
          Preconditions.checkState(yuvImage.compressToJpeg(rect, 20, stream));
          img.setData(stream.buffer().copy());
          stream.buffer().clear();

          // publish message
          current_time = System.nanoTime() / 1000000;
          publisher.publish(img);

          if ((current_time - last_time) < 23 && (current_time - last_time) > 0) {
            Thread.sleep(23 - (current_time - last_time));
          }

          last_time = current_time;
          sequenceNumber++;
        }
      }
    });
  }

  @Override
  public void onShutdown(Node node) {
    Log.d("rosjava", "onShutdown");
  }
}
